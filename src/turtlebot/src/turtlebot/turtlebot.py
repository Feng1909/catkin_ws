# !/usr/bin/python3


import rospy
from geometry_msgs.msg import PoseStamped, Twist
import math
import os
import numpy as np
from math import fabs, cos, sin, hypot, pi, atan2, acos, sqrt
from scipy import interpolate
import casadi as ca
import yaml
from easydict import EasyDict as edict
from turtlebot.type import State, ControlCommand


# from tf.transformations import euler_from_quaternion

class TurtleBotException(Exception):
    pass



class TurtleBot:
    def __init__(self):
        if not rospy.has_param('turtlebot_config'):
            raise TurtleBotException('No turtlebot configuration found')
        self.config = rospy.get_param('turtlebot_config')

        # initiate subscribers
        rospy.Subscriber('/vrpn_client_node/TurtleBot3/pose', PoseStamped,
                         self.state_callback)
        
        self.pub = rospy.Publisher('/state', PoseStamped)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist)
        
        self.pose = PoseStamped()
        '''
        orientation: 
        x: v
        y: theta
        z: omega
        '''

        self.path = []

        self.load_map()
        self.state = {}
        self.state['time_stamp'] = 0.0
        self.state['x'] = 0
        self.state['y'] = 0
        self.state['v'] = 0
        self.state['theta'] = 0
        self.state['omega'] = 0

        with open('bot.yaml') as yamlfile:
            cfgs = yaml.load(yamlfile, Loader=yaml.FullLoader)
            cfgs = edict(cfgs)
        self.cfg = cfgs
        self.N = self.cfg.MPC.N
        self.dt = self.cfg.MPC.dt
        self.L = self.cfg.model.L
        self.max_a = self.cfg.model.max_a
        self.max_v = self.cfg.model.max_v
        self.ref_path = []
        self.pre_path = []
        self.cmd = ControlCommand(0, 0)
        self.state = State()

        self.opti = ca.Opti()
        # ul, ur
        self.opt_controls = self.opti.variable(self.N, 2)
        self.ul = self.opt_controls[:, 0]
        self.ur = self.opt_controls[:, 1]
        self.cmd_all = np.zeros((self.N, 2))
        # x, y, theta, v, omega
        self.opt_states = self.opti.variable(self.N+1, 5)
        self.pos_x = self.opt_states[:, 0]
        self.pos_y = self.opt_states[:, 1]
        self.pos_theta = self.opt_states[:, 2]
        self.pos_v = self.opt_states[:, 3]
        self.pos_omega = self.opt_states[:, 4]
        # ref_path
        self.ref_path = self.opti.parameter(self.N, 5)
        # kinetic model
        self.f = lambda x, u: ca.vertcat(*[x[3]*np.cos(x[2]),
                                           x[3]*np.sin(x[2]),
                                           x[4],
                                           (u[0]+u[1])/2,
                                           (u[1]-u[0])/self.L])

        # init condition
        self.opti.subject_to(self.opt_states[0, :] == self.ref_path[0, :])
        for k in range(self.N): # loop over control intervals
            # Runge-Kutta 4 integration
            k1 = self.f(self.opt_states[k, :],              self.opt_controls[k, :])
            k2 = self.f(self.opt_states[k, :]+self.dt/2*k1.T, self.opt_controls[k, :])
            k3 = self.f(self.opt_states[k, :]+self.dt/2*k2.T, self.opt_controls[k, :])
            k4 = self.f(self.opt_states[k, :]+self.dt*k3.T,   self.opt_controls[k, :])
            x_next = self.opt_states[k, :] + self.dt*(k1/6.+k2/3.+k3/3.+k4/6.).T
            self.opti.subject_to(self.opt_states[k+1, :] == x_next)
        
        # cost function
        # some addition parameters
        self.Q = np.diag([self.cfg.MPC.state.x,
                          self.cfg.MPC.state.y,
                          self.cfg.MPC.state.theta,
                          self.cfg.MPC.state.v,
                          self.cfg.MPC.state.omega])
        self.R = np.diag([self.cfg.MPC.input.ul,
                          self.cfg.MPC.input.ur])
        self.obj = 0 # cost
        for i in range(self.N):
            self.obj = self.obj + ca.mtimes([(self.opt_states[i, :]-self.ref_path[i, :]),
                                              self.Q,
                                              (self.opt_states[i, :]-self.ref_path[i, :]).T]) + \
                                  ca.mtimes([(self.opt_controls[i, :]), self.R, (self.opt_controls[i, :]).T])
        
        self.opti.minimize(self.obj)
        self.opti.subject_to(self.opti.bounded(0, self.pos_v, self.max_v))
        self.opti.subject_to(self.opti.bounded(-self.max_a, self.ul, self.max_a))
        self.opti.subject_to(self.opti.bounded(-self.max_a, self.ur, self.max_a))

        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}

        self.opti.solver('ipopt', opts_setting)


        ## test mpc once
        ref_v = 0.5
        next_states = np.zeros((self.N+1, 5))
        for i in range(self.N):
            x = i
            state = [x,x,0,ref_v,0]
            self.opti.set_value(self.ref_path[i, :], state)
        self.opti.set_initial(self.opt_controls, np.zeros((self.N, 2)))
        self.opti.set_initial(self.opt_states, next_states)
        sol = self.opti.solve()

        rate = rospy.Rate(10)

        while(not rospy.is_shutdown()):
            self.pub.publish(self.pose)
            self.run_cmd()
            rate.sleep()
        rospy.spin()

    def state_callback(self, msg):
        next_pose = PoseStamped()
        next_pose.pose.position.x = msg.pose.position.x
        next_pose.pose.position.y = msg.pose.position.z

        if self.timestamp == 0.0:
            self.timestamp = msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec
        delta_time = msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec - self.timestamp
        self.timestamp = msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec

        w = msg.pose.orientation.w
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z

        matrix_c = 1-2*y*y-2*z*z
        matrix_s = 2*x*z-2*w*y
        theta = atan2(-matrix_s, matrix_c)/3.141592653*180

        delta_s = hypot(self.state['x'] - next_pose.pose.position.x, self.state['y'] - next_pose.pose.position.y)
        delta_theta = theta - self.state['theta']
        v = delta_s/delta_time
        omega = delta_theta/delta_time

        next_pose.pose.orientation.x = v
        next_pose.pose.orientation.y = theta
        next_pose.pose.orientation.z = omega

        self.pose = next_pose

        self.diff = 0

    def get_path(self):
        x = self.state['x']
        y = self.state['y']
        min_now = 9999
        index = -1
        num = 0
        a_1 = []
        a_2 = []
        b = [x, y]
        for i in self.path:
            if hypot(x-i[0], y-i[1]) < min_now:
                min_now = hypot(x-i[0], y-i[1])
                index = num
                a_2 = a_1
                a_1 = [i[0], i[1]]
            num += 1

        # cal l#
        if a_2 != []:
            a1_b = [b[0]-a_1[0], b[1]-a_1[1]]
            a1_a2 = [a_2[0]-a_1[0], a_2[1]-a_1[1]]
            self.l = abs(hypot(a1_b[0], a1_b[1])) * sin(acos(abs(
                (a1_a2[0]*a1_b[0]+a1_a2[1]*a1_b[1]) /
                (hypot(a1_a2[0], a1_a2[1])*hypot(a1_b[0], a1_b[1]))
            )))
        else:
            self.l = hypot(a_1[0]-b[0], a_1[1]-b[1])
        self.diff += self.l
        # print(self.l)

        if index == -1:
            cmd = Twist()
            cmd.twist.linear.x = 0
            cmd.twist.angular.y = 0
            self.cmd_pub.publish(cmd)
            raise Exception('find location error')
        
        if index+1+self.cfg.ref_ahead > len(self.path):
            cmd = Twist()
            cmd.twist.linear.x = 0
            cmd.twist.angular.y = 0
            self.cmd_pub.publish(cmd)
            print('diff: ', self.diff)
            raise Exception('mission finished')        
        path_next = self.path[index+1: index+1+self.cfg.ref_ahead]      
        path_return = []
        for i in path_next:
            x1 = i[0]-x
            y1 = i[1]-y
            D = hypot(x1, y1)
            Phi = atan2(y1, x1)-self.state['theta']
            theta = i[2] - self.state['theta']
            while theta < -pi:
                theta += 2*pi
            while theta > pi:
                theta -= 2*pi
            path_return.append([D*cos(Phi),
                                D*sin(Phi),
                                theta,
                                i[3],
                                i[4]])
        return path_return

    def run_cmd(self):
        # 路径在self.path中[x, y, theta, d_theta, dd_theta]
        desire_v = self.cfg.MPC.desire_v
        init_state = []
        init_state.append([0, 0, 0, self.state['v'], self.state['omega']])
        stot = 0
        v_old = self.state.v
        self.path_local = self.get_path()
        for i in range(self.N):
            path_ref = self.path_local[stot]

            k = abs(path_ref[4])/pow(sqrt(1+pow(path_ref[3], 2)), 3)
            r = 1/k
            v = max(min(min(desire_v, v_old+self.cfg.model.max_a*self.dt), sqrt(self.cfg.model.max_a*r)), 0.1)
            
            state = []
            for j in path_ref[:3]:
                state.append(j)
            state.append(v)
            state.append(path_ref[3])
            if i == 0:
                state = [0, 0, 0, self.state['v'], self.state['omega']]
            init_state.append([state[0],
                               state[1],
                               state[2],
                               state[3],
                               state[4]])
            self.opti.set_value(self.ref_path[i, :], state)
            stot += int(self.dt*v*100)
            v_old = v
        self.opti.set_initial(self.opt_controls, self.cmd_all)
        init_state = np.array(init_state)
        self.opti.set_initial(self.opt_states, init_state)
        sol = self.opti.solve()
        self.cmd_all = sol.value(self.opt_controls)
        [self.cmd.u_l, self.cmd.u_r] = self.cmd_all[0]  
        
        v_l = self.state['v'] - self.cfgs.model.L * self.state['omega'] / 2 + self.cmd.u_l
        v_r = self.state['v'] + self.cfgs.model.L * self.state['omega'] / 2 + self.cmd.u_r

        cmd_omega = (v_r - v_l) / self.cfg.model.L
        cmd = Twist()
        cmd.twist.linear.x = (v_l+v_r)/2/0.28
        cmd.twist.angular.y = cmd_omega
        self.cmd_pub.publish(cmd)


    def load_map(self):
        if not os.path.exists('spline.csv'):
            raise Exception("map do not exist")
        else:
            with open('spline.csv', 'r') as f:
                pathes = f.readlines()
            if len(pathes) == 0:
                raise Exception("path empty!")
            x = []
            y = []
            for i in pathes:
                i = i.replace('\n', '')
                i = i.replace(' ', '')
                coordinates = i.split(',')
                x.append(float(coordinates[0]))
                y.append(float(coordinates[1]))
            x = np.array(x)
            y = np.array(y)
            # get approximate length of track
            stot = 0
            for i in range(x.size-1):
                stot += hypot(x[i+1]-x[i], y[i+1]-y[i])
            stot = (stot//0.01)*0.01
            N = int(stot/0.01)
            unew = np.arange(0, 1.0, 1.0/N)
            # center
            # Find the B-spline representation of an N-D curve
            tck, u = interpolate.splprep([x,y], s=0)
            # Evaluate a B-spline or its derivatives.
            out = interpolate.splev(unew, tck)
            f_x = out[0]
            f_y = out[1]
            # get approximate length of track
            stot = 0
            for i in range(len(out[0])-1):
                stot += hypot(out[0][i+1]-out[0][i], out[1][i+1]-out[1][i])
            N = int(stot/0.01)
            unew = np.arange(0, 1.0, 1.0/N)
            # center
            # Find the B-spline representation of an N-D curve
            tck, u = interpolate.splprep([x,y], s=0)
            # Evaluate a B-spline or its derivatives.
            out = interpolate.splev(unew, tck)
            f_x = out[0]
            f_y = out[1]
            
            # set psic
            dx = np.diff(f_x)
            dy = np.diff(f_y)
            theta = np.arctan2(dy, dx)
            psic_final = np.arctan2(f_y[0]-f_y[-1], f_x[0]-f_x[-1])
            theta = np.append(theta, psic_final)

            for j in range(len(theta)-1):
                while(theta[j] - theta[j+1] > np.pi):
                    theta[j+1] = theta[j+1] + 2*np.pi
                    
                while(theta[j] - theta[j+1] <= -np.pi):
                    theta[j+1] = theta[j+1] - 2*np.pi

            s = np.arange(0, stot-0.01, 0.01)
            psic = theta[:len(s)]
            t, c, k = interpolate.splrep(s, psic, s=0, k=4)
            psic_spl = interpolate.BSpline(t, c, k, extrapolate=False)
            kappac_spl_one = psic_spl.derivative(nu=1)
            kappac_spl_one = kappac_spl_one(s)
            kappac_spl_two = psic_spl.derivative(nu=2)
            kappac_spl_two = kappac_spl_two(s)

            for i in range(len(f_x)):
                self.path.append([f_x[i], f_y[i], theta[i], kappac_spl_one[i], kappac_spl_two[i]])
