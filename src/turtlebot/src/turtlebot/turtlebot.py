# !/usr/bin/python3


import rospy
from geometry_msgs.msg import PoseStamped
import math
import os
import numpy as np
from math import fabs, cos, sin, hypot, pi, atan2, acos
from scipy import interpolate
import casadi


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
        
        self.pose = PoseStamped()
        '''
        orientation: 
        x: v
        y: theta
        z: omega
        '''

        self.path = []

        self.load_map()

    def state_callback(self, msg):
        next_pose = PoseStamped()
        next_pose.pose.position.x = msg.pose.position.x
        next_pose.pose.position.y = msg.pose.position.z

        v = hypot(self.pose.pose.position.x-next_pose.pose.position.x, self.pose.pose.position.y-next_pose.pose.position.y)/0.02

        w = msg.pose.orientation.w
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        angleR = r
        angleP = p
        angleY = y
        # angleR = r*180/math.pi
        # angleP = p*180/math.pi
        # angleY = y*180/math.pi

        theta = angleP
        omega = (theta-self.pose.pose.orientation.y)/0.02

        next_pose.pose.orientation.x = v
        next_pose.pose.orientation.y = theta
        next_pose.pose.orientation.z = omega

        self.pose = next_pose

	
        self.pub.publish(next_pose)

    def find_path(self):
        min_ = 9999
        index = -1
        for i in self.path:
            index += 1
            if hypot(self.pose.position.x - i[0], self.pose.position.y - i[1]) < min_:
                min_ = hypot(self.pose.position.x - i[0], self.pose.position.y - i[1])
            else:
                self.path = self.path[index:]

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
