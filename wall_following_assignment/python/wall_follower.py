#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from dynamic_reconfigure.server import Server
from wall_following_assignment.cfg import followerConfig

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt

        self.last_integral = 0

    def update_control(self, current_error, reset_prev=False):
        # todo: implement this
        #self.control = ???

        integral_gain = self.last_integral

        proportional_gain = self.Kp*current_error
        integral_gain += self.Ti * current_error *self.dt
        derivative_gain = self.Td * (current_error - self.prev_error) / self.dt

        #update value
        self.control = proportional_gain + integral_gain + derivative_gain
        self.prev_error = current_error
        self.last_integral = integral_gain

        return self.control

    def get_control(self):
        return self.control

class WallFollowerHusky:
    def __init__(self):
        rospy.init_node('wall_follower_husky', anonymous=True)

        server = Server(followerConfig, self.server_callback)
        self.forward_speed = rospy.get_param("/wall_follower/forward_speed")
        self.desired_distance_from_wall = rospy.get_param("/wall_follower/desired_distance_from_wall")
        #self.forward_speed = 1
        #self.desired_distance_from_wall = 1
        self.hz = 50

        # todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
        # using geometry_msgs.Twist messages
        # self.cmd_pub = ??
        self.cmd_pub = rospy.Publisher('/husky_1/cmd_vel',Twist, queue_size=10)

        # todo: set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spinOnce() call, as long as a laser scan
        # message has been published in the meantime by another node
        # self.laser_sub = ???
        self.laser_sub = rospy.Subscriber('/husky_1/scan',LaserScan,self.laser_scan_callback)

        self.pid_controller=PID(1.3,3, 0.01, 0.2)

    def server_callback(self, config, level):
        rospy.loginfo("Config set to {Kp}, {Ti}, {Td}, {dt}".format(**config))
	    self.Kp = config.Kp
        self.Ti = config.Ti
        self.Td = config.Td
        self.dt = config.dt
        rospy.loginfo(self.Kp)
        print(config)
        print("*******************")
        print(level)
        return config

    def laser_scan_callback(self, msg):
        # todo: implement this
        # Populate this command based on the distance to the closest
        # object in laser scan. I.e. compute the cross-track error
        # as mentioned in the PID slides.

        #compute the cross check error
        left_side = len(msg.ranges)/2 #get the left hand side of the scan range
        cross_check_error = min(msg.ranges[:left_side])- self.desired_distance_from_wall
        error_msg = 'cross-track error : %s' % cross_check_error
        print(error_msg)

        error_publisher = rospy.Publisher('/husky_1/cte',String, queue_size=100)
        error_publisher.publish(error_msg)


        # You can populate the command based on either of the following two methods:
        # (1) using only the distance to the closest wall
        # (2) using the distance to the closest wall and the orientation of the wall
        #
        # If you select option 2, you might want to use cascading PID control.

        # cmd.angular.z = ???

        angle = self.pid_controller.update_control(cross_check_error)
        print ('updated control: %s' % angle)
        linear = Vector3(self.forward_speed,0,0)
        angular = Vector3(0,0,angle)

        twist = Twist(linear,angular)
        self.cmd_pub.publish(twist)

        print('*********************************************')


    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()
