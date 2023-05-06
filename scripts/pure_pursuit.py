#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, ColorRGBA
from sensor_msgs.msg import Joy
from visual_servoing.msg import ConeLocation, ConeLocationPixel

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic","/pf/pose/odom")
        self.speed            = 3.0 #filled in for testing purposes, please update
        self.wheelbase_length = 0.32 #flilled in for testing purposes, please update

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.drive_pub2 = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)

        self.cterr_pub = rospy.Publisher("/crosstrackerror", Float64,queue_size=1)

        self.cone_sub = rospy.Subscriber("/relative_cone", ConeLocation, self.cone_callback, queue_size = 20)

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        # self.current_location = np.array([0,0])
        # self.x = self.current_location[0]
        # self.y = self.current_location[1]
        # self.theta = 0

        self.brake = False # Boolean condition to determine whether to stop
        # self.thresh = self.speed # distance from final path point at which car will stop
        self.teleop_subscriber = rospy.Subscriber("/vesc/joy", Joy, self.tcb, queue_size=1)        
        self.pressed = False	

    def tcb(self, msg):
        buttons = msg.buttons
        if(buttons[5] == 1):
            self.pressed = True
        else:
            self.pressed = False

    def cone_callback(self, msg):
        x = msg.x_pos
        y = msg.y_pos
        self.drive_command(x, y)

    def drive_command(self, goalx, goaly):
        #rospy.logerr("x: {}, y: {}".format(self.x, self.y))
        #rospy.logerr("goalx: {}, goaly: {}".format(goalx, goaly))
        #ospy.logerr("a")

        eta = np.arctan2(goaly, goalx)
        # R = self.lookahead / (2 * np.sin(eta))
        AckermannDrive = AckermannDriveStamped()
        AckermannDrive.header.stamp = rospy.Time.now()
        AckermannDrive.header.frame_id = "base_link"
        if (self.brake):
            rospy.logerr("STOPPING)")
            AckermannDrive.drive.speed = 0
            AckermannDrive.drive.steering_angle = 0
        else:
            rospy.logerr("sending drive signal")
            AckermannDrive.drive.speed = self.speed
            AckermannDrive.drive.steering_angle = np.arctan2(2 * self.wheelbase_length * np.sin(eta), np.sqrt(goalx**2 + goaly**2))

        #generalized sttering law by having a point ahead lecture slides
        # lfw = 0.05 #randomly defined based on lecture slides
        # AckermannDrive.drive.steering_angle = -1 * np.arctan(self.wheelbase_length * np.sin(eta) / (self.lookahead / 2  + lfw/np.cos(eta)))
        if (self.pressed):
            self.drive_pub2.publish(AckermannDrive)
        self.drive_pub.publish(AckermannDrive)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
