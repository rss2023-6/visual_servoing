#!/usr/bin/env python2

import numpy as np
import math
import rospy
from rospy.numpy_msg import numpy_msg
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    DRIVE_TOPIC = rospy.get_param("/vesc/ackermann_cmd_mux/input/navigation")
    ERROR_TOPIC = rospy.get_param("/lane_error")
    history_dist = 0
    history_para = 0
    ierr_para = 0
    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)  
        self.error_sub = rospy.Subscriber(self.ERROR_TOPIC, Float32, self.error_cb) 
    
    def error_cb(self,msg):
        
        solution_angle, solution_speed = self.compute_drive(msg.data)
        
        solution = AckermannDriveStamped()
        solution.header.stamp = rospy.get_rostime()
        solution.drive.steering_angle = solution_angle
        solution.drive.steering_angle_velocity = 0
        solution.drive.speed = solution_speed
        solution.drive.acceleration = 0
        solution.drive.jerk = 0
        #rospy.loginfo(solution_angle)
        self.pub.publish(solution)

    
    def compute_drive(self, error):
        # Computes the steering angle and speed of the car
        speed = 4
        p_gain = .4
        d_gain = .3
        #rospy.loginfo((error,'distance'))
        derr_dist = error-self.history_dist
        self.history_dist = error
        P = p_gain*error
        D = -d_gain*derr_dist
        steering_angle = P+D
        if steering_angle > .34:
            steering_angle = .34
            
        elif steering_angle <-.34:
	        steering_angle = -.34
        
        return steering_angle, speed
if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()