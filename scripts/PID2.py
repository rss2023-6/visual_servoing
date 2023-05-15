#!/usr/bin/env python2

import numpy as np
import math
import rospy
from rospy.numpy_msg import numpy_msg
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Float32MultiArray

class lane_PID:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation"
    ERROR_TOPIC = "/lane_error"

    def __init__(self):
        # Initialize your publishers and
        # subscribers here

        #might need to make these values all negative if I fucked up on signs / left right
        #self.p_gain = -0.15 2
        self.p_gain = -0.1
        self.max_angle = 0.34
        self.d_gain = -0.0 #-0.015
        #self.p_gain_angle = -0.35 2
        self.p_gain_angle = -0.3

        # if(self.lane == 'middle'):
        #     self.p_gain_angle = -0.3
        #     self.p_gain = -0.1

        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)  
        # self.error_sub = rospy.Subscriber(self.ERROR_TOPIC, Float32, self.error_cb) 
        self.pos_sub = rospy.Subscriber("/lane_position", Float32MultiArray, self.lane_pos_callback)
        self.last_time = rospy.Time.now().to_sec()
        self.left = None
        self.right = None
        self.speed = 4.0
        self.error_total = 0
        self.history_dist = 0
        self.prev_error = None
        rospy.logerr("PID starting")
    
    def lane_pos_callback(self, msg):
        self.left = -1.0 * msg.data[1]
        self.right = -1.0 * msg.data[0]
        self.angle = -1.0 * msg.data[2]

        solution_angle, solution_speed = self.compute_drive_w_angle(self.left, self.right, self.angle)

        solution_drive = AckermannDriveStamped()
        
        solution_drive.header.stamp = rospy.get_rostime()
        solution_drive.drive.steering_angle = solution_angle
        solution_drive.drive.steering_angle_velocity = 0
        solution_drive.drive.speed = 2.0
        solution_drive.drive.acceleration = 0
        solution_drive.drive.jerk = 0
        
        # rospy.logerr("publishing solution!")
        self.drive_pub.publish(solution_drive)


    def compute_drive_w_angle(self, left, right, angle):
        # print("CAR ANGLE (degrees)", angle / np.pi * 180)

        dT = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()

        error = left + right
        angle_error = angle

        derr_dist = (error - self.history_dist) / (dT * 1.0)

        # print(derr_dist)
        self.history_dist = error
        self.error_total += error

        P = self.p_gain*error
        D = self.d_gain*derr_dist
        Pangle = self.p_gain_angle*angle_error

        total_error = P+D+Pangle
        solution_angle = np.clip(P + D + Pangle, -1.0 * self.max_angle, 1.0 * self.max_angle)

        # if(self.prev_error == None):
        #     self.prev_error = total_error

        # if(self.prev_error < -0.12 and total_error > 0):
        #     solution_angle = -0.20
        
        self.prev_error = total_error

        # if(P + D + Pangle > 0):
        #     solution_angle = np.clip((P + D + Pangle)*3.0, -1.0 * self.max_angle, 1.0 * self.max_angle)

        solution_speed = 2.0 #*(1 - abs(total_error) * 2.0)
        
        # print('steerig (degrees)', solution_angle / np.pi * 180)
        return solution_angle, solution_speed
         
    
    # def compute_drive2(self, left, right):
    #     dT = (rospy.Time.now() - self.last_time).to_sec()
    #     error = left + right
    #     derr_dist = (error - self.history_dist) / (dT * 1.0)
    #     self.history_dist = error
    #     self.error_total += error

    #     P = -1.0 * self.p_gain*error
    #     D = -1.0 * self.d_gain*derr_dist

    #     solution_angle = np.clip(P + D, -1.0 * self.max_angle, 1.0 * self.max_angle)
    #     solution_speed = self.speed * np.clip(1.2 - 0.5 * abs(error), 1.0)
    #     #error shoudl range from roughly 0 to 0.5

    #     return solution_angle, solution_speed
    
    # def error_cb(self,msg):
    #     pass
    #     solution_angle, solution_speed = self.compute_drive(msg.data)
        
    #     solution = AckermannDriveStamped()
    #     solution.header.stamp = rospy.get_rostime()
    #     solution.drive.steering_angle = solution_angle
    #     solution.drive.steering_angle_velocity = 0
    #     solution.drive.speed = solution_speed
    #     solution.drive.acceleration = 0
    #     solution.drive.jerk = 0
    #     rospy.logerr(solution_angle)
    #     self.drive_pub.publish(solution)
    # def compute_drive(self, error):
    #     # Computes the steering angle and speed of the car
    #     pass
    #     speed = 4
    #     p_gain = .4
    #     d_gain = .1
    #     #rospy.loginfo((error,'distance'))
    #     derr_dist = error-self.history_dist
    #     self.history_dist = error

    #     P = p_gain*error
    #     max_angle = 0.17
    #     D = -d_gain*derr_dist
    #     steering_angle = P+D
    #     if steering_angle > max_angle:
    #         steering_angle = max_angle
            
    #     elif steering_angle <-1.0*max_angle:
	#         steering_angle = -1.0*max_angle
        
    #     return steering_angle, speed
    
if __name__ == "__main__":
    rospy.init_node('wall_follower')
    lane_PID = lane_PID()
    rospy.spin()
