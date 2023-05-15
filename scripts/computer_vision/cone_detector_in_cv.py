#!/usr/bin/env python
from __future__ import division
import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import transform_image, get_lane_position

class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.pos_pub = rospy.Publisher("/lane_position", Float32MultiArray, queue_size=30)
        self.err_pub = rospy.Publisher("/lane_error", Float32, queue_size=30)

        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=30)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.prev_left = None
        self.prev_right = None
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################
        rospy.logerr("image_msg!!")
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        strong_lines, x_intercept, strong_lines2 = transform_image(image)

        if(type(strong_lines) != np.ndarray):
            rospy.logerr("got no lines!! :(")
            return
        
        avg_angle, left, right = get_lane_position(strong_lines, x_intercept)

        if(self.prev_left == None):
            self.prev_left = left

        if(self.prev_right == None):
            self.prev_right == right

        if(abs(self.prev_left - left) > 0.5):
            self.left = self.prev_left + 0.83 #set out of lane
        if(abs(self.prev_right - right) > 0.5):
            self.right = self.right - 0.83 #set out of lane

        #debug msg
        #image1 = cv2.line(image, intersection_pt, end_intersection_pt, (0,200,255), 3, cv2.LINE_AA) #radius = 
        #img2 = cv2.circle(image, (end_intersection_pt[0],159),5,(255,0,0),-1)
        #debug_msg = self.bridge.cv2_to_imgmsg(image1, "bgr8")
        #self.debug_pub.publish(debug_msg)
        #rospy.logerr(Left)
        #rospy.logerr("Right{}".format(Right))

        # self.err_pub.publish(Left[1] + Right[1])

        float_array_msg = Float32MultiArray()
        float_array_msg.data = [left, right, avg_angle]
        self.pos_pub.publish(float_array_msg)
        self.prev_left = left
        self.prev_right = right


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass