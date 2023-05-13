#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import random

def simple_publisher():
        pub = rospy.Publisher('/vesc/joy', Joy)
        rospy.init_node('simple_publisher')
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            joy_msg = Joy()
            joy_msg.header.stamp = rospy.Time.now()
            joy_msg.axes = [0.0,0.0,0.0,0.0,0.0,0.0] 
            joy_msg.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]

            pub.publish(joy_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        pass


        
