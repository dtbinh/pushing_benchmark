#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

object_pose = []

def callback(data):
    global object_pose
    object_pose = data.data

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    object_pose_pub = rospy.Subscriber("/object_pose", Float32MultiArray, callback)
    robot_velocity_pub=rospy.Publisher('/robot_velocity', Float32MultiArray, queue_size = 10, latch=True)

    counter = 0
    rate = 250
    while True:
        print counter
        rospy.Rate(rate).sleep()
        counter+=1
        us = np.array([counter, 0.])
        #publish desired velocity
        robot_velocity_msg = Float32MultiArray()
        robot_velocity_msg.data = us
        robot_velocity_pub.publish(robot_velocity_msg)
