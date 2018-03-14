#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
sys.path.append('../../push_control/src')
sys.path.append('../../push_control')
from push_control.srv import *

object_pose = []
time = []

def callback(data):
    global object_pose
    object_pose = data.data

def callback_time(data):
    global time
    time = data.data

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    object_pose_pub = rospy.Subscriber("/object_pose", Float32MultiArray, callback)
    time_pub=rospy.Subscriber('/time', Float32, callback_time)
    robot_velocity_pub=rospy.Publisher('/robot_velocity', Float32MultiArray, queue_size = 10, latch=True)
    controller_srv = rospy.ServiceProxy('/push_control/srv_policy_FOM', Policy_SRV)

    rospy.sleep(1)
    while True:
        # print counter
        us = controller_srv(np.array([0.,0.,0.,0.,0.,0.]), time) #np.array([0.05, 0.])
        print us
        #publish desired velocity
        robot_velocity_msg = Float32MultiArray()
        robot_velocity_msg.data = us
        robot_velocity_pub.publish(robot_velocity_msg)
