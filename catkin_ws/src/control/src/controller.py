#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
sys.path.append('../../push_control/src')
sys.path.append('../../push_control')
from push_control.srv import *

object_pose = []
robot_pos = []
time = []

def callback(data):
    global object_pose
    object_pose = data.data

def callback_time(data):
    global time
    time = data.data

def callback_robot_pos(data):
    global robot_pos
    print robot_pos
    robot_pos = data.data

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    object_pos_pub = rospy.Subscriber("/object_pose", Float32MultiArray, callback)
    robot_pose_pub = rospy.Subscriber("/cart_command_states", Float32MultiArray, callback_robot_pos)
    time_pub=rospy.Subscriber('/time', Float32, callback_time)
    robot_velocity_pub=rospy.Publisher('/robot_velocity', Float32MultiArray, queue_size = 10, latch=True)
    controller_srv = rospy.ServiceProxy('/push_control/srv_policy_FOM', Policy_SRV)
    nominal_srv = rospy.ServiceProxy('/push_control/srv_get_nominal', Nominal_SRV)

    rospy.sleep(1)
    while True:
        # print counter
        # import pdb; pdb.set_trace()
        # print time
        t0 = rospy.get_time()

        xs = object_pose +robot_pos
        # out = nominal_srv(time) #np.array([0.05, 0.])
        # print 'xs', xs
        out = controller_srv(np.array(xs), time) #np.array([0.05, 0.])
        # print 'object_pose: ', object_pose
        # print 'robot_pos: ', robot_pos
        # print 'xs: ', xs
        # print 'robot vel: ', out.us
        # print rospy.get_time() - t0
        # us = np.array([0.05, 0.])
        # print out.us
        # #publish desired velocity
        robot_velocity_msg = Float32MultiArray()
        robot_velocity_msg.data = out.us
        robot_velocity_pub.publish(robot_velocity_msg)
