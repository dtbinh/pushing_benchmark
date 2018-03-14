#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import JointState
sys.path.append('../../push_control/src')
sys.path.append('../../push_control')
from push_control.srv import *
import tf
from helper import vicon

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    listener = tf.TransformListener()

    robot_velocity_pub=rospy.Publisher('/robot_velocity', Float32MultiArray, queue_size = 10, latch=True)
    time_pub=rospy.Publisher('/time', Float32, queue_size = 10, latch=True)
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size = 2)
    cart_sensed_pub = rospy.Publisher("/cart_sensed_states", Float32MultiArray, queue_size = 2)
    cart_command_pub = rospy.Publisher("/cart_command_states", Float32MultiArray, queue_size = 2)
    controller_srv = rospy.ServiceProxy('/push_control/srv_get_nominal', Nominal_SRV)

    #publish time
    time_msg = Float32()
    time_msg.data = 0;
    time_pub.publish(time_msg)
    #get nominal state
    out = controller_srv(0)
    xs = np.array(out.xs)
    object_pose = xs[0:3]
    pusher_pos = xs[3:5]
    us = np.array(out.us)
    #publish robot cart
    robot_cart_msg = Float32MultiArray()
    # cart_command_pub.data = pusher_pos
    # cart_command_pub.publish(cart_command_pub)
    # cart_sensed_pub.publish(cart_sensed_pub)
    # #publish robot velocity
    # vel_msg = Float32MultiArray()
    # vel_msg.data = us
    # robot_velocity_pub(vel_msg)
    # #start tracking the object_poserospy.sleep(1)
    # vicon.publish_object_pose(listener)
