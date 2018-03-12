#!/usr/bin/env python
from helper import egm_pb2, egm_helper, vicon
from time import sleep
import rospy
import math

import numpy as np
import time
import tf
import threading
from std_msgs.msg import Float32MultiArray

lock = threading.Lock()

if __name__=='__main__':
    rospy.init_node('egm_control')
    listener = tf.TransformListener()
    rospy.sleep(1)

    #initialize controller object
    EGM = egm_helper.EGMController()

    #initial desired position of robot in world frame
    position = np.array([.300, 0, .1782])
    rate = 250 # 250hz

    #start control loop
    while True:
        #enforce desired controller frequency
        rospy.Rate(rate).sleep()

        # 1. get robot pose (robot_pos = [x,y,z], robot_joints = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'])
        robot_pos, robot_joints = EGM.get_robot_pos()

        # 2. get object pose and publish
        vicon.get_object_pose(listener)

        #3. apply control policy
        vel_robot = rospy.wait_for_message("/robot_velocity", Float32MultiArray, 1)

        #4. send desired robot velocity
        position = EGM.send_robot_vel(position, vel_robot, rate)
        print position

    sock.close()
    print 'End of program'
