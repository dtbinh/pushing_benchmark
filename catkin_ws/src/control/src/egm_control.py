#!/usr/bin/env python
from helper import egm_pb2
from helper import egm_helper
from time import sleep
import rospy
import math
import numpy as np
import time
import tf

if __name__=='__main__':
    rospy.init_node('egm_control')
    listener = tf.TransformListener()

    rospy.sleep(1)
    EGM = egm_helper.EGMController()

    position = np.array([300, 0, 178.2])
    rate = 250 # 250hz

    while True:
        rospy.Rate(rate).sleep()

        # 0. Update desired robot position
        position = position + 1./250*np.array([50.,0.,0.])

        # 1. get robot pose
        robot_pos, robot_joint = EGM.get_robot_pos()

        # 2. get object pose
        object_pose = vicon.get_object_pose()

        # 2. publish robot joints (visualize rviz)
        EGM.publish_robot_joints(robot_joints)

        #3. send desired position
        # EGM.send_robot_pos(position)
        EGM.send_robot_vel(position, np.array([0.05,0,0]), rate)

    sock.close()
    print 'End of program'
