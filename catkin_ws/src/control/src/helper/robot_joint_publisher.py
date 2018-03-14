#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
import tf

def publish_robot_joints(listener):
    robot_joints_pub=rospy.Publisher("/joint_states", JointState, queue_size = 2)

    while True:
        robot_pos, robot_joints = EGM.get_robot_pos()
        rpy = tf.transformations.euler_from_quaternion(quat)
        theta = rpy[2]
        object_pose_msg = Float32MultiArray()
        object_pose_msg.data = [trans[0], trans[1], theta]
        object_pose_pub.publish(object_pose_msg)
        print object_pose_msg

if __name__ == '__main__':
    rospy.init_node('robot_joint_publisher', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(1)
    publish_robot_joints(listener)
