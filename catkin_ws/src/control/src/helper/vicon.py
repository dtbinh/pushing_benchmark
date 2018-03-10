#!/usr/bin/env python
import time
import socket
import rospy
import tf
from std_msgs.msg import Float32MultiArray

object_pose_pub=rospy.Publisher('/object_pose', Float32MultiArray, queue_size = 10, latch=True)

def get_object_pose(listener):
    (trans,quat) = listener.lookupTransform('/map','/vicon/StainlessSteel/StainlessSteel_rect', rospy.Time(0))
    rpy = tf.transformations.euler_from_quaternion(quat, axes='sxyz')
    theta = rpy[2]
    #publish object pose
    object_pose_msg = Float32MultiArray()
    object_pose_msg.data = [trans, theta]

    return trans, theta
