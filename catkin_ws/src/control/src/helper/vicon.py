#!/usr/bin/env python
import time
import socket
import rospy
import tf

def get_object_pose(listener):
    (trans,quat) = listener.lookupTransform('/map','/vicon/StainlessSteel/StainlessSteel_rect', rospy.Time(0))
    rpy = tf.transformations.euler_from_quaternion(quat, axes='sxyz')
    theta = rpy[2]
    return trans, theta
