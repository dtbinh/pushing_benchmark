#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker
import geometry_msgs
import json
import helper

object_pose = []

def callback(data):
    global object_pose
    object_pose = data.data

if __name__ == '__main__':
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker_desired_trajectory2', Marker, queue_size = 10)
    object_pose_pub = rospy.Subscriber("/object_pose", Float32MultiArray, callback)
    rate = 30
    # rospy.sleep()

    filepath = os.environ['PUSHING_BENCHMARK_BASE'] + '/Data/8Track_point_pusher_radius_0_15_vel_0_05_infinity.json'
    nominal_data = json.load(open(filepath))
    xs = np.array(nominal_data['Matrices']['xs_star'])
    end_index=3800
    erase_int = 50
    x_path = xs[0:end_index,0]
    y_path = xs[0:end_index,1]
    z_path = y_path*0. + 0.015
    # import pdb;pdb.set_trace()
    while True:
        # import pdb;pdb.set_trace()
        # print 'test'
        # print np.abs(x_path-object_pose[0]).argmin()
        # index = np.abs(x_path-object_pose[0]).argmin()
        # x_path_clean=np.delete(x_path,range(max(index-erase_int,0),min(index+erase_int,end_index)),0)
        # y_path_clean=np.delete(y_path,range(max(index-erase_int,0),min(index+erase_int,end_index)),0)
        # z_path_clean=np.delete(z_path,range(max(index-erase_int,0),min(index+erase_int,end_index)),0)
        line_strip = helper.trajectory_viz(x_path, y_path, z_path, color = (0.,0.,0., 1.))
        marker_pub.publish(line_strip)
        rospy.Rate(rate).sleep()
