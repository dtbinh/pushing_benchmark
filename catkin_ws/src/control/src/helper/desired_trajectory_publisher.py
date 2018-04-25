#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker
import geometry_msgs
import json
import helper

if __name__ == '__main__':
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker_desired_trajectory2', Marker, queue_size = 10)
    rate = 30

    filepath = os.environ['PUSHING_BENCHMARK_BASE'] + '/Data/8Track_point_pusher_radius_0_15_vel_0_05_infinity.json'
    nominal_data = json.load(open(filepath))
    xs = np.array(nominal_data['Matrices']['xs_star'])
    x_path = xs[0:3800,0]
    y_path = xs[0:3800,1]
    z_path = y_path*0. + 0.015
    # import pdb;pdb.set_trace()
    while True:
        line_strip = helper.trajectory_viz(x_path, y_path, z_path,color = (0.,0.,0., 1.))
        marker_pub.publish(line_strip)
        rospy.Rate(rate).sleep()
