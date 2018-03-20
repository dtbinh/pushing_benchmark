#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker
import geometry_msgs
import json

if __name__ == '__main__':
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker_spline', Marker, queue_size = 10)
    rate = 30

    filepath = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/8Track_point_pusher_radius_0_15_vel_0_05.json'
    nominal_data = json.load(open(filepath))
    xs = nominal_data['xs']
    x_path = xs[0]
    y_path = xs[1]
    z_path = [0.02]*len(y_path)
    while True:
        points = Marker()
        line_strip = Marker()
        line_list = Marker()
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/track_start"
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rospy.Time.now()
        points.action = line_strip.action = line_list.action = Marker.ADD
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0

        points.type = Marker.POINTS
        line_strip.type = Marker.LINE_STRIP

        points.scale.x = 0.2
        points.scale.y = 0.2

        line_strip.scale.x = 0.005

        points.color.g = 1.0
        points.color.a = 1.0

        # // Line strip is blue
        line_strip.color.b = 1.0
        line_strip.color.a = 1.0

        f = 0.
        # import pdb;pdb.set_trace()

        #// Create the vertices for the points and lines
        for i in range(0, len(z_path), 50):
            x= x_path[i]
            y= y_path[i]
            z= z_path[i]

            p =geometry_msgs.msg.Point()
            p.x = x
            p.y = y
            p.z = z
          #
            points.points.append(p)
            line_strip.points.append(p)
          #
          # // The line list needs two points for each line
            line_list.points.append(p)
        # }


        # marker_pub.publish(points)
        marker_pub.publish(line_strip)
        #
        rospy.Rate(rate).sleep()
        #
        f += 0.04
