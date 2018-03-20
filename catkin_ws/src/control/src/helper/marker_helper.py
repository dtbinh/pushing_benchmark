#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker
import geometry_msgs
if __name__ == '__main__':
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker_spline', Marker, queue_size = 10)
    rate = 30

    while True:
        points = Marker()
        line_strip = Marker()
        line_list = Marker()
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map"
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rospy.Time.now()
        points.action = line_strip.action = line_list.action = Marker.ADD
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0
        print 'hi'


        points.type = Marker.POINTS
        line_strip.type = Marker.LINE_STRIP
        line_list.type = Marker.LINE_LIST



        points.scale.x = 0.2
        points.scale.y = 0.2

        line_strip.scale.x = 0.1
        line_list.scale.x = 0.1

        points.color.g = 1.0
        points.color.a = 1.0

        # // Line strip is blue
        line_strip.color.b = 1.0
        line_strip.color.a = 1.0

        # // Line list is red
        line_list.color.r = 1.0
        line_list.color.a = 1.0

        f = 0.

            # // Create the vertices for the points and lines
        for i in range(100):
            y = 5 * np.sin(f + i / 100.0 * 2 * np.pi)
            z = 5 * np.cos(f + i / 100.0 * 2 * np.pi)

            p =geometry_msgs.msg.Point()
            p.x = i - 50
            p.y = y
            p.z = z
          #
            points.points.append(p)
            line_strip.points.append(p)
          #
          # // The line list needs two points for each line
            line_list.points.append(p)
            p.z += 1.0;
            line_list.points.append(p)
        # }


            marker_pub.publish(points)
            marker_pub.publish(line_strip)
            marker_pub.publish(line_list)
            #
            rospy.Rate(rate).sleep()
            #
            f += 0.04
