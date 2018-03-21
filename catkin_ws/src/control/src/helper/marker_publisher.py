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

    filepath = os.environ['PUSHING_BENCHMARK_BASE'] + '/Data/8Track_point_pusher_radius_0_15_vel_0_05_infinity.json'
    nominal_data = json.load(open(filepath))
    xs = np.array(nominal_data['Matrices']['xs_star'])
    x_path = xs[:,0]
    y_path = xs[:,1]
    z_path = y_path*0.
    # import pdb;pdb.set_trace()
    while True:
        points = Marker()
        line_strip = Marker()
        line_list = Marker()
        line_strip.header.frame_id  = "/track_start"
        line_strip.header.stamp = rospy.Time.now()
        line_strip.action =  Marker.ADD
        line_strip.pose.orientation.w =  1.0

        line_strip.type = Marker.LINE_STRIP

        line_strip.scale.x = 0.005

        # // Line strip is blue
        line_strip.color.r = 0.0
        line_strip.color.g = 0.0
        line_strip.color.b = 0.0
        line_strip.color.a = 1.0

        f = 0.
        # import pdb;pdb.set_trace()

        #// Create the vertices for the points and lines
        for i in range(0, 3800, 50):
            x= x_path[i]
            y= y_path[i]
            z= z_path[i]
            # print x,y,z

            p =geometry_msgs.msg.Point()
            p.x = x
            p.y = y
            p.z = z + 0.015
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
