#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker
import geometry_msgs
import json, pdb

object_pose = []

def callback(data):
    global object_pose
    object_pose = data.data

def trajectory_viz(frame_id = "/track_start", line_thick = 0.005, color = (0.,0.,1., 1.)):
    line_strip = Marker()
    line_strip.header.frame_id  = frame_id
    line_strip.header.stamp = rospy.Time.now()
    line_strip.action =  Marker.ADD
    line_strip.pose.orientation.w =  1.0
    line_strip.type = Marker.LINE_STRIP
    line_strip.scale.x = line_thick
    # // Line strip is blue
    line_strip.color.r = color[0]
    line_strip.color.g = color[1]
    line_strip.color.b = color[2]
    line_strip.color.a = color[3]
    return line_strip

if __name__ == '__main__':
    rospy.init_node('marker_publisher', anonymous=True)
    object_pos_pub = rospy.Subscriber("/object_pose", Float32MultiArray, callback)
    traj_pub = rospy.Publisher('/visualization_marker_traj', Marker, queue_size = 10)
    rate = 30
    rospy.sleep(1)

    while True:

        line_strip = trajectory_viz(color = (0.,0.,1.,1.))


        p =geometry_msgs.msg.Point()
        # pdb.set_trace()
        for i in range(0, 3800, 50):
            print object_pose
            x= object_pose[0]
            y= object_pose[1]

            p.x = x
            p.y = y
            p.z = 0 + 0.015

            line_strip.points.append(p)


        traj_pub.publish(line_strip)
        rospy.Rate(rate).sleep()
