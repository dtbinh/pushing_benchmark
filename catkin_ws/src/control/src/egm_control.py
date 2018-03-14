#!/usr/bin/env python
from helper import egm_pb2, egm_helper, vicon
from time import sleep
import rospy
import math

import numpy as np
import time
import tf
import threading
from std_msgs.msg import Float32MultiArray, Float32

robot_velocity = []

def callback(data):
    global robot_velocity
    robot_velocity = data.data

if __name__=='__main__':
    rospy.init_node('egm_control')
    listener = tf.TransformListener()
    robot_velocity_pub = rospy.Subscriber("/robot_velocity", Float32MultiArray, callback)
    time_pub=rospy.Publisher('/time', Float32, queue_size = 10, latch=True)

    rospy.sleep(1)

    #initialize controller object
    EGM = egm_helper.EGMController()

    #Define initial position
    EGM.position = np.array([.300, 0, .1782])

    #initial desired position of robot in world frame
    rate = 250 # 250hz

    #start control loop
    counter = 0
    while True:
        if (counter==0):
            time_ini = rospy.get_time()
        time = rospy.get_time()-time_ini
        time_msg = Float32()
        time_msg.data = time
        time_pub.publish(time_msg)
        print time
        #enforce desired controller frequency
        rospy.Rate(rate).sleep()

        # 1. get robot pose
        EGM.get_robot_pos()

        # send robot_velocity command
        EGM.send_robot_vel(robot_velocity, rate)
        #update counter
        counter+=1

    sock.close()
    print 'End of program'
