#!/usr/bin/env python
from helper import egm_pb2, egm_helper, data_recorder, helper
from time import sleep
import rospy
import math
import numpy as np
import time, datetime
import tf
import threading
from std_msgs.msg import Float32MultiArray, Float32
import subprocess, sys, os

robot_velocity = []

def callback(data):
    global robot_velocity
    robot_velocity = data.data

if __name__=='__main__':
    rospy.init_node('egm_control')
    listener = tf.TransformListener()
    robot_velocity_pub = rospy.Subscriber("/robot_velocity", Float32MultiArray, callback)
    time_pub=rospy.Publisher('/time', Float32, queue_size = 10, latch=True)

    # gdr = data_recorder.GraspDataRecorder('data')
    # gdr.start_recording(action='pushing', action_id=str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")), tote_num=0, frame_rate_ratio=5, image_size=-1)

    name_of_bag  = str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
    topics = ["/time", "/cart_command_states", "/cart_sensed_states", "/joint_states", "/robot_velocity", "/object_pose"]
    dir_save_bagfile = os.environ['HOME'] + '/pushing_benchmark_data/'
    rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
    rospy.sleep(1)
    rospy.loginfo('[EGM CONTROL] ready')

    time = 0.0
    time_msg = Float32()
    time_msg.data = time
    time_pub.publish(time_msg)
    # print time

    #initialize controller object
    EGM = egm_helper.EGMController(listener)

    #Define initial position
    EGM.position = np.array([-0.045, 0.009, .1782])

    #initial desired position of robot in world frame
    rate = 1000 # 250hz

    #start control loop
    counter = 0

    rospy.set_param('is_exit', False)
    t_loop = 1/rate

    t_loop_start = time_ini = rospy.get_time()

    while rospy.get_param('is_exit')==False:

        #0. publish time
        helper.publish_time(rospy.get_time()-time_ini, time_pub)

        # 1. get robot pose and publish robot states (rviz)
        EGM.get_robot_pos()

        # send robot_velocity command
        t_loop = rospy.get_time() - t_loop_start
        t_loop_start = rospy.get_time()
        EGM.send_robot_vel(robot_velocity, t_loop)

        #update counter
        counter+=1
        rospy.Rate(rate).sleep()

    #close EGM socket connection and stop rosbag recording
    EGM.sock.close()
    helper.terminate_ros_node('/record')
    rospy.loginfo('[EGM CONTROL] End of program')
