import rospy
import subprocess, sys, os
import csv
import pdb
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
import os

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for term in list_output.split("\n"):
        if (term.startswith(s)):
            os.system("rosnode kill " + term)
            print "rosnode kill " + term

def publish_time(time, time_pub):
    time_msg = Float32()
    time_msg.data = time
    time_pub.publish(time_msg)

def csv2dict(filename):
    # input_file = csv.DictReader(open(filename))
    with open (filename) as f:
        data = f.read()

    reader = csv.DictReader(data.splitlines(0)[0:])
    lines = list(reader)
    # for row in reader:
    # counties = {k: v for (k,v in ((line['%time']) for line in lines)}
    name_list = lines[0].keys()
    d = {}
    for key in name_list:
        d[key] = []
        for data in range(len(lines)):
            d[key].append(float(lines[data][key]))
    return d

def plot_trajectory(date):
    #Compile rosbag to .csv and plot trajectory
    val = os.system(".././bag2csv.sh %s" %'/home/mcube/pushing_benchmark_data/'  +date)
    base_path ='/home/mcube/pushing_benchmark_data/' + date + 'csv'
    sensed_cart = csv2dict(base_path + '/' + date + '__cart_sensed_states.csv')
    command_cart = csv2dict(base_path + '/' + date + '__cart_command_states.csv')
    object_pose = csv2dict(base_path + '/' + date + '__object_pose.csv')
    time = csv2dict(base_path + '/' + date + '__time.csv')
    time_x = np.array(time['%time']) - np.array(time['%time'][0])
    time_y = np.array(time['field.data'])
    time_sensed = (np.array(sensed_cart['%time']) - sensed_cart['%time'][0])/1e9
    x_sensed = object_pose['field.data0']
    y_sensed = object_pose['field.data1']
    time_command = (np.array(command_cart['%time']) - command_cart['%time'][0])/1e9
    x_command = command_cart['field.data0']
    y_command = command_cart['field.data1']
    # plt.plot(time_sensed, x_sensed);plt.plot(time_command, x_command);plt.show()
    # plt.plot(time_x, time_y);plt.show()
    plt.plot(x_sensed, y_sensed)
    plt.axis('equal')
    plt.show()

    pdb.set_trace()
