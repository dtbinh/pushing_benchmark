#!/usr/bin/env python


import sys
import os, time
import optparse
import json
import numpy as np
import json
import rospy
sys.path.append(os.environ['PUSHING_BENCHMARK_BASE'] + '/Data')
sys.path.append('../helper')
import scipy
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import copy
import pdb
import tf
import rospy
import tf.transformations as tfm


if __name__=='__main__':
    camera_matrix = np.array([[1391.26, 0.0, 939.721], [0.0, 1391.26, 531.336], [0.0, 0.0, 1.0]])
    #extrinsics = np.array([0.0938854516,0.0467152707,0.0080374023,0.0007305508,-0.00060057,-0.7132804394,0.7008781433])
    extrinsics = np.array([9.32469439e-01, 7.41843937e-03, 5.58807896e-01, 6.41777363e-01, 6.34991100e-01, -3.04887434e-01, -3.03235504e-01])
    # Matrix for extrinsics
    translate = extrinsics[0:3]
    quaternion = extrinsics[3:7]
    extrinsics_matrix = np.dot(tfm.compose_matrix(translate=translate),tfm.quaternion_matrix(quaternion))[0:3]
    
    # Transformation matrix
    transformation_matrix = np.dot(camera_matrix,extrinsics_matrix)
    
    
    #Load desired trajectory
    desired_traj_JSON = os.environ['PUSHING_BENCHMARK_BASE'] + '/Data/'+'8Track_point_pusher_radius_0.15_vel_0.08_3_laps.json'
    
    with open(desired_traj_JSON) as json_data:
        des_var = json.load(json_data)
    
    des_var = des_var['Matrices']
    print des_var.keys()
    
    #Load actual trajectory
    acutal_traj_JSON = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/'+'8Track_point_pusher_radius_0.11_vel_0.08_3_lapsexperiments.json'
    with open(acutal_traj_JSON) as json_data:
        act_var = json.load(json_data)
    print act_var.keys()
    x_off = 0.35
    y_off = 0
    z_off = 0

    # t,x,y,z of desired trajectory
    t_des = np.array(des_var['t_star'])[:,0]
    x_des = np.array(des_var['xc_star'])[:,0]+x_off
    y_des = np.array(des_var['xc_star'])[:,1]+y_off
    z_des = np.array(des_var['xc_star'])[:,2]*0+z_off
    
    # t,x,y,z of desired trajectory
    t_act = np.array(act_var['timeJSON'])
    x_act = np.array(act_var['xc'][0])+x_off
    y_act = np.array(act_var['xc'][1])+y_off
    z_act = np.array(act_var['xc'][2])*0+z_off
    
    vector_des = np.array([x_des, y_des, z_des, z_des*0+1])
    pixels_des = np.dot(transformation_matrix, vector_des)
    vector_act = np.array([x_act, y_act, z_act, z_act*0+1])
    pixels_act = np.dot(transformation_matrix, vector_act)
    pixels_center = np.dot(transformation_matrix, np.array([0.35,0,0,1]))
    plt.plot(pixels_center[0]/pixels_center[2], pixels_center[1]/pixels_center[2], 'o')
    pixels_center = np.dot(transformation_matrix, np.array([9.32469439e-01, 7.41843937e-03, 5.58807896e-01, 1]))
    plt.plot(pixels_center[1]/pixels_center[2], pixels_center[0]/pixels_center[2], 'o')
    #1920 1080
    
    
    #plt.axes().set_aspect('equal')
    #plt.show()
    
    img=mpimg.imread('/home/mcube/pushing_benchmark_data/Screenshot from 2018-05-03 17-59-13.png')
    
    plt.imshow(img)
    plt.plot(np.multiply(pixels_des[0], pixels_des[2]), np.multiply(pixels_des[1], pixels_des[2]))
    plt.plot(np.divide(pixels_act[0], pixels_act[2]), np.divide(pixels_act[1], pixels_act[2]))
    
    plt.show()
    pdb.set_trace()
    
    
    
