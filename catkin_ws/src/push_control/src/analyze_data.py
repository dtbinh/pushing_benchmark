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
import copy
import pdb



if __name__=='__main__':
    
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

    # t,x,y of desired trajectory
    t_des = np.array(des_var['t_star'])[:,0]
    x_des = np.array(des_var['xc_star'])[:,0]
    y_des = np.array(des_var['xc_star'])[:,1]
    plt.plot(x_des,y_des,'r'); 
    
    # t,x,y of desired trajectory
    t_act = np.array(act_var['timeJSON'])
    x_act = np.array(act_var['xc'][0])
    y_act = np.array(act_var['xc'][1])
    plt.plot(x_act,y_act, 'b'); 
    
    # timesteps for evaluation
    num_steps = 100

    t_final = min(t_des[-1], t_act[-1])
    time_steps = np.linspace(t_des[0], t_final, num=num_steps)
    print time_steps.shape
    
    # Interpolate for both trajectories:
    x_des_interp = np.interp(time_steps, t_des, x_des)
    y_des_interp = np.interp(time_steps, t_des, y_des)
    x_act_interp = np.interp(time_steps, t_act, x_act)
    y_act_interp = np.interp(time_steps, t_act, y_act)
    
    plt.plot(x_des_interp,y_des_interp, 'ro'); 
    plt.plot(x_act_interp,y_act_interp, 'bo'); 
    plt.axes().set_aspect('equal')
    plt.show()
    
    error = np.sum(np.sqrt((x_des_interp - x_act_interp)**2+(y_des_interp - y_act_interp)**2))
    
    print 'Total error:', error
    print ' Average errror:', error/num_steps
    pdb.set_trace()
    # plt.plot(time_steps, y_des_interp); plt.plot(time_steps, y_act_interp); plt.show()
