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
    
    list_des_files = ['8Track_point_pusher_radius_0.11_vel_0.08_3_laps.json',
    '8Track_point_pusher_radius_0.12_vel_0.08_3_laps.json',
    '8Track_point_pusher_radius_0.13_vel_0.08_3_laps.json',
    '8Track_point_pusher_radius_0.14_vel_0.08_3_laps.json',
    '8Track_point_pusher_radius_0.15_vel_0.08_3_laps.json',
    '8Track_point_pusher_radius_0.16_vel_0.08_3_laps.json',
    '8Track_point_pusher_radius_0.17_vel_0.08_3_laps.json',
    ]
   
    list_act_files = ['8Track_point_pusher_radius_0.11_vel_0.08_3_lapsexperiments.json',
    '8Track_point_pusher_radius_0.12_vel_0.08_3_lapsexperiments.json',
    '8Track_point_pusher_radius_0.13_vel_0.08_3_lapsexperiments.json',
    '8Track_point_pusher_radius_0.14_vel_0.08_3_lapsexperiments.json',
    '8Track_point_pusher_radius_0.15_vel_0.08_3_lapsexperiments.json',
    '8Track_point_pusher_radius_0.16_vel_0.08_3_lapsexperiments.json',
    '8Track_point_pusher_radius_0.17_vel_0.08_3_lapsexperiments.json',
    ] 
        
    total_error_vec = []
    avg_error_vec = []
    for counter in range(0, len(list_des_files)):

        
        #Load desired trajectory
        desired_traj_JSON = os.environ['PUSHING_BENCHMARK_BASE'] + '/Data/'+ list_des_files[counter]
        
        with open(desired_traj_JSON) as json_data:
            des_var = json.load(json_data)
        
        des_var = des_var['Matrices']
        print des_var.keys()
        
        #Load actual trajectory
        acutal_traj_JSON = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/'+list_act_files[counter]
        print acutal_traj_JSON
        with open(acutal_traj_JSON) as json_data:
            act_var = json.load(json_data)
        print act_var.keys()

        # t,x,y of desired trajectory
        t_des = np.array(des_var['t_star'])[:,0]
        x_des = np.array(des_var['xc_star'])[:,0]
        y_des = np.array(des_var['xc_star'])[:,1]
        plt.plot(x_des,y_des,'r'); 
        
        # t,x,y of actual trajectory
        t_act = np.array(act_var['timeJSON'])
        x_act = np.array(act_var['xc'][0])
        y_act = np.array(act_var['xc'][1])
        plt.plot(x_act,y_act, 'b'); 
        # import pdb;pdb.set_trace()
        
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
        # plt.show()
        plt.savefig(list_act_files[counter][0:-4] + 'png')
        plt.close()
        
        error = np.sum(np.sqrt((x_des_interp - x_act_interp)**2+(y_des_interp - y_act_interp)**2))
        
        total_error_vec.append(error)
        avg_error_vec.append(error/num_steps)
        print 'Total error:', error
        print ' Average errror:', error/num_steps
    # pdb.set_trace()
    
    
    #post-processing plotting
    # vel_vec = [.02,.03,.04,.05,.06,.07,.08,.09]
    # nu_vec = [0,.15,.3,.45,.6,.75,.9,1.]
    # horizon_vec = [5,10,20,30,40,50]
    radius_vec = [11,12,13,14,15,16,17]
    plt.close()
    # plt.plot(vel_vec, total_error_vec);
    # total_error_vec[0] = 100
    # total_error_vec[1] = 100
    plt.plot(radius_vec, total_error_vec);
    # plt.xlabel('Velocity (m/s)')
    # plt.xlabel('Pusher Friction Coefficient')
    # plt.xlabel('Horizon Steps')
    plt.xlabel('Radius')
    plt.ylabel('Error')
    # plt.show()
    # plt.savefig('Velocity.png')
    # plt.savefig('Friction.png')
    # plt.savefig('Horizon.png')
    plt.savefig('Radius.png')
        # plt.plot(time_steps, y_des_interp); plt.plot(time_steps, y_act_interp); plt.show()
