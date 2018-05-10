#!/usr/bin/env python


import sys
import os, time
import optparse
import json
import numpy as np
import json
import rospy
sys.path.append('/home/mcube/pushing_benchmark' + '/Data')
sys.path.append('../helper')
import scipy
import matplotlib.pyplot as plt
import copy
import pdb



if __name__=='__main__':
    
    parser = optparse.OptionParser()
    parser.add_option('-f', '--file', action="store", dest='file_JSON', 
                      help='JSON file with information', default='')
    (opt, args) = parser.parse_args()
    file_JSON = opt.file_JSON
    
   
    #Load actual trajectory
    #acutal_traj_JSON = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/'+list_act_files[counter]
    if file_JSON == '':
        file_JSON = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/8Track_point_pusher_radius_0_15_vel_0_05_3_laps_gp_controllertest.json'
    print file_JSON
    with open(file_JSON) as json_data:
        data = json.load(json_data)
    print data.keys()

    # t,x,y of desired trajectory
    t_des = np.array(data['timeJSON'])
    x_des = np.array(data['xc_desired'][0])
    y_des = np.array(data['xc_desired'][1])
    plt.plot(x_des,y_des,'r'); 
    
    # t,x,y of actual trajectory
    t_act = np.array(data['timeJSON'])
    x_act = np.array(data['xc'][0])
    y_act = np.array(data['xc'][1])
    plt.plot(x_act,y_act, 'b'); 
    # import pdb;pdb.set_trace()
    
    # Relevant parameters:
    Q = np.array(data['Q'])
    Qf = np.array(data['Qf'])
    R = np.array(data['R'])
    h_mpc = np.array(data['h_mpc'])
    steps_mpc = np.array(data['steps_mpc'])
    
    print 'Parameters.'
    print 'Q:', Q
    print 'Qf: ', Qf
    print 'R: ', R 
    print 'h_mpc: ', h_mpc
    print 'steps_mpc: ', steps_mpc
    
    
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
    #plt.savefig(list_act_files[counter][0:-4] + 'png')
    plt.close()
    
    error = np.sum(np.sqrt((x_des_interp - x_act_interp)**2+(y_des_interp - y_act_interp)**2))
    
    print 'Total error:', error
    print ' Average errror:', error/num_steps

