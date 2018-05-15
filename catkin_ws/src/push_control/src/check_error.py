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
        file_JSON = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/8Track_point_pusher_radius_0.15_vel_0.05_3_lapstest_fm.json'
    print file_JSON
    with open(file_JSON) as json_data:
        data = json.load(json_data)
    print data.keys()

    # t,x,y of desired trajectory
    t_des = np.array(data['timeJSON'])
    x_des = np.array(data['xc_desired'][0])
    y_des = np.array(data['xc_desired'][1])
    ori_des = np.array(data['xc_desired'][2])
    r_des = np.array(data['xc_desired'][3])
    vx_des = np.array(data['us_desired'][0])
    vy_des = np.array(data['us_desired'][1])
    q_des = np.array(data['q_pusher_commanded'])
    
    
    # t,x,y of actual trajectory
    t_act = np.array(data['timeJSON'])
    x_act = np.array(data['xc'][0])
    y_act = np.array(data['xc'][1])
    ori_act = np.array(data['xc'][2])
    r_act = np.array(data['xc'][3])
    vx_act = np.array(data['us'][0])
    vy_act = np.array(data['us'][1])
    q_act = np.array(data['q_pusher_sensed'])
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
    vx_des_interp = np.interp(time_steps, t_des, vx_des)
    vy_des_interp = np.interp(time_steps, t_des, vy_des)
    vx_act_interp = np.interp(time_steps, t_act, vx_act)
    vy_act_interp = np.interp(time_steps, t_act, vy_act)
    ori_des_interp = np.interp(time_steps, t_des, ori_des)
    r_des_interp = np.interp(time_steps, t_des, r_des)
    ori_act_interp = np.interp(time_steps, t_act, ori_act)
    r_act_interp = np.interp(time_steps, t_act, r_act)
    
    plt.figure(figsize=(20,20))
    ax1 = plt.subplot2grid((4,3), (0, 0), colspan=1, rowspan=4)
    ax1.plot(x_des,y_des,'r'); 
    ax1.plot(x_act,y_act, 'b'); 
    ax1.plot(x_des_interp,y_des_interp, 'ro'); 
    ax1.plot(x_act_interp,y_act_interp, 'bo'); 
    ax1.set_aspect('equal')
    #plt.show()
    #plt.savefig(list_act_files[counter][0:-4] + 'png')
    #plt.close()
    
    error = np.sum(np.sqrt((x_des_interp - x_act_interp)**2+(y_des_interp - y_act_interp)**2))
    
    print 'Total error:', error
    print ' Average errror:', error/num_steps
    ax1.set_title(' Average errror in trajectory: {}'.format(error/num_steps))

    ax2 = plt.subplot2grid((4,3), (0, 1), colspan=1, rowspan = 2)
    ax2.plot(vx_des_interp,vy_des_interp, 'ro'); 
    ax2.plot(vx_act_interp,vy_act_interp, 'bo'); 
    ax2.plot(vx_des,vy_des, 'r.'); 
    ax2.plot(vx_act,vy_act, 'b.'); 
    ax2.set_aspect('equal')
    #plt.show()
    #plt.savefig(list_act_files[counter][0:-4] + 'png')
    #plt.close()
    
    error = np.sum(np.sqrt((vx_des_interp - vx_act_interp)**2+(vy_des_interp - vy_act_interp)**2))
    
    print 'Total error in commands:', error
    print 'Average errror in commands:', error/num_steps
    ax2.set_title(' Average errror in commands: {}'.format(error/num_steps))
    
    ax3 = plt.subplot2grid((4, 3), (2, 2), rowspan=1, colspan=1)
    ax3.plot(time_steps, ori_des_interp, 'ro'); 
    ax3.plot(time_steps, ori_act_interp, 'bo'); 
    ax3.plot(t_des, ori_des, 'r'); 
    ax3.plot(t_act, ori_act, 'b'); 
    
    error = np.sum(np.sqrt((ori_des_interp - ori_act_interp)**2))
    
    print 'Total error in orientation:', error
    print ' Average errror in orientation:', error/num_steps
    ax3.set_title(' Average errror in orientation: {}'.format(error/num_steps))
    
    
    ax4 = plt.subplot2grid((4, 3), (3, 2), rowspan=1, colspan=1)
    ax4.plot(time_steps, r_des_interp, 'ro'); 
    ax4.plot(time_steps, r_act_interp, 'bo'); 
    ax4.plot(t_des, r_des, 'r'); 
    ax4.plot(t_act, r_act, 'b'); 
    
    error = np.sum(np.sqrt((r_des_interp - r_act_interp)**2))
    
    print 'Total error in contact point:', error
    print ' Average errror in contact_point:', error/num_steps
    ax4.set_title(' Average errror in contact_point: {}'.format(error/num_steps))
    
    ax5 = plt.subplot2grid((4, 3), (0, 2), rowspan=1, colspan=1)
    ax5.plot(time_steps, x_des_interp, 'ro'); 
    ax5.plot(time_steps, x_act_interp, 'bo'); 
    ax5.plot(t_des, x_des, 'r'); 
    ax5.plot(t_act, x_act, 'b'); 
    
    error = np.sum(np.sqrt((x_des_interp - x_act_interp)**2))
    
    print 'Total error in x:', error
    print ' Average errror in x:', error/num_steps
    ax5.set_title(' Average errror in x: {}'.format(error/num_steps))
    
    
    ax6 = plt.subplot2grid((4, 3), (1, 2), rowspan=1, colspan=1)
    ax6.plot(time_steps, y_des_interp, 'ro'); 
    ax6.plot(time_steps, y_act_interp, 'bo'); 
    ax6.plot(t_des, y_des, 'r'); 
    ax6.plot(t_act, y_act, 'b'); 
    
    error = np.sum(np.sqrt((y_des_interp - y_act_interp)**2))
    
    print 'Total error in y:', error
    print ' Average errror in y:', error/num_steps
    ax6.set_title(' Average errror in y: {}'.format(error/num_steps))
    
    
    ax7 = plt.subplot2grid((4, 3), (2, 1), rowspan=1, colspan=1)
    ax7.plot(time_steps, vx_des_interp, 'ro'); 
    ax7.plot(time_steps, vx_act_interp, 'bo'); 
    ax7.plot(t_des, vx_des, 'r'); 
    ax7.plot(t_act, vx_act, 'b'); 
    
    error = np.sum(np.sqrt((vx_des_interp - vx_act_interp)**2))
    
    print 'Total error in vx:', error
    print ' Average errror in vx:', error/num_steps
    ax7.set_title(' Average errror in vx: {}'.format(error/num_steps))
    
    ax8 = plt.subplot2grid((4, 3), (3, 1), rowspan=1, colspan=1)
    ax8.plot(time_steps, vy_des_interp, 'ro'); 
    ax8.plot(time_steps, vy_act_interp, 'bo'); 
    ax8.plot(t_des, vy_des, 'r'); 
    ax8.plot(t_act, vy_act, 'b'); 
    
    error = np.sum(np.sqrt((vy_des_interp - vy_act_interp)**2))
    
    print 'Total error in vy:', error
    print ' Average errror in vy:', error/num_steps
    ax8.set_title(' Average errror in vy: {}'.format(error/num_steps))
    
    plt.show()
    plt.close()
    '''
    plt.plot(q_des[0], q_des[1], 'r')
    plt.plot(q_act[0], q_act[1], 'b')
    plt.show()
    '''
