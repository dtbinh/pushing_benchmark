#!/usr/bin/env python


import sys
import os, time
import glob
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
import shutil



if __name__=='__main__':
    
    parser = optparse.OptionParser()
    parser.add_option('-f', '--file', action="store", dest='file_JSON', 
                      help='JSON file with information', default='')
    parser.add_option('-l', '--last', action='store_true', dest='use_last',
        help='Use last json file created', default=False)
    parser.add_option('-n', '--nlast', action="store", dest='nlast', type='int',
                  help='Which file you want', default=1)  
    (opt, args) = parser.parse_args()
    file_JSON = opt.file_JSON
    use_last = opt.use_last
    nlast = opt.nlast
    
   
    #Load actual trajectory
    #acutal_traj_JSON = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/'+list_act_files[counter]
    if file_JSON == '':
        file_JSON = '8Track_point_pusher_radius_0_15_vel_0_05_infinitypoint_FOM_8track_no_perturb_R_0_01.json'
    file_JSON = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/' + file_JSON
    if use_last:
        list_of_files = glob.glob('/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/*.json') 
        file_JSON = max(list_of_files, key=os.path.getctime)
    if nlast > 1:
        list_of_files = glob.glob('/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/*.json') 
        list_of_files.sort(key=os.path.getctime)
        file_JSON = list_of_files[-nlast]
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
    t_off = 0.1*0
    t_act = np.array(data['timeJSON'])-t_off
    it_cut = np.searchsorted(t_act, 0)
    t_act = t_act[it_cut::]
    x_act = np.array(data['xc'][0][it_cut::])
    y_act = np.array(data['xc'][1][it_cut::])
    ori_act = np.array(data['xc'][2][it_cut::])
    r_act = np.array(data['xc'][3][it_cut::])
    vx_act = np.array(data['us'][0][it_cut::])
    vy_act = np.array(data['us'][1][it_cut::])
    q_act = np.array(data['q_pusher_sensed'][it_cut::])
    # Relevant parameters:
    Q = np.array(data['Q'])
    Qf = np.array(data['Qf'])
    R = np.array(data['R'])
    h_mpc = np.array(data['h_mpc'])
    steps_mpc = np.array(data['steps_mpc'])
    if R.shape[0] < 3:
        vx_act = np.array(data['uc'][0])
        vy_act = np.array(data['uc'][1])
        vx_des = np.array(data['uc_desired'][0])
        vy_des = np.array(data['uc_desired'][1])
        
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
    plt.plot(x_des,y_des,'k', linewidth=5.0)
    plt.plot(x_des+0.4,y_des+0.04,'k'); 
    plt.plot(x_des-0.4,y_des-0.04,'k'); 
    plt.axis('equal')
    plt.show()
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
    average_error = np.sqrt((x_des_interp - x_act_interp)**2+(y_des_interp - y_act_interp)**2)
    
    
    print 'Total error:', error
    print ' Average errror:', error/num_steps
    ax1.set_title(' Average errror in trajectory: {}'.format(error/num_steps))

    ax2 = plt.subplot2grid((4,3), (0, 1), colspan=1, rowspan = 2)
    
    
    ax2.plot(vx_des_interp,vy_des_interp, 'ro'); 
    ax2.plot(vx_act_interp,vy_act_interp, 'bo'); 
    ax2.plot(vx_des,vy_des, 'r.'); 
    ax2.plot(vx_act,vy_act, 'b.'); 
    ax2.set_aspect('equal')
    
    '''
    
    ax2 = plt.subplot2grid((4,3), (1, 1), colspan=1, rowspan = 1)
    ax2.plot(time_steps,average_error, 'bo'); 
    
    '''
    #plt.show()
    
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
    Qt = np.transpose(Q)
    Qft = np.transpose(Qf)
    Rt = np.transpose(R)
    final_error = round(np.mean(average_error)*1000, 2)
    final_time = time.strftime('%X_%x').replace('/',':')
    fig_name = file_JSON[:-5] + '_err={}_R={}_Q={}_Qf={}_h={}_steps={}_t={}.png'.format(final_error,Rt[0],Qt[0],Qft[0],h_mpc[0],steps_mpc[0], final_time)
    plt.savefig(fig_name)
    plt.show()
    plt.close()
    print fig_name
    shutil.copyfile(file_JSON, file_JSON[:-5]+'_'+final_time+'.json')
    '''
    plt.plot(vx_des_interp,vy_des_interp, 'ro'); 
    plt.plot(vx_act_interp,vy_act_interp, 'bo'); 
    plt.plot(vx_des,vy_des, 'r.'); 
    plt.plot(vx_act,vy_act, 'b.'); 
    '''
    plt.plot(t_des, np.sqrt(np.power(vx_des,2)+np.power(vy_des,2)), 'r'); 
    plt.plot(t_act, np.sqrt(np.power(vx_act,2)+np.power(vy_act,2)), 'b'); 
    
    '''
    plt.plot(q_des[0], q_des[1], 'r')
    plt.plot(q_act[0], q_act[1], 'b')
    plt.axes().set_aspect('equal')
    '''
    plt.show()

