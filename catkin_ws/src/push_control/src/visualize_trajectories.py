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
import matplotlib.pyplot as plt
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from shapely.geometry import Point, Polygon

def get_tranf_matrix(extrinsics, camera_matrix):
    # Matrix for extrinsics
    translate = extrinsics[0:3]
    quaternion = extrinsics[3:7]
    extrinsics_matrix = np.dot(tfm.compose_matrix(translate=translate),tfm.quaternion_matrix(quaternion))[0:3]
    
    # Transformation matrix
    transformation_matrix = np.dot(camera_matrix,extrinsics_matrix)
    
    return transformation_matrix

def load_json_file(filename):
    
    with open(filename) as json_data:
        data = json.load(json_data)
        
    return data



if __name__=='__main__':
    
    #Default camera info
    camera_matrix = np.array([[1391.26, 0.0, 939.721], [0.0, 1391.26, 531.336], [0.0, 0.0, 1.0]])    
    extrinsics = np.array([-0.011075975475308975, -0.15364479808584514, 1.0761464358218347, -0.6417773633094387, -0.6349911003061667, 0.30488743414700425, -0.3032355041462077])
    transformation_matrix = get_tranf_matrix(extrinsics, camera_matrix)
    
    #Desired trajectory
    desired_traj_JSON = os.environ['PUSHING_BENCHMARK_BASE'] + '/Data/'+'8Track_point_pusher_radius_0.15_vel_0.08_3_laps.json'
    des_var = load_json_file(desired_traj_JSON)['Matrices']
    
    
    #Actual trajectory
    #   Load from JSON
    ''' 
    acutal_traj_JSON = '/home/mcube/pushing_benchmark/catkin_ws/src/push_control/src/Data/'+'8Track_point_pusher_radius_0.11_vel_0.08_3_lapsexperiments.json'
    act_var = load_json_file(actual_traj_JSON)
    '''
    #   Load from BAG
    data_filename = '/home/mcube/pushing_benchmark_data/2018-05-02-11-04-558Track_point_pusher_radius_0_15_vel_0.08_3_laps_horizon_5'
    print('Loading data from BAG file ...')
    if not os.path.isfile(data_filename+'.npz'):
        bridge = CvBridge()
        data = {}
        data['images'] = []; data['t_images'] = []; data['xc'] = []; data['timeJSON'] = []
        with rosbag.Bag(data_filename+'.bag', 'r') as bag:
                for topic, msg, t in bag.read_messages():
                    if topic == '/viewer/image_raw':
                        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
                        data['images'].append(cv_image)
                        data['t_images'].append(t.to_sec())
                    if topic == '/xc':
                        data['xc'].append(np.array(msg.data))
                        data['timeJSON'].append(t.to_sec())
        np.savez(data_filename, data)
    else:
        data = np.load(data_filename+'.npz')['arr_0'][()]
    act_var = data
    print('Done!')
    
    # Video time
    fps = np.floor(len(data['t_images'])/(data['t_images'][-1]-data['t_images'][0]))
    time_steps = np.arange(data['t_images'][0], data['t_images'][-1], 1.0/fps)
    print 'fps: ', fps
    
    # Projection offsets
    x_off = 0.345
    y_off = 0
    z_off = 0.015
    
    #Object properties
    obj_len = 0.09

    # t,x,y,z of the desired trajectory
    t_des = np.array(des_var['t_star'])[:,0]
    x_des = np.array(des_var['xc_star'])[:,0]+x_off
    y_des = np.array(des_var['xc_star'])[:,1]+y_off
    ori_des = np.array(des_var['xc_star'])[:,2]
    z_des = x_des*0+z_off
    
    # t,x,y,z of desired trajectory
    t_act = np.array(act_var['timeJSON'])
    x_act = np.array(act_var['xc'])[:,0]+x_off
    y_act = np.array(act_var['xc'])[:,1]+y_off
    ori_act = np.array(act_var['xc'])[:,2]
    z_act = x_act*0+z_off
    
    # Interpolate for both trajectories:
    t_des = t_des - t_des[0] + time_steps[0]  #Assumes it starts at the same time that the first image
    x_des = np.interp(time_steps, t_des, x_des)
    y_des = np.interp(time_steps, t_des, y_des)
    z_des = np.interp(time_steps, t_des, z_des)
    ori_des = np.interp(time_steps, t_des, ori_des)
    
    x_act = np.interp(time_steps, t_act, x_act)
    y_act = np.interp(time_steps, t_act, y_act)
    z_act = np.interp(time_steps, t_act, z_act)
    ori_act = np.interp(time_steps, t_act, ori_act)
    
    # Project trajectories into pixel space
    vector_des = np.array([x_des, y_des, z_des, z_des*0+1])
    pixels_des = np.dot(transformation_matrix, vector_des)
    pix_x_des = np.divide(pixels_des[0], pixels_des[2])
    pix_y_des = np.divide(pixels_des[1], pixels_des[2])
    
    vector_act = np.array([x_act, y_act, z_act, z_act*0+1])
    pixels_act = np.dot(transformation_matrix, vector_act)
    pix_x_act = np.divide(pixels_act[0], pixels_act[2])
    pix_y_act = np.divide(pixels_act[1], pixels_act[2])
        
    # Initialize time and figure
    fig = plt.figure()
    it_im = 0
    t_im = np.array(data['t_images']) - data['t_images'][0]
    time_steps -= time_steps[0]
    
    # For each time steps / frame
    for it in range(time_steps.shape[0]): #range(x_act.shape[0]):
        print 'Building frame:', it
        # Get real image
        while t_im[it_im]< time_steps[it]: #What is the right image from the sequence to plot?
            it_im += 1
        plt.imshow(data['images'][it_im])
        
        # Locate object position
        obj_pose = np.array([x_act[it], y_act[it], z_act[it]])  
        obj_ori = ori_act[it]
        # Create square representing the object
        square = np.array([[-1, -1],[1, -1],[1, 1], [-1, 1],[-1, -1]])*obj_len/2;
        c, s = np.cos(obj_ori), np.sin(obj_ori)
        R = np.array(((c,-s), (s, c)))
        act_obj = np.dot(R, np.transpose(square))
        act_obj = np.insert(act_obj, 2, 0, axis = 0)
        for i in range(3):
            act_obj[i] = act_obj[i]+obj_pose[i]
        act_obj = np.insert(act_obj, 3, 1, axis = 0)
        # Locate object pixels
        pixels_obj = np.dot(transformation_matrix, act_obj)
        a = pixels_obj[0]/pixels_obj[2]
        b = pixels_obj[1]/pixels_obj[2]
        polygon = Polygon([(a[0], b[0]), (a[1], b[1]), (a[2], b[2]), (a[3], b[3])])
        
        # Plot desired trajectory        
        x = []; y = []
        for it_2 in range(x_des.shape[0]):
            point = Point(pix_x_des[it_2],pix_y_des[it_2])
            if polygon.contains(point):
                plt.plot(x,y, 'k')
                x = []; y = []
            else:
                x.append(pix_x_des[it_2]); y.append(pix_y_des[it_2])
        if len(x) >  0:
            plt.plot(x,y, 'k')
            
        # Plot actual trajectory
        x = []; y = []
        for it_2 in range(x_act.shape[0]):
            point = Point(pix_x_act[it_2],pix_y_act[it_2])
            if polygon.contains(point) and t_act[it_2] < t_act[it]:
                plt.plot(x,y, 'b')
                x = []; y = []
            else:
                x.append(pix_x_act[it_2]); y.append(pix_y_act[it_2])
        
        # Plot properties
        plt.axis('off')
        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)

        # Convert canvas to image
        fig.canvas.draw()
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        #   Img is rgb, convert to opencv's default bgr
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)[120:380,150:600] #[90:410,220:820]
        
        # Initialize video
        if it == 0:
            video_name = data_filename + '.avi'
            height, width, layers = img.shape
            fourcc = cv2.VideoWriter_fourcc(*'DIVX')
            video = cv2.VideoWriter(video_name,fourcc, fps, (width,height)) 
        
        # Record video
        video.write(img)

        # Create new figure
        #img_name = 'Images/push_frame_{}.png'.format(it)
        #plt.savefig(img_name,bbox_inches='tight',pad_inches = 0)
        plt.close()
        fig = plt.figure()
    
    cv2.destroyAllWindows()
    video.release()
    
    
