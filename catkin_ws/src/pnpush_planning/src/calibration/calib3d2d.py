#!/usr/bin/env python
import numpy as np
from scipy.optimize import least_squares
import tf.transformations as tfm
import cv2
from numpy import array as npa
import os, json, sys


if len(sys.argv) > 2:
    save_dir = os.environ["PUSHING_BENCHMARK_BASE"] + "/" +sys.argv[2] +"/"
else:
    save_dir = os.environ["PUSHING_BENCHMARK_BASE"] + "/camera_calib/"
    
def quat_to_rod(q):
    # q = [qx, qy, qz, qw]
    rot = tfm.quaternion_matrix(q)[0:3][:, 0:3]
    dst, jacobian = cv2.Rodrigues(rot)
    return dst.T.tolist()[0]


def rod_to_quad(r):
    # q = [qx, qy, qz, qw]
    rotmat , jacobian = cv2.Rodrigues(npa(r))
    rotmat = np.append(rotmat, [[0,0,0]], 0)  
    rotmat = np.append(rotmat, [[0],[0],[0],[1]], 1)  
    q = tfm.quaternion_from_matrix(rotmat)
    return q.tolist()

class Program:
    def __init__(self, point3Ds, point2Ds, x0, x0_int):
        self.point3Ds = point3Ds
        self.point2Ds = point2Ds
        self.x0 = x0
        self.x0_int = x0_int

    def obj_func(self, cam):

        fx, fy = self.x0_int[0], self.x0_int[1]
        cx, cy = self.x0_int[2], self.x0_int[3]
        distCoeffs = (0.0, 0.0, 0.0, 0.0, 0.0)   ## hack no distortion
        tvec = cam[0:3]  # x,y,z
        rvec = cam[3:6]  # rodrigues

        # project
        #point2Ds_p = cv.project(point3Ds, cam)
        cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        point2Ds_p, jacobian = cv2.projectPoints(npa(self.point3Ds, dtype=np.float), rvec, tvec, cameraMatrix, distCoeffs)
        #print point2Ds_p
        point2Ds_pp = [list(p[0]) for p in point2Ds_p]
        diff = npa(point2Ds_pp, dtype=np.float) - npa(self.point2Ds, dtype=np.float)
        diff = diff.flatten(1)
        #import pdb;
        #pdb.set_trace()
        #print diff
        res = np.linalg.norm(diff)
        print res / 27.0
        return diff

    def run(self):
        
        res_1 = least_squares(self.obj_func, self.x0)
        
        print '--original--'
        trans =  self.x0[0:3]
        rod =  self.x0[3:6]
        q = rod_to_quad(rod)
        print 'pose', list(trans) + list(q)
        
        
        print '\n--optimized--'
        trans = res_1.x[0:3]
        rod = res_1.x[3:6]
        q = rod_to_quad(rod)
        print 'pose', list(trans) + list(q)
        
    
        
        transform = tfm.concatenate_matrices(tfm.translation_matrix(trans), tfm.quaternion_matrix(q))
        inversed_transform = tfm.inverse_matrix(transform)
        translation = tfm.translation_from_matrix(inversed_transform)
        quaternion = tfm.quaternion_from_matrix(inversed_transform)
        pose =  translation.tolist() + quaternion.tolist()
        print 'webcam_T_robot:', " ".join('%.8e' % x for x in pose)
        print 'K: ', [self.x0_int[0], 0.0, self.x0_int[2], 0.0, self.x0_int[1], self.x0_int[3], 0.0, 0.0, 1.0]
        print 'P: ', [self.x0_int[0], 0.0, self.x0_int[2], 0.0, 0.0, self.x0_int[1], self.x0_int[3], 0.0, 0.0, 0.0, 1.0, 0.0]
        #print res_1
        return res_1.x
        

if __name__ == '__main__':
    color1 = (0,255,255)
    color2 = (0,0,255)
    color3 = (255,0,255)
    
    # x0_ext_old = [1.08592196e-01, -5.96469288e-01, 6.09164354e-01] + 
        # [9.05378371e-01, 3.06042346e-02, -5.82887034e-02, -4.19470873e-01]) # viewer
    x0_ext_old = [7.16834068e-01, -7.68849430e-02, 3.78838750e-01] + [6.89344221e-01, 6.44350485e-01, -2.20748124e-01, -2.46753445e-01]  # observer
    
    
    transform = tfm.concatenate_matrices(tfm.translation_matrix(x0_ext_old[0:3]), tfm.quaternion_matrix(x0_ext_old[3:7]))
    inversed_transform = tfm.inverse_matrix(transform)
    translation = tfm.translation_from_matrix(inversed_transform)
    quaternion = tfm.quaternion_from_matrix(inversed_transform)

    x0_ext =  translation.tolist() + quat_to_rod(quaternion.tolist())

    if sys.argv[1] == 'observer':
        if sys.argv[3] == '640':
            x0_int = [617.605, 617.605, 305.776, 238.914, 0.0,0.0,0.0,0.0,0.0]  # observer
        elif sys.argv[3] == '1080':
            x0_int = [1389.61, 1389.61, 927.997, 537.558, 0.0,0.0,0.0,0.0,0.0]  # observer
    else:
        if sys.argv[3] == '640':
            x0_int = [618.337, 618.337, 310.987, 236.15, 0.0,0.0,0.0,0.0,0.0] # viewer 
        elif sys.argv[3] == '1080':
            x0_int = [1391.26, 1391.26, 939.721, 531.336, 0.0,0.0,0.0,0.0,0.0]  # viewer
            
    x0 = x0_ext
    
    with open(save_dir + 'data.extracted2d.json') as data_file:
        data = json.load(data_file)
        
    point3d = [d["cross3d"][0:3] for d in data]
    point2d = [d["cross2d"] for d in data]
    #print point3d
    p = Program(point3d, point2d, x0, x0_int)
    cam = p.run()
    
    # show reprojection
    fx, fy = x0_int[0], x0_int[1]
    cx, cy = x0_int[2], x0_int[3]
    distCoeffs = (0,0,0,0,0)
    tvec = cam[0:3]  # x,y,z
    rvec = cam[3:6]  # rodrigues

    # project
    cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    point2Ds_p, jacobian = cv2.projectPoints(npa(point3d, dtype=np.float), rvec, tvec, cameraMatrix, distCoeffs)
    point2Ds_p_nd, jacobian = cv2.projectPoints(npa(point3d, dtype=np.float), rvec, tvec, cameraMatrix, (0.,0.,0.,0.,0.))

    for i, d in enumerate(data):
        image_viz = cv2.imread(save_dir + d['pic_path'])
        
        
        pt_int = tuple([int(round(p)) for p in d['cross2d']])
        cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color1)
        cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color1)
        
        pt_int = tuple([int(round(p)) for p in point2Ds_p_nd[i][0]])
        cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color3)
        cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color3)
        
        pt_int = tuple([int(round(p)) for p in point2Ds_p[i][0]])
        cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color2)
        cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color2)
        cv2.imshow("image", image_viz)
        
        
        while True:
            # display the image and wait for a keypress
            key = cv2.waitKey(3) & 0xFF
            if key == ord("n"):
                break
                
    #cv2.undistortPoints(npa(point3d, dtype=np.float), point2d, K, dist_coef)
