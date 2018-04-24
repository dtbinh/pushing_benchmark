#!/usr/bin/env python
# nx,ny,nz -> theta -> distance, K -> Fx, Fy, Fz

import json
import numpy as np
import tf.transformations as tfm
import sys
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation as anim
import time
from numpy import array as npa
from PySide.QtGui import QApplication, QImage
import io

# ROS
import rospy, tf
from ik.roshelper import lookupTransformList
from geometry_msgs.msg import WrenchStamped

def add_clipboard_to_figures():
    # use monkey-patching to replace the original plt.figure() function with
    # our own, which supports clipboard-copying
    oldfig = plt.figure

    def newfig(*args, **kwargs):
        fig = oldfig(*args, **kwargs)
        def clipboard_handler(event):
            if event.key == 'ctrl+c':
                # store the image in a buffer using savefig(), this has the
                # advantage of applying all the default savefig parameters
                # such as background color; those would be ignored if you simply
                # grab the canvas using Qt
                buf = io.BytesIO()
                fig.savefig(buf)
                
                QApplication([]).clipboard().setImage(QImage.fromData(buf.getvalue()))
                buf.close()

        fig.canvas.mpl_connect('key_press_event', clipboard_handler)
        return fig

    plt.figure = newfig


def ftmsg2listandflip(ftmsg):
    return [-ftmsg.wrench.force.x,ftmsg.wrench.force.y,-ftmsg.wrench.force.z,
            -ftmsg.wrench.torque.x,ftmsg.wrench.torque.y,-ftmsg.wrench.torque.z]
            
def translate_ft(ft):
    A = npa([[0,-ft[2],ft[1]],[ft[2],0,-ft[0]],[-ft[1], ft[0], 0]]).T
    b = npa(ft[3:6])
    print "det", np.linalg.det(A.T), "multiply", ft[0]*ft[1]*ft[2]
    print A
    r = np.linalg.solve(A, b)
    
    # calculate new r
    print r
    newr = r - npa([0,0,0.14])
    
    newft = ft[0:3] + np.cross(f, r).tolist()
    return newft

def cross2d(a,b):
    return a[0]*b[1] - a[1]*b[0]

def main(argv):
    rospy.init_node('suctioncup_interactive')
    lr = tf.TransformListener()
    
    
    json_filepath = argv[1]
    
    xaxis = argv[2]
    yaxis = argv[3]
    
    with open(json_filepath, 'r') as infile:
        data = json.load(infile)
    
    n = min(min(len(data['ft_wrench']), len(data['object_pose'])), len(data['tip_pose']))
    print len(data['ft_wrench']), len(data['object_pose']), len(data['tip_pose'])
    zs = []
    xs = []
    ys = []
    cs = []
    
    # add copy feature
    add_clipboard_to_figures()
    mpl.rcParams["savefig.directory"] = '/home/mcube/pushdata/suction/figure_userobot/'
    
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    ax.set_xlabel(xaxis)
    ax.set_ylabel(yaxis)
        
    #object_frame = '/vicon/CalibPlate/CalibPlate'
    object_frame = '/vicon/InsertObject/InsertObject'
    
    nom = lookupTransformList('/map',object_frame, lr)
    def update(i):
        # get object pose in world
        # get ft measurement
        cs.append(i)
        pose = lookupTransformList('/map',object_frame, lr)
        ft = ftmsg2listandflip(rospy.wait_for_message('netft_data', WrenchStamped))
        xyz = npa(pose[0:3]) - npa(nom[0:3])
        pose = xyz.tolist() + pose[3:7]
        matrix = tfm.quaternion_matrix(pose[3:7])
        print matrix, '\n'
        print matrix[0][2], matrix[2][0], 
        x,y,z = pose[0], pose[1], pose[2]
        thx = np.arcsin(matrix[2][0])
        thy = np.arcsin(matrix[2][1])
        
        #ft = data['ft_wrench'][i][1:7]
        
        #ft = translate_ft(ft);
        
        print thx, thy
        R2D = 180.0 / np.pi
        r = 0.04
        values = {'x':x, 'y': y, 'z': z, 'sinthx':np.sin(thx), 'thx':thx*R2D, 'thy':thy*R2D, 
                  'fx': ft[0], 'fy': ft[1], 'fz': ft[2], 
                  'tx': ft[3], 'ty': ft[4], 'tz': ft[5], 
                  'proj_force_x': ft[0]*np.cos(thx) + ft[2]*np.sin(thx), 
                  'proj_force_y': ft[1]*np.cos(thy) + ft[2]*np.sin(thy),
                  'torque_center': cross2d([matrix[0][0]*r, matrix[2][0]*r],[ft[0], ft[2]])}
        
        xs.append(values[xaxis])
        ys.append(values[yaxis])
        
        if(i%10==0):
            plt.scatter(xs, ys, c=cs)

    a = anim.FuncAnimation(fig, update, frames=xrange(0,n,10), repeat=False, interval=10)  # interval in ms
    plt.show(block=True)


if __name__=='__main__':
    main(sys.argv)
