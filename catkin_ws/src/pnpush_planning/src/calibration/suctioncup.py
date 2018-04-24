#!/usr/bin/env python
# nx,ny,nz -> theta -> distance, K -> Fx, Fy, Fz

import json
import numpy as np
import tf.transformations as tfm
import sys
import matplotlib.pyplot as plt
import matplotlib as mpl
import time
from numpy import array as npa
from PySide.QtGui import QApplication, QImage
import io

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


def main(argv):
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
    for i in xrange(n):            

            
        cs.append(i)
        pose = data['object_pose'][i][1:8]
        xyz = npa(pose[0:3]) - (npa(data['tip_pose'][i][1:4])-npa(data['tip_pose'][0][1:4])).tolist()
        pose = xyz.tolist() + pose[3:7]
        matrix = tfm.quaternion_matrix(pose[3:7])
        #z = (1-matrix[2][2]**2)**0.5
        #z = matrix[2][2]
        x,y,z = pose[0], pose[1], pose[2]
        th = np.arccos(matrix[2][2])
        
                
        ft = data['ft_wrench'][i][1:7]
        #fxy = (ft[0]*ft[0] + ft[1]*ft[1])**0.5
        fx = ft[0]
        
        values = {'x':x, 'y': y, 'z': z, 'th':th, 'fx': ft[0], 'fy': ft[1], 'fz': ft[2], 'tx': ft[3], 'ty': ft[4], 'tz': ft[5]}
        
        xs.append(values[xaxis])
        ys.append(values[yaxis])
        #fxys.append(fxy)
        #fz = ft[1]
        #fzs.append(fz)
    plt.xlabel(xaxis)
    plt.ylabel(yaxis)
    
    plt.scatter(xs, ys, c=cs)
    plt.show()



if __name__=='__main__':
    main(sys.argv)
