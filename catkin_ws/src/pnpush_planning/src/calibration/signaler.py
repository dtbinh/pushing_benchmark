#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Do contour following with the poker around the shape using the force
# torque reading.

import sys
import numpy as np
from ik.roshelper import ROS_Wait_For_Msg
from ik.roshelper import lookupTransform
from ik.ik import setSpeed
from geometry_msgs.msg import WrenchStamped
import tf
import tf.transformations as tfm
import rospy
import json
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
import sensor_msgs.msg
import geometry_msgs.msg
import os
import scipy.io as sio
import socket

setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZero = rospy.ServiceProxy('/ft_left/zero', Zero)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    print 'setCart', param
    #pause()
    setCartRos(*param)

def pause():
    print 'Press any key to continue'
    raw_input()

def callback(data):
    global sock
    v = ftmsg2list(data)
    nn = norm(v)
    #print nn
    if nn > 5:
        sock.sendall('x')

def ftmsg2list(ftmsg):
    return [ftmsg.wrench.force.x,ftmsg.wrench.force.y,ftmsg.wrench.force.z]

def norm(vect):
    vect = np.array(vect)
    return np.sqrt(np.dot(vect, vect))

def main(argv):
    global sock
    rospy.init_node('get_averaged_ft', anonymous=True)
    
    setZero()
    rospy.sleep(0.5)
    rospy.Subscriber('/ft_left/netft_data', geometry_msgs.msg.WrenchStamped, callback)
    
    TCP_PORT=5004
    sequenceNumber = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect the socket to the port where the server is listening
    server_address = ('192.168.1.2', TCP_PORT)
    print >>sys.stderr, 'connecting to %s port %s' % server_address
    sock.connect(server_address)
    
    rospy.spin()
    
if __name__=='__main__':
    main(sys.argv)




