#!/usr/bin/env python

'''
Training script for classification cats/dogs classification problem
'''
import sys
import os, time
import optparse
import json
import numpy as np
import json
import rospy
sys.path.append(os.environ['PUSHING_BENCHMARK_BASE'] + '/Simulation/learning')
from main import modeSelector
# from load_data import load_data
sys.path.append('../helper')
import scipy
import matplotlib.pyplot as plt
import copy
import pdb
from push_control.srv import *

task_name = 'push_learning'

# ms = modeSelector(  os.environ['HOME'] +  '/Data/saved_models/push_learning_line_pusher/saved_models/' +  'mode_learning_mode_')
ms = modeSelector(  os.environ['HOME'] +  '/Data/saved_models/push_learning/saved_models/' +  'mode_learning_mode_')
x=.1*np.array([1,2,3,4])
mode_list  =ms.predict(x)
mode_schedule  = ms.index_convertor(mode_list)
ms.index_convertor(ms.predict(x))

def mode_learner(req):

    # t = time.time()
    mode_list  = ms.predict(x)
    mode_schedule  = ms.index_convertor(mode_list)
    ms.index_convertor(ms.predict(req.delta_x))
    tf = time.time()
    # print tf-t

    return MODE_SRVResponse(ms.index_convertor(ms.predict(req.delta_x)))

def mode_learner_main():
    rospy.init_node('mode_learner')
    s = rospy.Service('mode_learner', MODE_SRV, mode_learner)
    rospy.spin()

if __name__=='__main__':
    #~ import rospy
    mode_learner_main()
    #~ main()
    # task_name = 'push_learning'
    # ms = modeSelector(  os.environ['HOME'] +  '/Data/saved_models/push_learning/saved_models/' +  'mode_learning_mode_')
    # x=.1*np.array([1,2,3,4])
    # mode_list  =ms.predict(x)
    # mode_schedule  = ms.index_convertor(mode_list)
    # ms.index_convertor(ms.predict(x))

    # for i in range (100000):
    #     # print i
    #     t = time.time()
    #     ms.index_convertor(ms.predict(x))
    #     print 'time:', time.time() - t

    #~ pdb.set_trace()
    # K.clear_session()
