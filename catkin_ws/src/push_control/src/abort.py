#!/usr/bin/env python

import rospy

rospy.init_node('abort')
rospy.set_param('is_exit', True)
