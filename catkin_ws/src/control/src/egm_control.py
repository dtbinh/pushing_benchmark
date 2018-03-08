#!/usr/bin/env python
from helper import egm_pb2
from helper import egm_helper
from time import sleep
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from ik.helper import quat_from_yaw, qwxyz_from_qxyzw, transform_back
import tf

if __name__=='__main__':
    rospy.init_node('egm_control')
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size = 2)
    br = tf.TransformBroadcaster()

    theta = 0.0
    flag = True
    rospy.sleep(1)
    starttick = egm_helper.GetTickCount()

    E = egm_helper.EGMController()

    position = np.array([300, 0, 178.2])
    rate = rospy.Rate(250) # 250hz
    while flag:
        rate.sleep()
        # 1. get robot pose
        robot_pos, egm_robot = E.get_robot_pos()

        # 2. if robot connected, do the following

        # 3. publish joint position and cartesian position
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = [j for j in egm_robot.feedBack.joints.joints]
        js.velocity = [0.0 for i in xrange(6)]
        js.effort = [0.0 for i in xrange(6)]
        joint_pub.publish(js)

        # send desired position
        egm_sensor_write = egm_pb2.EgmSensor()

        header = egm_pb2.EgmHeader()
        header.mtype = egm_pb2.EgmHeader.MSGTYPE_CORRECTION
        header.seqno = E.sequenceNumber
        E.sequenceNumber += 1
        header.tm = egm_helper.GetTickCount()-starttick
        egm_sensor_write.header.CopyFrom(header)

        pos = egm_pb2.EgmCartesian()
        position = position + 1./250*np.array([50.,0.,0.])
        print position
        # pos.x, pos.y, pos.z = capit(smooth_it(pos_read, (xd,yd,zd)))
        pos.x, pos.y, pos.z = position[0], position[1], position[2]
        orient = egm_pb2.EgmQuaternion()
        q = transform_back([0,0,0,0,1,0,0], [0,0,0]+quat_from_yaw(theta))[3:7]
        qtuple = tuple(qwxyz_from_qxyzw(q))
        #pubFrame(br, [0,0,0] + q)

        #print qtuple
        orient.u0, orient.u1, orient.u2, orient.u3 = qtuple
        pose = egm_pb2.EgmPose()
        pose.orient.CopyFrom(orient)
        pose.pos.CopyFrom(pos)
        planned = egm_pb2.EgmPlanned()
        planned.cartesian.CopyFrom(pose)
        egm_sensor_write.planned.CopyFrom(planned)
        #print 'write', egm_sensor_write
        sent = E.sock.sendto(egm_sensor_write.SerializeToString(),E.addr)

    sock.close()
    print 'End of program'
