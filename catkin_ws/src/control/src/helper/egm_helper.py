#!/usr/bin/env python
import time
import socket
import egm_pb2

def GetTickCount():
    return int((time.time() + 0.5) * 1000)

class EGMController():

    def __init__(self):
        self.UDP_PORT=6510
        self.sequenceNumber = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.UDP_PORT))
        data, self.addr = self.sock.recvfrom(1024)

    def get_robot_pos(self):
        try:
            egm_robot = egm_pb2.EgmRobot()
            data, addr = self.sock.recvfrom(1024)

            egm_robot.ParseFromString(data)
            pos_read = egm_robot.feedBack.cartesian.pos
            orient_read = egm_robot.feedBack.cartesian.orient
            pos_read = pos_read.x,  pos_read.y,  pos_read.z
            return pos_read,egm_robot
        except Exception as e:
            return None
