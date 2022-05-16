import socket
import time
import math
import random
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


def getDistance(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5



def getAngle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1) * 180 / math.pi





def isArrived(x1, y1, x2, y2):
    if abs(x1 - x2) < 0.2 and abs(y1 - y2) < 0.2:
        return True
    else:
        return False
def slowServoMove(i,d1,d2):
    command = 'CMD_SERVO#%d#%d\n' % (i, d1)
    s.send(command.encode('utf-8'))
    time.sleep(0.2)
    command = 'CMD_SERVO#%d#%d\n' % (i, d1+(d2-d1)/2)
    s.send(command.encode('utf-8'))
    time.sleep(0.2)
    command = 'CMD_SERVO#%d#%d\n' % (i, d2)
    s.send(command.encode('utf-8'))
    time.sleep(0.2)



