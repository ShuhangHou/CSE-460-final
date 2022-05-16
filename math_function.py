import math
import numpy
import random


def distance(x1, y1, x2, y2):
    return math.sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))


def angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1) * 180 / math.pi


def ismoving(pre_x, pre_y, x, y):
    d = distance(pre_x, pre_y, x, y)
    if d < 0.05:
        return False
    else:
        return True

