#!/usr/bin/env python3

#ENPM661 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#Maitreya Ravindra Kulkarni, UID: 117506075
#jpittma1@umd.edu and mkulk98@umd.edu 
#Project #3 Phase 2

import numpy as np
import math
from Obstacle import *


def halfRound(n):
    val = round(2*n)/2
    if (val == 10):
      val -= 0.5
    return val


def toRadian(angle):
    return np.pi * angle / 180
def toDegree(angle):
    return 180 * angle / np.pi