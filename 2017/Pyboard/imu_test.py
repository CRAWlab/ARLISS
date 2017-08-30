#!/bin/sh

#  imu_test.sh
#  
#
#  Created by Joseph Fuentes on 8/24/17.
#


import pyb
import UART
import math
import time
from pyboard_razorIMU import Razor

razor_imu = Razor(1,57600)
while True:
    angle = razor_imu.get_one_frame()
    print(angle)
    time.sleep(0.5)
