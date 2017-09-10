#!/bin/sh

#  cruise_control_test.py
#  
#
#  Created by Joseph Fuentes on 9/10/17.
#
import pyb
import math
import functions

# Simple test to tune the kp, ki, kd values for PID control
# Adjust values in 'functions.py'

# Duration = 10 seconds
# Speed = 50% duty cycle
functions.cruise_control(10000,50)
