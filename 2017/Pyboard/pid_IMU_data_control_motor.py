#!/bin/sh

#  pid_IMU_data_control_motor.py
#  
#
#  Created by Joseph Fuentes on 8/23/17.
#
'''Use angle data from IMU to implement to control motor speed based on yaw
    
    Razor IMU:
    UART 1
    Baudrate= 57600
    
    Motors/ Drivers:
    A=Left Track
    A_EN= 'Y3'
    A_PH= 'Y5'
    B=Right Track
    B_EN= 'Y4'
    B_PH= 'Y6'

    '''
import pyb
import math
import time
from pyboard_razor_IMU import Razor
from pyboard_PID import PID
from motor import motor

#IMU:
razor_imu = Razor(1,57600)

#Motors:
DIRA = 'Y5'
PWMA = 'Y3'
TIMA = 10
CHANA = 1

DIRB = 'Y6'
PWMB = 'Y4'
TIMB = 11
CHANB = 1

######################### Defining Modules################################

#Map function to scale one set of values to another
def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#Reading the IMU to get angle data
def IMU_read():
    angle = razor_imu.get_one_frame()
    return angle[0]


################################## Defining Perameters ###########################
#Motor Object:
motor_left = motor(PWMA, DIRA, TIMA, CHANA)
motor_right = motor(PWMB, DIRB, TIMB, CHANB)

#PID object
kp_IMU  = 2
ki_IMU  = 0.001
kd_IMU  = 1
dt = 10000
max_out = 90
min_out = 10
pid_IMU = PID(kp_IMU, ki_IMU, kd_IMU, dt, max_out, min_out)

IMU_min =-180
IMU_max = 180
# Including sensitivity
sensitivity = 10
######################## Establish Angle / Start Motors ##########################
initial_read = IMU_read()
initial_angle = initial_read
motor_left.start(50,'cw')
motor_right.start(50,'cw')

#PID loop correcting motors speed in relation
while True:
    
    new_angle = IMU_read()

    if new_angle > (initial_angle + sensitivity): #The tank is drifting right
    
        angle_correction = pid_IMU.compute_output(initial_angle,new_angle)
        speed_correction= arduino_map(angle_correction, IMU_min, IMU_max, 0, 100)
        current_speed = motor_left.currentSpeed
        print(speed_correction)
        motor_left.set_speed(current_speed - speed_correction)
        motor_right.set_speed(current_speed + speed_correction)
        print(motor_left.currentSpeed)
        print(motor_right.currentSpeed)
        print(initial_angle)
        print(new_angle)

        pyb.delay(500)
        continue

    elif new_angle < (initial_angle - sensitivity): #The tank is drifting left
        angle_correction = pid_IMU.compute_output(initial_angle,new_angle)
        speed_correction= arduino_map(angle_correction, IMU_min, IMU_max, 0, 100)
        current_speed = motor_left.currentSpeed
        motor_left.set_speed(current_speed + speed_correction)
        motor_right.set_speed(current_speed - speed_correction)
        print(speed_correction)
        print(motor_left.currentSpeed)
        print(motor_right.currentSpeed)
        print(initial_angle)
        print(new_angle)

        pyb.delay(500)
        continue
    else:
        print("No Change")
        motor_right.set_speed(100)
        motor_left.set_speed(100)
        print(initial_angle)
        pyb.delay(500)
        continue













