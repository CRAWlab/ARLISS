#!/bin/sh

#  Process.py
#  ARLISS 2017
#
#  Created by Joseph Fuentes on 8/22/17.
#
'''This file is a higher level process for the ARLISS robot project
    
    Process Algorithm:
    -Accept goal coordinates
    -Await ascent
    -Detach from rocket
    -Detect descent
    -Once rover is no longer descending communicate that it has landed
    -Begin output signal to burn parachute 
    -Wait time period [delta_burn] to insure parachute has finished burning off 
    -End relay output signal
    -Determine the orienation of how the tank has landed 
    -Aquire Current location 
    -Drive forward for time period [delta_foward]
    -Aquire new location and bearing of rover
    -Calculate Distance to target 
    -Turn to face target
    -Begin Travel loop
        -Maintain Bearing 
            IMU:
                -Monitor angular rotation 
                -Send Signal To motors of rotation is detected
            Motors: 
                -Use PID's to maintain desired speed 
                -Coorporate with the IMU to ensure straight travel 
        -Distance From Target 
            GPS: 
                -Check Position as a function 
                    [Position needs to be checked more frequently as the rover is further from he target]
                    
'''
#################### Import Libraries#######################
import pyb
from pyb import UART
from pyb import ExtInt
from pyb import Pin
from micropyGPS import MicropyGPS
import motor
import time
import math

################### User Input ############################
finish_point=
launch_point=

################# End User Input ##########################

###########################################################

''' Pinout Section for the Pyboard
    
    Motors/ Drivers:
    A=Left Track
    A_EN= 'Y3'
    A_PH= 'Y5'
    B=Right Track
    B_EN= 'Y4'
    B_PH= 'Y6'
    
    Encoder:
    A=Left Track
    enc_A_chan_A='X1'
    enc_A_chan_B='X2'
    B=Right Track
    enc_B_chan_A='Y7'
    enc_B_chan_B='Y8'
    GPS:
    -Top GPS:
    UART 3
    Baudrate=9600
    -Bottom GPS:
    UART 6
    Baudrate=9600
    
    IMU: 
    Internal 
    Razor IMU 
        UART 1 
        Baudrate= 57600
    
    XBEE:
    UART 2
    Baudrate=115200
    
    Relay = 'Y11'
'''
################## Peripherial Setup ######################
razor_imu_uart = UART(1,57600)
xbee = UART(2, 115200)
top_gps_uart = UART(3,9600)
bot_gps_uart = UART(6,9600)

#Instantiate the micropyGPS object
top_gps = MicropyGPS(-7)
bot_gps = MicropyGPS(-7)

#Quadrature encoders:
enc_A_chan_A = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_A_chan_B = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_B_chan_A = pyb.Pin('Y7', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)
enc_B_chan_B = pyb.Pin('Y8', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)

#Motors:
DIRA = 'Y5'
PWMA = 'Y3'
TIMA = 10
CHANA = 1

DIRB = 'Y6'
PWMB = 'Y4'
TIMB = 11
CHANB = 1

#Motor Object:
motorA = motor(PWMA, DIRA, TIMA, CHANA)
motorB = motor(PWMB, DIRB, TIMB, CHANB)

# Create relay object:
relay = Pin('Y11', Pin.OUT_PP)

################### Begin Process #########################
process_start= pyb.millis()









