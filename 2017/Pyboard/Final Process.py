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
    -Determine the orienation of how the tank has landed
    -Establish fix and get first location
    -Begin output signal to burn parachute 
    -Wait time period [burn_time] to insure parachute has finished burning off
    -End relay output signal
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
from pyboard_razor_IMU import Razor
from pyb import ExtInt
from pyb import Pin
from micropyGPS import MicropyGPS
from motor import motor
import time
import math
import functions
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

def pps_callback(line):
    # print("Updated GPS Object...")
    global new_data  # Use Global to trigger update
        new_data = True

pps_pin = 'X8'
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP,
                    pps_callback)
################### Global Variables ######################
burn_time = 10000 #Time it takes to Burn Nychron wire completely [ms]
backup_timer = 5400000
altitude_concurrent_timer = 3600000
acceptable_dist_from_launch = 1000
acceptable_altitude_change = 25
black_rock_alt_ft = 3907
black_rock_alt_m = 1191
threshold = 1
distance_tolerance = 3


################### Begin Process #########################
process_start= pyb.millis()

xbee.write('\nprogram initiated at: {}'.format(my_gps.timestamp))
xbee.write('\ntarget point: {}'.format(finish_point))
xbee.write('\nlaunching at: {}'.format(launch_point))

#Altitude check to see if rover has landed
while True:
    my_gps.update(chr(uart.readchar()))
    if my_gps.altitude != 0
        altitude = mygps.altitude
        if altitude =< (black_rock_alt_m + threshold):
            xbee.write('\nRover has landed, Altitude: {}'.format(altitude))
            break
        elif pyb.elapsed_millis(start) >= backup_timer:
            break
        else:
            xbee.write('\nRover is in descent')


#Checking orientation
orientation_check = functions.get_landing_orientation()
#defining which gps is in use
my_gps = orientation_check[0]
# sending the orientation to observer
xbee.write(orientation_check[1])


#Establishing landing point
while True:
    my_gps.update(chr(uart.readchar()))
        if my_gps.latitude[0] != 0:
            xbee.write('\nChecking GPS and altitude to see if landed...')
            landing_lat = functions.convert_latitude(my_gps.latitude)
            landing_long = functions.convert_longitude(my_gps.longitude)
            landing_point = (landing_lat, landing_long)
            xbee.write('\nLanding point: {}'.format(landing_point))

#Calculate Distance to goal
dist_from_goal = functions.calculate_distance(finish_point, landing_point)
xbee.write('\nDistance from Goal: {}'.format(dist_from_goal))

#Burn Parachute
functions.burn_parachute(burn_time)
functions.move_forward(100)
pyb.delay(10000)
motor.stop()

######################## Begin Navigation loop #######################
#Establishing first location after separation of parachute
while True:
    if new_data == True:
        while True:
            my_gps.update(chr(uart.readchar()))
            if my_gps.latitude[0] != 0:
                start_lat = functions.convert_latitude(my_gps.latitude)
                start_long = functions.convert_longitude(my_gps.longitude)
                start_point = (start_lat, start_long)
                new_data = False
                past_point2 = None
                break
            else:
                new_data = False

xbee.write('\nI will begin navigation at this location: {}'.format(past_point))
dist_from_goal = functions.calculate_distance(finish_point, start_point)
degree_to_turn = functions.calculate_bearing(start_point,finish_point)
#turn rover degree

#Start Movement
while True:
    functions.move_forward(100)
    functions.imu_pid()
    time.sleep(20)
    functions.stop()
    # log.write("\nStopping!")
    # motor.stop()
    # time.sleep(0.1) # The sleep is for the slow stop; allows for a complete stop
    my_gps.update(chr(uart.readchar()))
    pres_lat = convert_latitude(my_gps.latitude)
    pres_long = convert_longitude(my_gps.longitude)
    present_point = (pres_lat, pres_long)
    dist_from_goal = functions.calculate_distance(finish_point,present_point)
    degree_to_turn = functions.calculate_bearing(present_point,finish_point)
    #turn rover degree
    if dist_from_goal <= distance_tolerance:
        functions.stop()
        xbee.write('\nDestination reached, present location: {}'.format(present_point))
        break
    else:
        continue
    




