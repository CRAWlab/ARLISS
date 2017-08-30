#!/bin/sh

#  functions.py
#  
#
#  Created by Joseph Fuentes on 8/28/17.
#

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


################### Defining Functions ####################
#GPS
EARTH_RADIUS = 6370000
MAG_LAT = 82.7
MAG_LON = -114.4

direction_names = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S",
                   "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]

directions_num = len(direction_names)
directions_step = 360 / directions_num

def calculate_bearing(position1, position2):
    ''' Calculate the bearing between two GPS coordinates
        Equations from: http://www.movable-type.co.uk/scripts/latlong.html
        Input arguments:
        position1 = lat/long pair in decimal degrees DD.dddddd
        position2 = lat/long pair in decimal degrees DD.dddddd
        Returns:
        bearing = initial bearing from position 1 to position 2 in degrees
        Created: Joshua Vaughan - joshua.vaughan@louisiana.edu - 04/23/14
        Modified:
        *
        '''
    print(position1[0])
    lat1 = math.radians(position1[0])
    long1 = math.radians(position1[1])
    lat2 = math.radians(position2[0])
    long2 = math.radians(position2[1])
    
    dLon = long2 - long1
    
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
    
    return bearing


def calculate_distance(position1, position2):
    ''' Calculate the distance between two lat/long coords using simple
        cartesian math
        
        Equation from: http://www.movable-type.co.uk/scripts/latlong.html
        
        Input arguments:
        position1 = lat/long pair in decimal degrees DD.dddddd
        position2 = lat/long pair in decimal degrees DD.dddddd
        
        Returns:
        distance = distance from position 1 to position 2 in meters
        
        
        Created: Joshua Vaughan - joshua.vaughan@louisiana.edu - 04/24/14
        
        Modified:
        * Forrest Montgomery -- the micropython board did not like the radians
        conversion turned into a tuple. So I separated all the lats and longs.
        '''
    
    R = 6373000        # Radius of the earth in m
    
    lat1, long1 = position1
    lat2, long2 = position2
    lat1 = math.radians(lat1)
    long1 = math.radians(long1)
    lat2 = math.radians(lat2)
    long2 = math.radians(long2)
    
    dLat = lat2 - lat1
    dLon = long2 - long1
    
    x = dLon * math.cos((lat1+lat2)/2)
    distance = math.sqrt(x**2 + dLat**2) * R
    
    return distance


# converting functions from Dr. Vaughan
def convert_longitude(long_EW):
    """ Function to convert deg m E/W longitude to DD.dddd (decimal degrees)
        Arguments:
        long_EW : tuple representing longitude
        in format of MicroGPS gps.longitude
        Returns:
        float representing longtidue in DD.dddd
        """
    
    return (long_EW[0] + long_EW[1] / 60) * (1.0 if long_EW[2] == 'E' else -1.0)


def convert_latitude(lat_NS):
    """ Function to convert deg m N/S latitude to DD.dddd (decimal degrees)
        Arguments:
        lat_NS : tuple representing latitude
        in format of MicroGPS gps.latitude
        Returns:
        float representing latitidue in DD.dddd
        """
    
    return (lat_NS[0] + lat_NS[1] / 60) * (1.0 if lat_NS[2] == 'N' else -1.0)
#Relay
def burn_parachute(burn_time):
    relay.high()
    pyb.delay(burn_time)
    relay.low()
#Pyboard IMU
def get_landing_orientation():
    landing_orientation = accel.z() # Determines the landing orientation of rover
    if landing_orientation > 0:
        my_gps = top_gps_uart
        message = 'Pyboard facing sky'
        orientation = [my_gps,message]
        return orientation
    
    elif landing landing_orientation < 0:
        my_gps = bot_gps_uart
        message = 'Pyboard facing ground'
        orientation = [my_gps,message]
        return orientation

#Xbee

#IMU
def get_yaw():
    angle = razor_imu.get_one_frame()
    yaw = angle[0]
    return yaw

#Motor Functions:
def move_forward(speed):
    #Move both wheels at same speed in same direction to move forward
    motorA.start(speed, 'CCW')
    motorB.start(speed, 'CW')

def move_backward(speed):
    #Move both wheels at same speed in same direction to move backward
    motorA.start(speed, 'CW')
    motorB.start(speed, 'CCW')

def speed_change(speed):
    motorA.set_speed(speed)

def stop():
    motorA.stop()
    motorB.stop()

def hard_stop():
    motorA.hard_stop()
    motorB.hard_stop()

def turn_left(speed):
    motorA.start(speed,'CW')
    motorB.start(speed,'CW')

def turn_right(speed):
    motorA.start(speed,'CCW')
    motorB.start(speed,'CCW')

def turn_degree(speed,direction,degree):
    motorA.stop()
    motorB.stop()
    angle = razor_imu.get_one_frame()
    old_yaw = angle[0]
    degree_to_turn = degree
    current_angle = razor_imu.start_streaming()
    if direction == 'CCW':
        new_yaw = int(old_yaw) + degree_to_turn
        motorA.start(50,'CW')
        motorB.start(50,'CW')
        if new_yaw >= 180:
            new_yaw = new_yaw-360
            continue
        continue
    elif direction == 'CW':
        new_yaw = int(old_yaw)-degree_to_turn
        motorA.start(speed,'CCW')
        motorB.start(50,'CCW')
        if new_yaw <= -180:
            new_yaw = new_yaw+360
            continue
        continue
    while True:
        current_yaw = int(current_angle[0])
        if current_yaw == new_yaw:
            motorA.stop()
            motorB.stop()
            razor_imu.get_one_frame()
            break

def one_wheel_pid(init_speed, desired_speed):
    motorA.start(speed, 'CCW')
    kp = (2*np.pi)**2
    ki = 135.0
    kd = 10.0
    pid = PID(kp, ki, kd, 0.001, 100, 0, 0.0)
    pid.compute_output(desired_speed, motorA.currentSpeed)


def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#Reading the IMU to get angle data
def IMU_read():
    angle = razor_imu.get_one_frame()
    return angle[0]

def imu_pid():


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

    initial_read = IMU_read()
    initial_angle = initial_read


    #PID loop correcting motors speed in relation
    while True:
    
        new_angle = IMU_read()
    
        if new_angle > (initial_angle + sensitivity): #The tank is drifting right
        
            angle_correction = pid_IMU.compute_output(initial_angle,new_angle)
            speed_correction= arduino_map(angle_correction, IMU_min, IMU_max, 0, 100)
            current_speedA = motorA.currentSpeed
            current_speedB = motorB.currentSpeed
            motorA.set_speed(current_speedA - speed_correction)
            motorB.set_speed(current_speedB + speed_correction)
            pyb.delay(500)
            break

        elif new_angle < (initial_angle - sensitivity): #The tank is drifting left
            angle_correction = pid_IMU.compute_output(initial_angle,new_angle)
            speed_correction= arduino_map(angle_correction, IMU_min, IMU_max, 0, 100)
            current_speedA = motorA.currentSpeed
            current_speedB = motorB.currentSpeed
            motorA.set_speed(current_speedA + speed_correction)
            motorB.set_speed(current_speedB - speed_correction)
        
            pyb.delay(500)
            break
        else:
            break


#Encoders

