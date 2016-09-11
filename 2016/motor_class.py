#!/usr/bin/python
###############################################################################
# Filename    : motor_class.py
# Created     : August 31, 2016
# Author      : Forrest
'''
Description   :
This is a modification of Dr Vaughan's pololu high voltage motor driver code.

                          +---+                 +---+
                          |   |                 |   |
                          |   |                 |   |
                          |   |                 |   |
                          | A +-----|----|------+ B |
                          |   |     |    |      |   |
                          |   |     |    |      |   |
                          |   |                 |   |
                          +---+                 +---+

'''
# Modified    :
###############################################################################

import pyb
from pyb import Pin, Timer
import time
import math
import utime
# import pyboard_PID as pyPID

class motor(object):

    def __init__(self, PWMpin, DIRpin, timer_id, channel_id):
        self.PWMpin     = PWMpin
        self.DIRpin     = DIRpin
        self.timer_id   = timer_id
        self.channel_id = channel_id

        self.isRunning = False
        self.currentDirection = None
        self.currentSpeed = 0

    def start(self, speed, direction):
        PWM_py_pin = Pin(self.PWMpin)
        DIR_py_pin = Pin(self.DIRpin, Pin.OUT_PP)

        tim = Timer(self.timer_id, freq=1000)
        ch = tim.channel(self.channel_id, Timer.PWM, pin=PWM_py_pin)

        if direction in ('cw', 'CW', 'clockwise'):
            DIR_py_pin.high()

        elif direction in ('ccw', 'CCW', 'counterclockwise'):
            DIR_py_pin.low()

        else:
            raise ValueError('Please enter CW or CCW')

        if 0 <= speed <= 100:
            ch.pulse_width_percent(speed)

        else:
            raise ValueError("Please enter a speed between 0 and 100")

        self.isRunning        = True
        self.currentDirection = direction
        self.currentSpeed     = speed

    def stop(self):
        PWM_py_pin = Pin(self.PWMpin)
        DIR_py_pin = Pin(self.DIRpin, Pin.OUT_PP)

        tim = Timer(self.timer_id, freq=1000)
        ch = tim.channel(self.channel_id, Timer.PWM, pin=PWM_py_pin)
        for i in range(self.currentSpeed):
            ch.pulse_width_percent(self.currentSpeed-i)
            time.sleep(0.01)
        ch.pulse_width_percent(0)
        self.isRunning = False
        self.currentDirection = None
        self.currentSpeed = 0.0

    def change_speed(self,newspeed):
        PWM_py_pin = Pin(self.PWMpin)
        DIR_py_pin = Pin(self.DIRpin, Pin.OUT_PP)

        tim = Timer(self.timer_id, freq=1000)
        ch = tim.channel(self.channel_id, Timer.PWM, pin=PWM_py_pin)
        ch.pulse_width_percent(newspeed)
        self.currentSpeed = newspeed
        self.isRunning = True

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
