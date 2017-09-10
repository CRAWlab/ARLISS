#!/bin/sh

#  correct_course_error.py
#  
#
#  Created by Joseph Fuentes on 9/10/17.
#
import pyb
import functions
from pyb import Pin
from pyb import ExtInt

# Goal of this test is to find optimal value for course_error_gain

finish_point = #enter desired finish point
course_error_gain = 0.5 # Adjust to fine tune turning
def pps_callback(line):
    '''The Adafruit GPS has a PPS pin that changes from high to low only when we are recieving data
        we will use this to our advantage by associating it with an iterrupt to change indicate we are recieving new data'''
    global new_data # Use Global to trigger update
    new_data = True # Raise flag

# Create an external interrupt on pin X8
pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, pps_callback)

finish_point = #enter desired finish point
course_error_gain = 0.5 # Adjust to fine tune turning
while True:
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        start_lat = functions.convert_latitude(my_gps.latitude) # Grabbing parameter designated by micropyGPS object
        start_lon = functions.convert_longitude(my_gps.longitude) # Grabbing parameter designated by micropyGPS object
        start_point = (start_lat, start_lon) # Creating single variable for utilization in calculations
        if landing_point[0] != 0:
            continue
        new_data = False
        break

while True:
    '''Using Cruise control to drive straight
        Duration = 10 seconds
        Desired_speed = 80% duty cycle
        '''
    functions.cruise_control(10000,80)
    
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        arb_lat = functions.convert_latitude(my_gps.latitude) # Grabbing parameter designated by micropyGPS object
        arb_lon = functions.convert_longitude(my_gps.longitude) # Grabbing parameter designated by micropyGPS object
        arbitrary_point = (arb_lat, arb_lon) # Creating single variable for utilization in calculations
        if landing_point[0] != 0:
            continue
    
        new_data = False
        break

# Now that 3 points have been established we can establish bearing and correct course
functions.bearing_difference(finish_point,start_point, arbitrary_point)

# Correcting Course Error
functions.correct_course_error(course_error_gain)
