#!/bin/sh

#  altitude_check.py
#  
#
#  Created by Joseph Fuentes on 9/13/17.
#
import pyb
import functions

current_altitude = functions.monitor_descent()
print(current_altitude)
with open('/sd/log.txt', 'a') as log:
    log.write('I have landed with altitude: {}\n'.format(current_altitude))
