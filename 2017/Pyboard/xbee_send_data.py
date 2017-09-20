#!/bin/sh

#  xbee_send_data.py
#  
#
#  Created by Joseph Fuentes on 9/13/17.
#
import pyb
import functions

xbee = functions.xbee

while True:

xbee.write('Hello')
pyb.delay(3000)
