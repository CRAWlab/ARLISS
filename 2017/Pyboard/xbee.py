#!/bin/sh

#  xbee.py
#  
#
#  Created by Joseph Fuentes on 9/11/17.
#
import serial
import time
from xbee import XBee

serial_port = serial.Serial('/dev/ttyUSB0', 9600)

def print_data(data):
    """
        This method is called whenever data is received
        from the associated XBee device. Its first and
        only argument is the data contained within the
        frame.
        """
    print data

xbee = XBee(serial_port, callback=print_data)

while True:
    try:
        time.sleep(0.001)
    except KeyboardInterrupt:
        break

xbee.halt()
serial_port.close()
