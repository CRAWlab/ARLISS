#!/bin/sh

#  relay_control.py
#  
#
#  Created by Joseph Fuentes on 8/28/17.
#
import pyb
from pyb import Pin
relay = Pin('Y11', Pin.OUT_PP)

while True:
    relay.high()
    pyb.delay(5000)
    relay.low()
    pyb.delay(5000)
