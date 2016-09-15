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
