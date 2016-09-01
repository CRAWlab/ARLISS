#!/usr/bin/python
###############################################################################
# Filename    : motor_class.py
# Created     : August 31, 2016
# Author      : Forrest
'''
Description   :
This is a modification of Dr Vaughan's pololu high voltage motor driver code.
The pin assignments are:
PWMA --- X8 (TIM14 CH1)
DIRA --- Y9
PWMB --- Y7 (TIM12 CH1)
DIRB --- Y8
V+   --- +12
OUTA --- motor
OUTB --- motor
GND  --- GND

                          +---+                 +---+
                          |   |                 |   |
                          |   |                 |   |
                          |   |                 |   |
                          | B +-----------------+ A |
                          |   |                 |   |
                          |   |                 |   |
                          |   |                 |   |
                          +---+                 +---+

                       * Wired with correct polarity, the pololu 47:1 motors
                         will rotate clockwise.
                       * Motor A -- positive --> OUTA
                                 -- negative --> OUTB

                       * Motor B -- positive --> OUTB
                                 -- negative --> OUTA

'''
# Modified    :
###############################################################################

from pyb import Pin, Timer
import time
import math
import utime
import pyboard_PID as pyPID

class motor(object):

    def __init__(self, PWMpin, DIRpin, timer_id, channel_id):
        self.PWMpin     = PWMpin
        self.DIRpin     = DIRpin
        self.timer_id   = timer_id
        self.channel_id = channel_id

        self.isRunning = False
        self.currentDirection = None
        self.currentSpeed = 0

    def encoder_cps(self):
        # TODO find a way to get encoder data from individual motors

        pin_a = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                        af=pyb.Pin.AF1_TIM2)

        pin_b = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                        af=pyb.Pin.AF1_TIM2)

        pin_c = pyb.Pin('X9', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                        af=pyb.Pin.AF2_TIM4)

        pin_d = pyb.Pin('X10', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                        af=pyb.Pin.AF2_TIM4)

        enc_timer = pyb.Timer(2, prescaler=0, period=65535)
        enc_timer_1 = pyb.Timer(4, prescaler=0, period=65535)

        enc_channel = enc_timer.channel(1, pyb.Timer.ENC_AB)
        enc_channel_1 = enc_timer_1.channel(2, pyb.Timer.ENC_AB)

        cts_list = []
        cts_average = None
        while self.isRunning == True:
            init_encoder_1 = enc_timer.counter()
            start_time = utime.ticks_us()
            while init_encoder_1 == enc_timer.counter():
                if init_encoder_1 != enc_timer.counter():
                    break
            init_encoder_1_later = enc_timer.counter()
            time_sum = utime.ticks_diff(start_time, utime.ticks_us())
            encoder_sum = init_encoder_1_later - init_encoder_1
            cts = (encoder_sum / time_sum) * 100
            cts_list.insert(0, cts)
            if len(cts_list) == 10:
                cts_average = sum(cts_list) / 10.0
                return cts_average
                cts_list.pop()


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


# Set up motorA
DIRA = 'Y9'
PWMA = 'X8'
TIMA = 14
CHANA = 1
motorA = motor(PWMA, DIRA, TIMA, CHANA)
motorA.start(30,'cw')
# while True:
    # motorA.encoder_cps()

kp = (2*3.14)**2
ki = 135.0
kd = 10.0
pid = pyPID.PID(kp, ki, kd, 0.001, 100, 0, 0)
while True:
    correction = pid.compute_output(1000, motorA.encoder_cps())
