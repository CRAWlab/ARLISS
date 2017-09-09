#!/bin/sh

#  pid_encoder_motors.sh
#  
#
#  Created by Joseph Fuentes on 8/23/17.
#
#################### Import Libraries#######################
import pyb
from pyb import Pin
from motor import motor
from pyboard_PID import PID
import time
import math

#Quadrature encoders:
enc_A_chan_A = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_A_chan_B = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_B_chan_A = pyb.Pin('Y7', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)
enc_B_chan_B = pyb.Pin('Y8', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)

#Spec encoder counts per revolution
cpr =2249

#Putting the Pyboard Channels into encoder mode
enc_timer_left = pyb.Timer(2, prescaler=0, period = 65535)
enc_timer_right = pyb.Timer(4, prescaler=0, period = 65535)
left_encoder = enc_timer_left.channel(1, pyb.Timer.ENC_AB)
right_encoder = enc_timer_right.channel(2, pyb.Timer.ENC_AB)

#Motor Object:
motor_left = motor(PWMA, DIRA, TIMA, CHANA)
motor_right = motor(PWMB, DIRB, TIMB, CHANB)

kp_motor = 1
ki_motor = 0
kd_motor = 0


#Creating PID object for both tracks
left_pid = PID(kp_motor, ki_motor, kd_motor, 100000, 3.5, -3.5)
right_pid = PID(kp_motor, ki_motor, kd_motor, 100000, 3.5, -3.5)

#starting count
initial_count= 0

def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def calculate_ang_velocity(self, encoder_timer):
    cpr =2249
    start = pyb.millis()
    initial_count = self.encoder_timer.count()
    sample_time = pyb.delay(500)
    #velocity in counts per second
    ang_velocity_cps = 1000*abs(self.encoder_timer.count()-initial_count)/(pyb.elapsed_millis(start))
    #velocity in revolutions per second
    ang_velocity_revps = ang_velocity_cps/cpr
    #velocity in radians per second
    ang_velocity_radps = ang_velocity_revps*6.28
    
    #Creating the angular velocity as Tuple


    return ang_velocity_revps

desired_velocity = 2
while True:
    motor_left.start(50,'cw')
    motor_right.start(50,'ccw')
    left_velocity = calculate_ang_velocity(enc_timer_left)
    right_velocity = calculate_ang_velocity(enc_timer_right)
    
    left_correction_= left_pid.compute_output(desired_velocity, left_velocity)
    conversion_left = arduino_map(left_correction, 0, 3.5, 0, 100)
    motor_left.set_speed(conversion_left)

    right_correction_= right_pid.compute_output(desired_velocity, right_velocity)
    conversion_right = arduino_map(right_correction, 0, 3.5, 0, 100)
    motor_right.set_speed(conversion_right)
    time.sleep(3)





