#!/bin/sh

#  cruise_control_tweaking.py
#  
#
#  Created by Joseph Fuentes on 9/10/17.
#
import pyb
from pyb import Pin
from pyb import Timer
from motor import motor
from pyboard_PID import PID

########### Quadrature encoder set up ##############
# Pin(Board Pin, Alternate function, Pull up resistor enabled, changing Alternate function to use for encoder channel)
enc_A_chan_A = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2) #Timer 2 CH1
enc_A_chan_B = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2) #Timer 2 CH2
enc_B_chan_A = pyb.Pin('Y1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF3_TIM8) #Timer 8 CH1
enc_B_chan_B = pyb.Pin('Y2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF3_TIM8) #Timer 8 CH2

# Putting the Pyboard Channels into encoder mode
enc_timer_A = pyb.Timer(2, prescaler=0, period = 65535)
enc_timer_B = pyb.Timer(8, prescaler=0, period = 65535)
encoder_A= enc_timer_A.channel(1, pyb.Timer.ENC_AB)
encoder_B = enc_timer_B.channel(2, pyb.Timer.ENC_AB)

################# Motor Set up #####################
#Motors:
DIRA = 'Y5'
PWMA = 'Y3'
TIMA = 10
CHANA = 1

DIRB = 'Y6'
PWMB = 'Y4'
TIMB = 4
CHANB = 4

#Motor Object:
motorA = motor(PWMA, DIRA, TIMA, CHANA)
motorB = motor(PWMB, DIRB, TIMB, CHANB)

def calculate_ang_velocity(encoder_timer):
    '''Uses data from associated encoder calculates difference in count over a sample time to output the angular velocity of the motor.
        Created By: Joseph Fuentes - jaf1036@louisiana.edu 08/09/2017'''
    
    cpr = 2256 # Counts per revolution based on specifications and gear ratio of motor
    start = pyb.millis() # Begin sample time
    initial_count = encoder_timer.counter() # Grabbing initial count of encoder
    sample_time = pyb.delay(10) # Delay small but significant for sample time
    '''Since the  maximum rps is 3.5 with 2249 cpr that equates to 7.87 counts per ms maximum allowing for 50ms to be sufficient'''
    
    # velocity in counts per second
    ang_velocity_cps = 1000*(abs(encoder_timer.counter()-initial_count))/(pyb.elapsed_millis(start))
    
    #velocity in revolutions per second
    ang_velocity_revps = ang_velocity_cps/cpr
    
    return ang_velocity_revps

def cruise_control(duration, speed):
    '''Use encoder data to calulate actual angular velocity of motors based on input PWM. Then compares this actual angular velocity to a desired velocity and maps it to the appropriate PWM correction. Once the run time of the cruise control algorithm exceeds the desired duration of the function then it will stop the motors and end proceedure.
        Like a normal cruise control it is intended for use only while moving forward.....
        Created By: Joseph Fuentes - jaf1036@louisiana.edu 08/09/2017'''
    
    # Maximum possible revolution per second of motor = 3.5 [210rpm]
    
    run_time = pyb.millis() # Start timer for the run time of the cruise control
    # Duration: Desired duration of the cruise control [ms]
    # Speed: Desired speed, value of PWM
    
    desired_velocity = arduino_map(speed, 0, 100, 0, 3) #Mapping desired PWM to angular velocity
    
    # PID values [NEEDS TUNING!!!!!]
    kp_motor = 0.07
    ki_motor = 0.05
    kd_motor = 0.03
    
    # Creating PID object for each encoder
    A_pid = PID(kp_motor, ki_motor, kd_motor, 100000, 3, -3)
    B_pid = PID(kp_motor, ki_motor, kd_motor, 100000, 3, -3)
    
    
    #Starting motors at the desired PWM
    move_forward(speed)
    while True:
        #Calculating actual angular velocity
        A_velocity = calculate_ang_velocity(enc_timer_A)
        B_velocity = calculate_ang_velocity(enc_timer_B)
        
        # Computing error of desired vs. actual
        
        A_correction= A_pid.compute_output(desired_velocity, A_velocity)
        
        #Mapping correction to the appropriate PWM
        conversion_A = arduino_map(A_correction, 0, 3, 0, 100)
        motorA.set_speed(conversion_A)
        
        # Computing error of desired vs. actual
        B_correction= B_pid.compute_output(desired_velocity, B_velocity)
        
        #Mapping correction to the appropriate PWM
        conversion_B = arduino_map(abs(B_correction), 0, 3, 0, 100)
        motorB.set_speed(conversion_B)
        
        if pyb.elapsed_millis(run_time) > duration: # Condition to end function
            motorA.set_speed(0)
            motorB.set_speed(0)
            break

