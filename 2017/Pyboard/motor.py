#! /usr/bin/env python

###############################################################################
# Requires - Pyboard pyb library
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 01/09/15
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * Forrest Montgomery -- converted into pyboard usable functions
#   * I did not delete the original code it was just commented out.
#   PWMA --- X8 (TIM14 CH1)
#   DIRA --- Y9
#   PWMB --- Y7 (TIM12 CH1)
#   DIRB --- Y8
#   V+   --- +12
#   OUTA --- motor
#   OUTB --- motor
#   GND  --- GND
#
#   * Matthew Begneaud -- added functions outside of the class that 
#                         simplify the commands necessary to provide
#                         basic movement for the rover. (See below motor class)
#                      -- more to be added...
#                            ideas: - smooth accel/decel function
# 
#    * Forrest Montgomery -- August 28, 2016 
#                         * modified to work with the pololu md07a
#                          +---+                 +---+
#                          |   |                 |   |
#                          |   |                 |   |
#                          |   |                 |   |
#                          | B +-----------------+ A |
#                          |   |                 |   |
#                          |   |                 |   |
#                          |   |                 |   |
#                          +---+                 +---+

#                       * Wired with correct polarity, the pololu 47:1 motors
#                         will rotate clockwise.
#                       * Motor A -- positive --> OUTA
#                                 -- negative --> OUTB
#
#                       * Motor B -- positive --> OUTB
#                                 -- negative --> OUTA
###############################################################################

from pyb import Pin, Timer
import time
import math
# import pyboard_PID as pid

################################# Encoder Setup ################################
pin_a = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
pin_b = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)

pin_c = pyb.Pin('X9', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)
pin_d = pyb.Pin('X10', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)

# The prescaler is ignored. When incrementing, the counter will count up-to
# and including the period value, and then reset to 0.
enc_timer = pyb.Timer(2, prescaler=0, period=65535)
enc_timer_1 = pyb.Timer(4, prescaler=0, period=65535)
# ENC_AB will increment/decrement on the rising edge of either the A channel or the B
# B channel.
enc_channel = enc_timer.channel(1, pyb.Timer.ENC_AB)
enc_channel_1 = enc_timer_1.channel(2, pyb.Timer.ENC_AB)

############################## Motor Class ##############################

class motor(object):
    """ Convenience class for controlling a motor
    Arguments
      ControlPin1 : the x01 pin
      ControlPin2 : the x02 pin
      PWMpin : the PWM pin
      DIRpin : the DIR (standby) pin on the board
    Other atributes
      isRunning : Boolean describing if motor is running or not
      speed : current speed of the motor
      direction : current direction the motor is running
                  =None if the motor is not currently moving
    """
    def __init__(self, PWMpin, DIRpin, timer_id, channel_id):
        self.PWMpin = PWMpin
        self.DIRpin = DIRpin
        self.timer_id = timer_id
        self.channel_id = channel_id

        self.isRunning = False
        self.currentDirection = None
        self.currentSpeed = 0

        # Set up the GPIO pins as output
        PWMpin = Pin(self.PWMpin)
        DIRpin = Pin(self.DIRpin, Pin.OUT_PP)

        tim = Timer(self.timer_id, freq=1000)
        ch = tim.channel(self.channel_id, Timer.PWM, pin=PWMpin)


    def start(self, speed, direction):
        """ method to start a motor
        Arguments:
          speed : speed of the motor 0-100 (as percentage of max)
          direction : CW or CCW, for clockwise or counterclockwise
        """

        PWMpin = Pin(self.PWMpin)
        DIRpin = Pin(self.DIRpin, Pin.OUT_PP)
        # DIR1 and DIR2 have to be opposite to move, toggle to change direction
        if direction in ('cw','CW','clockwise'):
            DIRpin.high()

        elif direction in ('ccw','CCW','counterclockwise'):
            DIRpin.low()

        else:
            raise ValueError('Please enter CW or CCW for direction.')

        # Start the motor
        if 0 <= speed <= 100:
            # self.PWMpin = Pin('X4')
            tim = Timer(self.timer_id, freq=1000)
            ch = tim.channel(self.channel_id, Timer.PWM, pin=PWMpin)
            ch.pulse_width_percent(speed)
            # PWM.start(self.PWMpin, speed)
        else:
            raise ValueError("Please enter speed between 0 and 100")

        # set the status attributes
        self.isRunning = True
        self.currentDirection = direction
        self.currentSpeed = speed
        while self.isRunning:
            print("\nMotorA =", enc_timer.counter())
            print("\nMotorB =", enc_timer_1.counter())

    def stop(self):
        """ redirects to a soft stop """
        self.soft_stop()


    def hard_stop(self):
        """ Method to hard stop an individual motor"""

        # self.PWMpin = Pin('X4')
        # tim = Timer(2, freq=1000)
        # ch = tim.channel(4, Timer.PWM, pin=self.PWMpin)
        ch.pulse_width_percent(0)

        # set the status attributes
        self.isRunning = False
        self.currentDirection = None
        self.currentSpeed = 0


    def soft_stop(self):
        """ Method to soft stop (coast to stop) an individual motor"""
        PWMpin = Pin(self.PWMpin)
        tim = Timer(self.timer_id, freq=1000)
        ch = tim.channel(self.channel_id, Timer.PWM, pin=PWMpin)
        for i in range(self.currentSpeed):
            ch.pulse_width_percent(self.currentSpeed-i)
            time.sleep(0.01)
        ch.pulse_width_percent(0)

        # set the status attributes
        self.isRunning = False
        self.currentDirection = None
        self.currentSpeed = 0.0


    def set_speed(self, newSpeed):
        """ Method to change the speed of the motor, direciton is unchanged
        Arugments
          newSpeed : the desired new speed 0-100 (as percentage of max)
        """

        PWMpin = Pin(self.PWMpin)
        tim = Timer(self.timer_id, freq=1000)
        ch = tim.channel(self.channel_id, Timer.PWM, pin=PWMpin)
        ch.pulse_width_percent(newSpeed)
        # PWM.set_duty_cycle(self.PWMpin, newSpeed)
        self.currentSpeed = newSpeed

################################# Pin Setup ####################################

DIRA = 'Y9'
PWMA = 'X8'
TIMA = 14
CHANA = 1

DIRB = 'Y8'
PWMB = 'Y7'
TIMB = 12
CHANB = 1

motorA = motor(PWMA, DIRA, TIMA, CHANA)
motorB = motor(PWMB, DIRB, TIMB, CHANB)


######################## 2-Wheeled Rover Functions ###############################

'''The following functions utilize the functions created in the motor class
and provide basic movement functions for the rover.

All spin-directions for motors assume left wheel is motorA and right wheel is motorB 
when looking at top of rover with front facing forward.'''

def move_forward(speed):
#Move both wheels at same speed in same direction to move forward
    motorA.start(speed, 'CCW')
    motorB.start(speed, 'CW')

def move_backward(speed):
#Move both wheels at same speed in same direction to move backward
    motorA.start(speed, 'CW')
    motorB.start(speed, 'CCW')

def speed_change(speed):
    motorA.set_speed(speed)

def stop():
    motorA.stop()
    motorB.stop()
            
def hard_stop():
    motorA.hard_stop()
    motorB.hard_stop()

def one_wheel_pid(init_speed, desired_speed):
    motorA.start(speed, 'CCW')
    kp = (2*np.pi)**2
    ki = 135.0
    kd = 10.0
    pid = PID(kp, ki, kd, 0.001, 100, 0, 0.0)
    pid.compute_output(desired_speed, motorA.currentSpeed)

