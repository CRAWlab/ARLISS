#! /usr/bin/env python

###############################################################################
# py_motor_TB6612FNG.py
#
# Basic test of motor control using the SparkFun TB6612FNG breakout board
#
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
#   PWMA --- X4
#   AIN1 --- X2
#   AIN2 --- X3
#   STBY --- X1
#   VM   --- +12
#   VCC  --- +3.3
#   GND  --- GND
#
#   * Matthew Begneaud -- added functions outside of the class that 
#                         simplify the commands necessary to provide
#                         basic movement for the rover. (See below motor class)
#                      -- more to be added...
#                            ideas: - smooth accel/decel function
#                                   - 
###############################################################################

from pyb import Pin, Timer
# import Adafruit_BBIO.PWM as PWM
import time
import math



############################## Motor Class ##############################

class motor(object):
    """ Convenience class for controlling a motor
    Arguments
      ControlPin1 : the x01 pin
      ControlPin2 : the x02 pin
      PWMpin : the PWM pin
      STBYpin : the STBY (standby) pin on the board
    Other atributes
      isRunning : Boolean describing if motor is running or not
      speed : current speed of the motor
      direction : current direction the motor is running
                  =None if the motor is not currently moving
    """
    def __init__(self, ControlPin1, ControlPin2, PWMpin, STBYpin):
        self.ControlPin1 = ControlPin1
        self.ControlPin2 = ControlPin2
        self.PWMpin = PWMpin
        self.STBYpin = STBYpin
        self.isRunning = False
        self.currentDirection = None
        self.currentSpeed = 0

        # Set up the GPIO pins as output
        self.STBYpin = Pin(self.STBYpin, Pin.OUT_PP)
        # GPIO.setup(self.STBYpin, GPIO.OUT)
        self.ControlPin1 = Pin(self.ControlPin1, Pin.OUT_PP)
        # GPIO.setup(self.ControlPin1, GPIO.OUT)
        self.ControlPin2 = Pin(self.ControlPin2, Pin.OUT_PP)
        # GPIO.setup(self.ControlPin2, GPIO.OUT)
        self.PWMpin = Pin(self.PWMpin)
        tim = Timer(2, freq=1000)
        ch = tim.channel(4, Timer.PWM, pin=self.PWMpin)


    def start(self, speed, direction):
        """ method to start a motor
        Arguments:
          speed : speed of the motor 0-100 (as percentage of max)
          direction : CW or CCW, for clockwise or counterclockwise
        """

        # Standby pin should go high to enable motion
        self.STBYpin.high()
        # STBYpin.high()
        # GPIO.output(self.STBYpin, GPIO.HIGH)

        # x01 and x02 have to be opposite to move, toggle to change direction
        if direction in ('cw','CW','clockwise'):
            self.ControlPin1.low()
            # GPIO.output(self.ControlPin1, GPIO.LOW)
            self.ControlPin2.high()
            # GPIO.output(self.ControlPin2, GPIO.HIGH)
        elif direction in ('ccw','CCW','counterclockwise'):
            self.ControlPin1.high()
            # GPIO.output(self.ControlPin1, GPIO.HIGH)
            self.ControlPin2.low()
            # GPIO.output(self.ControlPin2, GPIO.LOW)
        else:
            raise ValueError('Please enter CW or CCW for direction.')

        # Start the motor
        # PWM.start(channel, duty, freq=2000, polarity=0)
        if 0 <= speed <= 100:
            self.PWMpin = Pin('X4')
            tim = Timer(2, freq=1000)
            ch = tim.channel(4, Timer.PWM, pin=self.PWMpin)
            ch.pulse_width_percent(speed)
            # PWM.start(self.PWMpin, speed)
        else:
            raise ValueError("Please enter speed between 0 and 100, \
                              representing a percentage of the maximum \
                              motor speed.")

        # set the status attributes
        self.isRunning = True
        self.currentDirection = direction
        self.currentSpeed = speed

    def stop(self):
        """ redirects to a soft stop """
        self.soft_stop()


    def hard_stop(self):
        """ Method to hard stop an individual motor"""

        self.PWMpin = Pin('X4')
        tim = Timer(2, freq=1000)
        ch = tim.channel(4, Timer.PWM, pin=self.PWMpin)
        ch.pulse_width_percent(0)
        # PWM.set_duty_cycle(self.PWMpin, 0.0)

        # set the status attributes
        self.isRunning = True
        self.currentDirection = None
        self.currentSpeed = 0


    def soft_stop(self):
        """ Method to soft stop (coast to stop) an individual motor"""

        # Make both control pins low
        self.ControlPin1.low()
        # GPIO.output(self.ControlPin1, GPIO.LOW)
        self.ControlPin2.low()
        # GPIO.output(self.ControlPin2, GPIO.LOW)
        self.PWMpin = Pin('X4')
        tim = Timer(2, freq=1000)
        ch = tim.channel(4, Timer.PWM, pin=self.PWMpin)
        ch.pulse_width_percent(0)
        self.STBYpin.low()
        # GPIO.output(self.STBYpin, GPIO.LOW)

        # set the status attributes
        self.isRunning = True
        self.currentDirection = None
        self.currentSpeed = 0.0


    def set_speed(self, newSpeed):
        """ Method to change the speed of the motor, direciton is unchanged
        Arugments
          newSpeed : the desired new speed 0-100 (as percentage of max)
        """

        self.PWMpin = Pin('X4')
        tim = Timer(2, freq=1000)
        ch = tim.channel(4, Timer.PWM, pin=self.PWMpin)
        ch.pulse_width_percent(newSpeed)
        # PWM.set_duty_cycle(self.PWMpin, newSpeed)
        self.currentSpeed = newSpeed




################################# Pin Setup #####################################

STBY = 'X1'       # STBY pin on the breakout, must go low to enable motion
A01 = 'X2'        # A01 pin on board, controls direction along with A02
A02 = 'X3'        # A02 pin on board, controls direction along with A01
PWMA = 'X4'       # PWMA pin on board, controls the speed of Motor A
B01 = 'X11'        # B01 pin on board, controls directions along with B02 
B02 = 'X12'        # B02 pin on board, controls directions along with B02
PWMB = 'X10'      # PWMB pin on board, controls the speed of Motor B (must be 
                  # this pin to be on same timer and channel as PWMA)

motorA = motor(A01, A02, PWMA, STBY)
motorB = motor(B01, B02, PWMB, STBY)


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

def angle_to_motor_turn(angle, direction):
    """
    This function takes the angle and issues a timed command to one motor to
    rotate the rover to a specific angle. Angle must be in degrees; Direction is
    right or left: Right = 1     Left = -1
    ** If you want to turn right the left wheel must turn.**
    """
    wheel_distance = 10.25
    radius = 2.4375
    angle = math.radians(angle)
    number_of_revolutions = (wheel_distance * angle) / (2 * math.pi * radius)
    # At speed = 30% how long does it take for the wheel to make one rev?
    one_rev_time = 1
    # work with the gain to make the rover turn correctly
    gain = 0.33
    time_to_rotate = (number_of_revolutions * one_rev_time) * gain
    print(time_to_rotate)
    # This is a right turn.
    if direction == 1:
        motorA.start(30, 'CCW')
        print("time: {}".format(time_to_rotate))
        time.sleep(abs(time_to_rotate))
        motorA.stop()
    # This is a left turn
    else:
        motorB.start(30, 'CW')
        print("time: {}".format(time_to_rotate))
        time.sleep(abs(time_to_rotate))
        motorB.stop()

def stationary_turn(speed, direction):
#Move one wheel to turn left or right (to move both in opposite direction for pure rotation, uncomment the 2 lines of code)
#Direction is left or right
    if direction == 1:
        motorA.start(speed, 'CCW')
        #motorB.start(speed, 'CCW')
    elif direction == -1:
        #motorA.start(speed, 'CW')
        motorB.start(speed, 'CW')
    else:
        print('Error: please enter direction as "left" or "right"')

def turn(speed, direction):
#Moves both wheels  at varying speed in same direction so that the rover turns either left or right while moving forward
#Intensity of turn can be modified by changing the intensity of the modifier. 

    intensity_modifier = 0.8 # can be changed; may eventually become an input argument following
                             # further testing and mapping of arc length vs intensity

    if intensity_modifier < 1:
        if direction == 'right':
            motorA.start(speed, CCW)
            motorB.start(speed*intensity_modifier, CW)
        elif direction == 'left':
            motorA.start(speed*intensity_modifier, CCW)
            motorB.start(speed, CW)
        else:
            print('Error: please enter direction as "left" or "right"')
    else:
            raise ValueError('Please change intensity_modifier to a value < 1')

def stop():
    motorA.stop()
    motorB.stop()
            
def hard_stop():
    motorA.hard_stop()
    motorB.hard_stop()


################################## Motor Test ####################################

# if __name__ == '__main__':
#     Demonstrates the use of this class

#     Set up the pins - These are mutable, but *don't* change them
#    STBY = 'X1'       # STBY pin on the breakout, must go low to enable motion
#    A01 = 'X2'        # A01 pin on board, controls direction along with A02
#    A02 = 'X3'        # A02 pin on board, controls direction along with A01
#    PWMA = 'X4'      # PWMA pin on board, controls the speed of Motor A

#     Create the motorA instance of the class
#    motorA = motor(A01, A02, PWMA, STBY)

#     We can check if it's running
#    if motorA.isRunning:
#        print('Motor A is currently running.')
#    else:
#        print('Motor A is not currently running.')


#     Now, let's run it for 2s, off for 2s, on for 2s... for 5 times
#     let's print that it's running each time too, using our inRunning attribute
#    for index in range(2):
#        motorA.start(100,'CCW')

#          We can check if it's running
#        if motorA.isRunning:
#            print('Motor A is spinning {} at {}% of maximum speed.'.format(motorA.currentDirection, motorA.currentSpeed))

#        time.sleep(2)
#        print('This is a hard stop. It effectively brakes.\n')
#        motorA.hard_stop()
#        time.sleep(2)

#        motorA.start(100,'CW')

#         We can check if it's running
#        if motorA.isRunning:
#            print('Motor A is spinning {} at {}% of maximum speed.'.format(motorA.currentDirection, motorA.currentSpeed))

#        time.sleep(2)

#        print('This is a soft stop. It coasts to stop.\n')
#        motorA.stop()
#        time.sleep(2)


#     Let's vary the speed - we'll get fancy and use a sinusoidal variance
#    motorA.start(75,'CW')
#    lastTime = time.time()
#    startTime = time.time()

#    while time.time()-startTime < 30:
#        speed = 75 + 25 * math.sin(0.25 * 2 * math.pi * (time.time()-startTime))
#        motorA.set_speed(speed)

#        if time.time() - lastTime > 1:
#            print('Motor A is spinning {} at {:>6.2f}% of maximum speed.'.format(motorA.currentDirection, motorA.currentSpeed))
#            lastTime = time.time()

#        time.sleep(0.01)

#    motorA.stop()

#     We can still access pins and "raw" Adafruit library functions if needed
#    motorA.STBYpin.value()
#     GPIO.output(motorA.STBYpin, GPIO.LOW)

#Wheel test for transformable wheels
# pyb.delay(5000)
# while True:
#     move_forward(40)
#     pyb.delay(10000)
#     stop()
#     pyb.delay(15000)
