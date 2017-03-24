# Created: 01/09/15
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#Modified: 02/19/17
#   -Joseph Fuentes
#   -jafuentes3594@yahoo.com
#Purpose: Use with LoPy by Pycom in Pymakr IDE
#Set up of motors and Driver 
'''  motorA----------------------motorB
    ll  DriverA              DriverB    ll
    ll                                        ll
    ll                                        ll
    ll                                        ll
    ll                                        ll
    ll                                        ll
    ll                                        ll
    ll                                        ll
    ll   DriverC             DriverD   ll
    motorC-----------------------motorD'''
#Modified: 03/20/17
# -Added Encoder structure to recognize direction

#Import Libraries used in class
import machine
from machine import PWM, Timer
from machine import Pin
import time


class motor(object):
    """ Convenience class for controlling a motor
    Arguments
      ENpin: Speed Control input used with PWM
      PHpin: Direction control input High or Low value
    Other atributes
      isRunning : Boolean describing if motor is running or not
      speed : current speed of the motor
      direction : current direction the motor is running
                  =None if the motor is not currently moving
    """
    
    def __init__motor(self, ENpin, PHpin,  timer_id, channel_id, sensorApin, sensorBpin):

        #Assign IO as Enable and Phase  output,pins
        EN = PWM(self.timer_id, frequency = 5000)
        ENpin = EN.channel(self.channel_id, ENpin, duty_cycle=0)
        PHpin = Pin(self.PHpin, Pin.OUT_PP)
        
        #Preliminary Attributes
        self.isRunning = False
        self.currentDirection = None
        self.speed = self.duty_cycle

        #Assign hall effect sensor pins frrom encoder as input pins 
        hallsensorA = Pin(self.sensorApin, mode=Pin.IN, pull=Pin.PULL_UP)
        hallsensorB = Pin(self.sensorBpin, mode=Pin.IN, pull=Pin.PULL_UP)
        
        #Assign Trigger for when sensor hits falling edge
        low_value_trigger_A = encoder_A.callback(Pin.IRQ_FALLING , rencoder) #Trigger is the falling edge; handler is rencoder event
        low_value_trigger_B = encoder_B.callback(Pin.IRQ_FALLING , rencoder)

        #Prelimnary Attributes
        count = long(pulse = 0)
        
    def rencoder(self):
        if self.hallsensorA()==0:
            self.pulse += 1
        elif self.hallsensorB()==0:
            self.pulse -= 1
        
    def stand(self):
        '''''Called When rover lands as expected'''''
        motorA.speed=0.2
        motorB.speed=0.2
        motorA.PHpin(1)
        motorB.PHpin(1)
        if motorA.count == 0:
            motorA.speed = 0
            continue
        if motorB.count == 0:
            motorB.speed =0
            continue
        motorC.speed=0.2
        motorD.speed=0.2
        motorC.PHpin(1)
        motorD.PHpin(1)
        if motorC.count & motorB.count == 0:
            motorD.speed = 0
            motorC.speed = 0
            continue

    def upside_down_stand(self,  speed, motorA, motorB, motorC, motorD):  
        '''''Called When rover lands unexpectedly'''''
        
        pass
        
    def stop(self, speed, motorA, motorB, motorC, motorD):
        pass
    def forward(self, speed, motorA, motorB, motorC, motorD):
        pass
    def backward(self, speed, motorA, motorB, motorC, motorD):
        pass
    def turn_left(self, speed, motorA, motorB, motorC, motorD):
        pass
    def turn_right(self, speed, motorA, motorB, motorC, motorD):
        pass
        
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
            DIRpin(True)
        elif direction in ('ccw','CCW','counterclockwise'):
            DIRpin(False)

        else:
            raise ValueError('Please enter CW or CCW for direction.')

        # Start the motor
        for item in speed:
            if 0 <= item <= 100:
            # self.PWMpin = Pin('X4')
             tim = Timer(self.timer_id, freq=5000)
            ch = tim.channel(self.channel_id, Timer.PWM, pin=PWMpin)
            ch.pulse_width_percent(item)
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
        self.ch.pulse_width_percent(0)

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
motorA = motor(16, 38,  0, 0, 5, 6)
motorB = motor(20, 39,  1, 1, 7, 8)
motorC = motor(22, 18,  2, 2, 11, 10)
motorD = motor(42, 17,  3, 3, 12, 13)
