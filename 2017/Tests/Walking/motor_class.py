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

from machine import PWM, Pin
import time


class motor:
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
    
    def __init__(self, ENpin, PHpin,  timer_id, channel_id):
        self.timer_id=timer_id
        self.channel_id=channel_id
        #Assign IO as Enable and Phase  output,pins
        self.EN = PWM(self.timer_id, frequency = 5000)
        self.ENpin = self.EN.channel(self.channel_id, ENpin, duty_cycle=0)
        self.PHpin = Pin(PHpin, Pin.OUT_PP)
        
        #Preliminary Attributes
        self.isRunning = False
        self.currentDirection = None
        self.current_speed= 0
    def direction(self, direction):
        
        self.current_direction=direction
        
        if direction=='CCW, ccw ':
            self.PHpin(0)
        elif direction =='CW, cw':
            self.PHpin(1)
            
    def change_speed(self, newspeed):
        self.ENpin.duty_cycle(newspeed)
        self.current_speed=newspeed   

    def stop(self):
        """ Method to hard stop an individual motor"""
        self.ENpin.duty_cycle(0)
        # set the status attributes
        self.is_Running = False
        self.current_direction = None
        self.current_speed = 0

    def soft_stop(self):
        """ Method to soft stop (coast to stop) an individual motor"""
        for i in range(self.current_speed):
            self.ENpin.duty_cycle(self.current_speed-i)
            time.sleep(0.01)
        self.ENpin.duty_cycle(0)

        # set the status attributes
        self.isRunning = False
        self.current_direction = None
        self.current_speed = 0.0
    def attributes(self):
        ENpin=self.ENpin
        PHpin=self.PHpin
        timer_id=self.timer_id
        channel_id=self.channel_id
        current_direction=self.current_direction
        current_speed=self.current_speed
        print('ENpin is:' , ENpin)
        print('PHpin is:',  PHpin)
        print( 'Motor on timer:', timer_id )
        print('Motor on channel:', channel_id)
        print('Current direction:',  current_direction)
        print('The current speed is:', current_speed)
        
        
class encoder:
    
    def __init__(self, hallsensorApin, hallsensorBpin):
        self.count = 0

        
        self.hallsensorA = Pin(hallsensorApin, mode=Pin.IN, pull=Pin.PULL_UP)
        self.hallsensorB = Pin(self.hallsensorBpin, mode=Pin.IN, pull=Pin.PULL_UP)
        self.count_max=1800 #This is the amount of pulses per revolution 
        self.count_min=0
        
    
    def rencoder(self):
        if self.hallsensorA()==0:
            self.count += 1
            if self.count==self.count_max:
                self.count=0
        elif self.hallsensorB()==0:
            self.count -= 1
            if self.count==self.count_min:
                self.count=1800
    def set_count(self):
        self.count= 0 
        return self.count
        
    def get_count(self):
        return self.count
        
    def trigger(self, direction):   
        self.direction = direction
        if self.direction =='CCW':
            self.hallsensorA.callback(Pin.IRQ_FALLING,  self.rencoder)
        if self.direction =='CW':
            self.hallsensorB.callback(Pin.IRQ_FALLING, self.rencoder)
            

        
