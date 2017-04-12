#Created by Joseph Fuentes (02/16/2017). 
#Microcontroller: LoPy. 
#Motor Driver: L298N Duel Motor Driver. 
#Polulu 175 RPM motor w/ 12CPR encoder
''''Encoder: 1800 countable events per revolution
    360 degrees per rev
    resulting in 5 pulses/ 1 degree'''
#This Code was created for controlling motors w encoders

import machine
from machine import PWM, Timer
from machine import Pin
import time

#################Pin Set up for motor functions##############################
IN1 = Pin('G14', mode=Pin.OUT) #LoPy Pin 14 to IN1 of L298N
IN2 = Pin('G15', mode=Pin.OUT)#LoPy Pin 15 to IN2 of L298N
#IN3 = Pin('G17', mode=Pin.OUT) #LoPy Pin 17 to IN1 of L298N
#IN4 = Pin('G23', mode=Pin.OUT)#LoPy Pin 23 to IN2 of L298N

################Set Up of timer ENA for speed and funtion for IN1,IN2 ########################## 
ENA = PWM(0, frequency=50000)  # use PWM timer 0, with a frequency of 50KHz
EN_A = ENA.channel(0, pin='G12', duty_cycle=0.5) #LoPy Pin 12  to ENA of L298N Default duty cycle 0.5

################Set Up of timer ENB for speed and funtion for IN3,IN4 ########################## 
#ENB = PWM(1, frequency=50000)  # use PWM timer 0, with a frequency of 50KHz
#EN_B = ENB.channel(1, pin='G13', duty_cycle=0.5) #LoPy Pin 12  to ENA of L298N Default duty cycle 0.5

#######################Entering Speed of Motor####################
speed= [0.5,0.5]#Edit this Value for Speed adjustment 0<=speed<1
EN_A.duty_cycle(speed[0]) # change the duty cycle to 30%
#EN_B_speed= 0.2 #Edit this Value for Speed adjustment 0<=speed<1
#EN_B.duty_cycle(speed[1]) # change the duty cycle to 30%

######################Setting up encoder pinout ###############################
encoder_A = Pin('G5', mode=Pin.IN, pull=Pin.PULL_UP)
encoder_B = Pin('G6', mode=Pin.IN, pull=Pin.PULL_UP)

###########################Defining encoder event #############################
pulses = long(pulse = 0)
def rencoder(self):
    if encoder_A()==0:
        self.pulse += 1
    elif encoder_B()==0:
        self.pulse -= 1
        
#############################Defining Triggers as #############################
low_value_trigger_A = encoder_A.callback(Pin.IRQ_FALLING , rencoder) #Trigger is the falling edge; handler is rencoder event
low_value_trigger_B = encoder_B.callback(Pin.IRQ_FALLING , rencoder)

########################Teting 'IN' Sequence#####################
while (True):
    IN1(True)
    if encoder_A()==0:
        low_value_trigger_A
        global pulses
        if pulses==100:
            IN1(False)   
            print (pulses)
            del low_value_trigger
            del value_A
            del pulses
            time.sleep(2) 
            continue
    IN2(True)
    if encoder_B()==0:
        low_value_trigger_B
        global pulses
        if abs(pulses)==100:
            IN1(False)   
            print (pulses)
            del low_value_trigger
            del value_A
            del pulses
            time.sleep(1) 
            continue

            
#        except MemoryError : 
#            print('Memory full')
#            break


#    time.sleep(0.01)
##    print(pulses)
##    time.sleep(1)
#    IN1(False)
#    time.sleep(1)
#    
#    number=pulse.counter()
#    print(number)
    
    
#    if pulses==1200:
#            print('quarter turn complete')
#            print(pulses)
#            IN1(False)
#            time.sleep(4)
#            break

