#Created: 02/16/2017
# - Joseph Fuentes
# - jafuentes3594@yahoo.com
#Microcontroller: LoPy with pyco expansion board
#Motor Driver: L298N Duel Motor Driver. 

#This Code was created for controlling motors w/o encoders using duel motor drivers

from machine import PWM
from machine import Pin
import time

#################Pin Set up for motor functions##############################

IN1 = Pin('G14', mode=Pin.OUT) #LoPy Pin 14 to IN1 of L298N
IN2 = Pin('G15', mode=Pin.OUT)#LoPy Pin 15 to IN2 of L298N
IN3 = Pin('G17', mode=Pin.OUT) #LoPy Pin 17 to IN1 of L298N
IN4 = Pin('G23', mode=Pin.OUT)#LoPy Pin 23 to IN2 of L298N

################Set Up of timer ENA for speed and funtion for IN1,IN2 ########################## 
ENA = PWM(0, frequency=50000)  # use PWM timer 0, with a frequency of 50KHz
EN_A = ENA.channel(0, pin='G12', duty_cycle=0.5) #LoPy Pin 12  to ENA of L298N Default duty cycle 0.5 on channel 0

################Set Up of timer ENB for speed and funtion for IN3,IN4 ########################## 
ENB = PWM(1, frequency=50000)  # use PWM timer 1, with a frequency of 50KHz
EN_B = ENB.channel(1, pin='G13', duty_cycle=0.5) #LoPy Pin 13  to ENA of L298N Default duty cycle 0.5 on channel 1

#######################Entering Speed of Motor####################
EN_A_speed= 0.5 #Edit this Value for Speed adjustment 0<=speed<1
EN_A.duty_cycle(EN_A_speed) # change the duty cycle to 30%
EN_B_speed= 0.2 #Edit this Value for Speed adjustment 0<=speed<1
EN_B.duty_cycle(EN_B_speed) # change the duty cycle to 30%

########################Teting 'IN' Sequence#####################
while (True):
    IN1(True)
    print('IN 1 Active')
    time.sleep(2)
    IN1(False)
    print('IN 1 Inactive')
    time.sleep(2)
    IN2(True)
    print('IN 2 Active')
    time.sleep(2)
    IN2(False)
    print('IN 2 Inactive')
    time.sleep(2)
    IN3(True)
    print('IN 3 Active')
    time.sleep(2)
    IN3(False)
    print('IN 3 Inactive')
    time.sleep(2)
    IN4(True)
    print('IN 4 Active')
    time.sleep(2)
    IN4(False)
    print('IN 4 Inactive')
    time.sleep(2)


