#Created by 02/16/17
# - Joseph Fuentes
# - jafuentes3594@yahoo.com
#Microcontroller: LoPy with expansion board
#Motor Driver: Pololu DVR 8838 
#Polulu 175 RPM motor w/ 12CPR encoder

#This Code was created for controlling motors w/o utilization of encoders

#Importing necessary libraries to control drivers
import machine
from machine import PWM, Timer
from machine import Pin
import time

#################Set up Phase Pin for DVR8838##############################
PHpin= Pin('G14', mode=Pin.OUT) #LoPy Pin 14 to Phase pin of DVR8838

################Set up Enable Pin for DVR8838############################### 
EN = PWM(0, frequency=50000)  # use PWM timer 0, with a frequency of 50KHz
ENpin= EN.channel(0, pin='G12', duty_cycle=0) #LoPy Pin 12  to Enable pin of DVR8838. Default duty cycle 0.5 on channel zero
#Duty cyle must be equal to zero while initializing motor


''''Guide to controlling Direction
Forward: PHpin(0);   ENpin.duty_cycle(value)   Where 0<value<1
Backward: PHpin(1); ENpin.duty_cycle(value)  Where 0<value<1
Stop: PHpin(); ENpin.duty_cycle(0)'''

#########################Test the motor direction############################
#Go foward for 3 seconds
PHpin(0)
ENpin.duty_cycle(0.5)
time.sleep(3)

#Stop for 3 seconds
ENpin.duty_cycle(0)
time.sleep(3)

#Go backward for 3 seconds
PHpin(1)
ENpin.duty_cycle(0.5)
time.sleep(3)

#Stop 
ENpin.duty_cycle(0)