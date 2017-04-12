#Created by 02/16/17
# - Joseph Fuentes
# - jafuentes3594@yahoo.com
#Microcontroller: LoPy with expansion board
#Motor Driver: Pololu DVR 8838 
#Polulu 175 RPM motor w/ 12CPR encoder

#This Code was created for controlling motors w/o utilization of encoders

#Importing necessary libraries to control drivers

from machine import PWM,  Pin
import time

#################Set up Phase Pin for DVR8838##############################
PHpin= Pin('G14', mode=Pin.OUT) #LoPy Pin 14 to Phase pin of DVR8838

################Set up Enable Pin for DVR8838############################### 
EN = PWM(0, frequency=50000)  # use PWM timer 0, with a frequency of 50KHz
ENpin= EN.channel(0, pin='G12', duty_cycle=1) #LoPy Pin 12  to Enable pin of DVR8838. Default duty cycle 0.5 on channel zero

################Encoder Setup############################################
hallsensorA = Pin('G16', mode=Pin.IN, pull=Pin.PULL_UP)
hallsensorB = Pin('G17', mode=Pin.IN, pull=Pin.PULL_UP)
global count 
count=0
#def get_count():
#    if  hallsensorA()==0:
#        global count
#        count+=1
hallsensorApin= 'G16'
hallsensorBpin='G17'
class encoder(hallsensorApin, hallsensorBpin):
    global count
    count =0
    
    def _init_(self):
        global hallsensorA
        hallsensorA = Pin(self.hallsensorApin, mode=Pin.IN, pull=Pin.PULL_UP)
        global hallsensorB
        hallsensorB = Pin(self.hallsensorBpin, mode=Pin.IN, pull=Pin.PULL_UP)
        
    def count_up(self):
        if self.hallsensorA.callback(Pin.IRQ_RISING | Pin.IRQ_FALLING)==True:
            self.count+=1
            
    def count_down(self):
        if self.hallsensorB.callback(Pin.IRQ_RISING | Pin.IRQ_FALLING)==True:
            self.count-=1
    
    def get_count(self):
        current_count= self.count
        return current_count
        
