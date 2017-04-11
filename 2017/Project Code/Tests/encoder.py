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
def one_revolution(self):
    
    while True:
        if  int(hallsensorA())==0:
            global count
            count+=1
            if (count)==3000:
                ENpin.duty_cycle(0)
                time.sleep(3)
                break

one_revolution(True)
#def count_up(self):
#        while True: 
#            self.count += 1
#        return
#
#
#def  rotate(self, angle, direction , speed):
#        low_value_trigger_A = hallsensorA.callback(Pin.IRQ_FALLING, count_up)
#        
#        desired_angle = self.angle
#        PHpin(self.direction)
#        ENpin.duty_cycle(self.speed)
#        desired_count = (5)*desired_angle
#        startcount = self.count
#        if startcount == desired_count:
#            ENpin.duty_cycle(0)
#        
#rotate(360, 0, 0.5)
#   
''''Guide to controlling Direction
Forward: PHpin(0);   ENpin.duty_cycle(value)   Where 0<value<1
Backward: PHpin(1); ENpin.duty_cycle(value)  Where 0<value<1
Stop: PHpin(); ENpin.duty_cycle(0)'''

#########################Test the motor direction############################
#Go foward for 3 seconds
    
