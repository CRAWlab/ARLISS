'''boot.py
    Intended for tank design for Arliss 2017
    Created: Joseph Fuentes 
                -jaf1036@louisiana.edu
                
    Must have the following files in directory:
        functions.py
        micropyGPS.py
        motor.py
        pyboard_razor_imu.py
    
    Peripherals:
    Adafruit Ultimate GPS
    Sparkfun Razor IMU
    XBee Pro with sparkfun breakout board
    210 RPM Gearmotor With 48 CPR Encoders
    md07a High-power Pololu Motor Drivers
    '''

################## Importing Necessary files ###############
import pyb
from pyb import UART
from pyboard_razor_IMU import Razor
from pyb import Pin
from micropyGPS import MicropyGPS
from motor import motor
import time
import math

#Light Sequence Distinguishing setup is in progress
pyb.LED(3).on()
pyb.delay(2000)
pyb.LED(3).off()
pyb.LED(4).on()

################## Peripherial Setup ######################
#IMU set up
razor_imu = Razor(1,57600) #Baud Rate is specified in spec sheets

#Pyboard IMU set up
pyb_accel = pyb.accel() #Using the on-board accelerometer to determine landing orientation


#Radio Communication
xbee = UART(2, 115200) #Baud Rate is specified in spec sheets

#GPS set up [Reliant on landing orientation]
top_gps_uart = UART(3,9600) #Baud Rate is specified in spec sheets
bot_gps_uart = UART(6,9600) #Baud Rate is specified in spec sheets

#Instantiate the micropyGPS object
top_gps = MicropyGPS(-7)
bot_gps = MicropyGPS(-7)

#Quadrature encoders:
enc_A_chan_A = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2) #Timer 2 CH1
enc_A_chan_B = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2) #Timer 2 CH2
enc_B_chan_A = pyb.Pin('Y7', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF3_TIM12) #Timer 12 CH1
enc_B_chan_B = pyb.Pin('Y8', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF3_TIM12) #Timer 12 CH2

#Motors:
DIRA = 'Y5'
PWMA = 'Y3'
TIMA = 10
CHANA = 1

DIRB = 'Y6'
PWMB = 'Y4'
TIMB = 11
CHANB = 1

#Motor Object:
motorA = motor(PWMA, DIRA, TIMA, CHANA)
motorB = motor(PWMB, DIRB, TIMB, CHANB)

# Create relay object:
relay = Pin('Y11', Pin.OUT_PP)

'''The Adafruit GPS has a PPS pin that changes from high to low only when we are recieving data 
    we will use this to our advantage by associating it with an iterrupt to change indicate we are recieving new data see functions.py for pps_callback'''

#PPS Pin to indicate signal from Top gps
top_pps = 'X8'
top_pps_pin = pyb.Pin(top_pps, Pin.IN_PP)
top_extint = top_pps_pin.irq(functions.pps_callback, Pin.IRQ_FALLING)

#PPS Pin to indicate signal from Bottom gps
bot_pps = 'X7'
bot_pps_pin = pyb.Pin(bot_pps, Pin.IN_PP)
top_extint = bot_pps_pin.irq(functions.pps_callback, Pin.IRQ_FALLING)

# Running main.py after setup is finished
pyb.usb_mode('CDC+MSC') #Setup for quick diagnostic via usb cable to REPL prompt
pyb.main('main.py') #Run the main process after boot has finished

#Indicate that setup has been completed
pyb.LED(4).off()
