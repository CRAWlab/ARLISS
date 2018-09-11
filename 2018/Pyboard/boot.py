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
'''from pyb import UART
from pyboard_razor_IMU import Razor
from pyb import Pin
from micropyGPS import MicropyGPS
from motor import motor
import time
import math
'''
#Light Sequence Distinguishing setup is in progress
pyb.LED(3).on()
pyb.delay(2000)
pyb.LED(3).off()
pyb.LED(4).on()


# Running main.py after setup is finished
pyb.usb_mode('CDC+MSC') #Setup for quick diagnostic via usb cable to REPL prompt
pyb.main('main.py') #Run the main process after boot has finished

#Indicate that setup has been completed
pyb.LED(4).off()
