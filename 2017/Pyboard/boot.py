# boot.py -- runs on boot-up
# Let's you choose which script to run.
# > To run 'datalogger.py':
#       * press reset and do nothing else
# > To run 'cardreader.py':
#       * press reset
#       * press user switch and hold until orange LED goes out

import pyb
from pyb import UART
from pyboard_razor_IMU import Razor
from pyb import ExtInt
from pyb import Pin
from micropyGPS import MicropyGPS
from motor import motor
import time
import math
pyb.LED(3).on()                 # indicate we are waiting for switch press
pyb.delay(2000)                 # wait for user to maybe press the switch
switch_value = pyb.Switch()()   # sample the switch at end of delay
pyb.LED(3).off()                # indicate that we finished waiting for the switch

pyb.LED(4).on()                 # indicate that we are selecting the mode

################## Peripherial Setup ######################
#IMU set up
razor_imu = Razor(1,57600)

#Pyboard IMU set up
pyb_accel = pyb.accel()


#Radio Communication
xbee = UART(2, 115200)

#GPS set up [Reliant on landing orientation]
top_gps_uart = UART(3,9600)
bot_gps_uart = UART(6,9600)

#Instantiate the micropyGPS object
top_gps = MicropyGPS(-7)
bot_gps = MicropyGPS(-7)

#Quadrature encoders:
enc_A_chan_A = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_A_chan_B = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_B_chan_A = pyb.Pin('Y7', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)
enc_B_chan_B = pyb.Pin('Y8', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)

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

if switch_value:
    pyb.usb_mode('CDC+MSC')
    pyb.main('cardreader.py')           # if switch was pressed, run this
else:
    pyb.usb_mode('CDC+MSC')
    pyb.main('main.py')           # if switch wasn't pressed, run this

pyb.LED(4).off()                # indicate that we finished selecting the mode
