##################### IMPORT LIBRARIES ####################
from motor import motor
import time
import pyb
import machine
from pyb import pin
from configurations import *
from pyboard_razor_IMU import Razor
from pyb import ExtInt
import functions
import math
from micropyGPS import MicropyGPS

################## GLOBAL FLAG FOR GPS PROCESSING ###############
new_data = False
def pps_callback(line):
    print('Updated GPS object...')
    global new_data
    new_data = True



while True:
    trigger_Pin_Value = trigger_Pin.value()
    time.sleep_ms(50)
    print(trigger_Pin_Value)
    if trigger_Pin_Value = 1:
        print('Ready for Launch!')
        red_LED.off()
        green_LED.on()
        break
        break
    else:
        red_LED.toggle()
