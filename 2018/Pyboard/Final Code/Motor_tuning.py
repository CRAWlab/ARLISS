###############################################################################
# main_JV_PID.py
#
# These are some minor updates to the ARLISS 2018 main algorithm. Will be
# (as of 9:45pm on 9/11) PID-control on the heading
#
# Modifications started: 09/11/18
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   *
#
# TODO:
#   *
###############################################################################

##################### IMPORT LIBRARIES ####################
from configurations import * # import configuration information first
from motor import motor
import time
import pyb
import machine
from pyb import Pin
from pyboard_razor_IMU import Razor
from pyb import ExtInt
import functions
import math
from micropyGPS import MicropyGPS


################### READ FOR FLIGHT TRIGGER WIRE #######################


while 1:
    while True:
        trigger_Pin_Value = usrSwitch()
        time.sleep_ms(50)
        print('Trigger: {}'.format(trigger_Pin_Value))

        if trigger_Pin_Value == 1:
            print('Ready for Launch!')
            red_LED.off()
            green_LED.on()
            break

        else:
            red_LED.toggle()


    time.sleep_ms(1000)

    for x in range(0,100):
        blue_LED.toggle()
        pyb.delay(10)
        green_LED.toggle()
        pyb.delay(10)

    print('moving forward')

    functions.motor_accel(100, 80)
    pyb.delay(5000)
    print('stopping')
    functions.motor_deccel(100, 80)


    functions.stop()
