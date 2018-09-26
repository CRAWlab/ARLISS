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

################## GLOBAL INTERUPPT FOR GPS PROCESSING ###############

new_data = False

def pps_callback(line):
    #print('Updated GPS Object...')
    global new_data
    new_data = True
    green_LED.toggle()
    pyb.delay(10)
    green_LED.off()
    pyb.delay(10)
    green_LED.toggle()
    pyb.delay(10)
    green_LED.off()



pps_pin = pyb.Pin.board.X5
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, pps_callback)

################### READ FOR FLIGHT TRIGGER WIRE #######################

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
absolute_start_time = pyb.millis()
log_sd = open('/sd/log.csv','w')
log_sd.write('The start time of the code is {}\n'.format(absolute_start_time))
log_sd.close()
time.sleep_ms(2500)


#will be a good idea to mark the time down
start_time = pyb.millis()

for x in range(0,100):
    blue_LED.toggle()
    pyb.delay(10)
    green_LED.toggle()
    pyb.delay(10)

blue_LED.off()
green_LED.off()

dist_from_goal = 100

print('getting first point')

while True:
    while my_gps_uart.any():
        valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))


    if valid_sentence_received:
        first_lat = functions.convert_latitude(my_gps.latitude)
        first_lon = functions.convert_longitude(my_gps.longitude)
        first_point = (first_lat, first_lon)

        if abs(first_lat) == (0.0):
            print(first_point)
            pyb.delay(10)

        else:

            print(first_point)
            log_sd = open('/sd/log.csv','a')
            log_sd.write('The first point is {},{} at {}\n'.format(first_lat,first_lon, pyb.elapsed_millis(absolute_start_time)))
            log_sd.close()
            time.sleep_ms(1000)
            break


while dist_from_goal > 3:

    print('moving forward')
    functions.motor_accel(100,100)
    time.sleep_ms(8000)
    print('stopping')
    functions.motor_deccel(100,50)

    print('getting second point')

    while True:
        while my_gps_uart.any():
            valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))

        if valid_sentence_received:
            second_lat = functions.convert_latitude(my_gps.latitude)
            second_lon = functions.convert_longitude(my_gps.longitude)
            second_point = (second_lat, second_lon)

            if abs(second_lat) == (0.0):
                print(second_point)
                pyb.delay(10)
            else:
                print(second_point)
                log_sd = open('/sd/log.csv','a')
                log_sd.write('The first point is {},{} at {}\n'.format(second_lat,second_lon, pyb.elapsed_millis(absolute_start_time)))
                log_sd.close()
                time.sleep_ms(1000)
                break




    dist_from_goal = functions.calculate_distance(finish_point, second_point)
    degree_to_turn = functions.bearing_difference(finish_point, first_point, second_point)
    log_sd = open('/sd/log.csv', 'a')
    log_sd.write('The distance from the target is {} at {}\n'.format(dist_from_goal, pyb.elapsed_millis(absolute_start_time)))
    log_sd.close()
    current_bearing = functions.calculate_bearing(first_point, second_point)
    final_bearing = functions.calculate_bearing(first_point, finish_point)

    log_sd = open('/sd/log.csv','a')
    log_sd.write('The current bearing is {} at {}\n'.format(current_bearing, pyb.elapsed_millis(absolute_start_time)))
    log_sd.write('The final bearing is {}\n'.format(final_bearing))
    log_sd.write('The course correction is {}\n'.format(degree_to_turn))
    log_sd.close()



    print('course correction')
    print(degree_to_turn)

    pyb.delay(1000)

    functions.crude_course_correct(degree_to_turn)

    print('setting first point to second')
    first_point = second_point

    orange_LED.on()
    time.sleep_ms(2000)
    orange_LED.off()
