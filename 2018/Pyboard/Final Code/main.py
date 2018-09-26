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

time.sleep_ms(2500)
green_LED.off()


#save this to get an idea on how long the code was running
absolute_start_time = pyb.millis()

# Write the start time to the SD card
log_sd = open('/sd/log.csv','w')
log_sd.write('The start time of the code is {}\n'.format(absolute_start_time))
log_sd.close()

#will be a good idea to mark the time down
start_time = pyb.millis()

###### TIME DELAY ON LOOKING FOR GPS COORDINATES ##############
while pyb.elapsed_millis(start_time) < 1000*60*60:
    time.sleep_ms(1000)
    blue_LED.toggle()

blue_LED.off()

#this sequence test first if there are gps coordinates to be read,
#then tests the change in altitude to see if it has landed,
#then tests to see if there is any change in the accelerometer values
#the accelerometer essentially tests to make sure it is not bouncing
#from the impact
#there is a time limit condition associated with it which, if exceeded,
#will trigger the burn wire. the time limit is set for 90 minutes

######################### WAITING FOR NEW DATA ###############
start_time = pyb.millis()

while pyb.elapsed_millis(start_time) < 60*1000*90 and new_data == 0:
    print(new_data)
    time.sleep_ms(500)

    #toggles morse code for S0S if the gps is not reading new data
    while new_data == 0:
        for x in range(0,6):
            orange_LED.toggle()
            time.sleep_ms(250)
        for x in range(0,6):
            orange_LED.toggle()
            time.sleep_ms(500)


orange_LED.off()
log_sd = open('/sd/log.csv', 'a')
log_sd.write('The first GPS instance is {}\n'.format(pyb.elapsed_millis(absolute_start_time)))
log_sd.close()

###################### CALCULATE ALTITUDE CHANGE ######################
while pyb.elapsed_millis(start_time) < 60*1000*90 and change_in_altitude < -5:

    while my_gps_uart.any():
        my_gps.update(chr(my_gps_uart.readchar()))

    # parse the altitude at the first instance
    altitude_1 = my_gps.altitude

    # During this check, let's record the altitude and location
    # We should be able to monitor the decent of the device this way.
    log_sd = open('/sd/log.csv', 'a')
    log_sd.write('altitude:{},latitude:{},longitude:{}\n'.format(altitude_1,
                                                                  functions.convert_latitude(my_gps.latitude),
                                                                  functions.convert_longitude(my_gps.longitude)))
    log_sd.close()

    print('alt 1: {}'.format(altitude_1))

    time.sleep_ms(3000)

    # Read the GPS message and parse the altitude and its change from the first
    # time read
    while my_gps_uart.any():
        my_gps.update(chr(my_gps_uart.readchar()))
    altitude_2 = my_gps.altitude
    print('alt 2: {}'.format(altitude_2))

    change_in_altitude = altitude_2 - altitude_1

    # Print it, then save it to the log file
    print('alt change: {}'.format(change_in_altitude))
    log_sd = open('/sd/log.csv', 'a')
    log_sd.write('The change in altitude is {} at {}\n'.format(change_in_altitude, pyb.elapsed_millis(absolute_start_time)))
    log_sd.close()

pyb.delay(2000)

################# BURN OFF PARACHUTE ########################
functions.burn_parachute(450)
log_sd = open('/sd/log.csv','a')
log_sd.write('The parachute was burned at {}\n'.format(pyb.elapsed_millis(absolute_start_time)))
log_sd.close()
functions.motor_accel(100,100)
time.sleep_ms(10000)
functions.motor_deccel(100,10)

################# ESTABLISH LANDING POINT ######################
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

dist_from_goal = 100
################ ESTABLISH SECOND POINT #########################
#this is necessary to calculate the bearing

while dist_from_goal > 30:

    print('moving forward')
    functions.motor_accel(100,100)
    time.sleep_ms(6500)
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


functions.motor_deccel(100, 50)
log_sd = open('/sd/log.csv', 'a')
log_sd.write('The destination was reached at {}\n'.format(pyb.elapsed_millis(absolute_start_time)))
log_sd.close()

for x in range(0,1000):
    red_LED.toggle()
    time.sleep_ms(100)
    green_LED.toggle()
    time.sleep_ms(100)
    orange_LED.toggle()
    time.sleep_ms(100)
    blue_LED.toggle()
    time.sleep_ms(100)

log_sd = open('/sd/log.csv', 'a')
log_sd.write('WE DID IT\n')
log_sd.close()
