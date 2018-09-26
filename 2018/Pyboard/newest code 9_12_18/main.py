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

    time.sleep_ms(50)
    print('Trigger: {}'.format(pyb_switch))

    if pyb_switch == 1:
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
# TODO: 09/12/18 - JEV - What is the reason for this while loop?
while pyb.elapsed_millis(start_time) < 5000:
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

while pyb.elapsed_millis(start_time) < FORCE_START_TIMER and new_data == 0:
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
# TODO: 09/12/18 - JEV - need to test noise in GPS altitude signal to determine
#                        the proper value for change in altitude check
while pyb.elapsed_millis(start_time) < FORCE_START_TIMER and change_in_altitude < -5:

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


############# CONFIRM DEVICE HAS LANDED USING ACCELEROMETER #############
while pyb.elapsed_millis(start_time) < FORCE_START_TIMER and change_in_accel_abs > [2,2,2]:
    # TODO: 09/11/18 - JEV - Are these the right changes in accel for this limit?
    accel_x_1 = accel.x()
    accel_y_1 = accel.y()
    accel_z_1 = accel.z()

    time.sleep_ms(1000)

    accel_x_2 = accel.x()
    accel_y_2 = accel.y()
    accel_z_2 = accel.z()

    change_in_accel_x = accel_x_2 - accel_x_1
    change_in_accel_y = accel_y_2 - accel_y_1
    change_in_accel_z = accel_z_2 - accel_z_1
    change_in_accel = [change_in_accel_x, change_in_accel_y, change_in_accel_z]

    log_sd = open('/sd/log.csv', 'a')
    log_sd.write('The change in acceleration {} at {}\n'.format(change_in_accel, pyb.elapsed_millis(absolute_start_time)))
    log_sd.close()

    change_in_accel_abs = [abs(x) for x in change_in_accel]
    print(change_in_accel_abs)

################# BURN OFF PARACHUTE ########################
functions.burn_parachute(400)
log_sd = open('/sd/log.csv','a')
log_sd.write('The parachute was burned at {}\n'.format(pyb.elapsed_millis(absolute_start_time)))
log_sd.close()

# Drive forward for 10 seconds
functions.motor_accel(100,50)
time.sleep_ms(10000)
functions.motor_deccel(100,50)

################# ESTABLISH LANDING POINT ######################
while True:
    while my_gps_uart.any():
        valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))

        if valid_sentence_received:
            first_lat = functions.convert_latitude(my_gps.latitude)
            first_lon = functions.convert_longitude(my_gps.longitude)
            first_point = (first_lat, first_lon)

            log_sd = open('/sd/log.csv', 'a')
            log_sd.write('The first recorded latitude, longitude is {} at {}.\n'.format(first_point, pyb.elapsed_millis(absolute_start_time)))
            log_sd.close()
            time.sleep_ms(1000)
            break

# Drive forward for 10 seconds
functions.motor_accel(100,50)
time.sleep_ms(10000)
functions.motor_deccel(100,50)

################ ESTABLISH SECOND POINT #########################
#this is necessary to calculate the bearing

while True:
    while my_gps_uart.any():
        valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))

        if valid_sentence_received:
            second_lat = functions.convert_latitude(my_gps.latitude)
            second_lon = functions.convert_longitude(my_gps.longitude)
            second_point = (second_lat, second_lon)

            log_sd = open('/sd/log.csv', 'a')
            log_sd.write('The second recorded (latitude, longitude) is {} at {}\n'.format(second_point, pyb.elapsed_millis(absolute_start_time)))
            log_sd.close()
            time.sleep_ms(1000)
            break


############ CALCULATE BEARING AND DISTANCE FROM GOAL ###############
dist_from_goal = functions.calculate_distance(finish_point, first_point)
degree_to_turn = function.bearing_difference(finish_point, first_point, second_point)

# TODO: 09/11/18 - JEV - update this to finer control
functions.course_correct(degree_to_turn)

log_sd = open('/sd/log.csv', 'a')
log_sd.write('The first recorded distance from the goal is {} at {}\n'.format(dist_from_goal, pyb.elapsed_millis(absolute_start_time)))
log_sd.write('The degree change needed to achieve this goal is {}\n'.format(degree_to_turn))
log_sd.close()

past_point = second_point

# Toggle LED light to indicate location in code progression
orange_LED.on()
time.sleep_ms(2000)
orange_LED.off()


################### MAIN NAVIGATION LOOP #########################
while True:
    iteration_number = iteration_number + 1
    log_sd = open('/sd/log.csv', 'a')
    log_sd.write('Iteration:    {}\n'.format(iteration_number))
    log_sd.write('Time:         {}\n'.format(pyb.elapsed_millis(absolute_start_time)))
    log_sd.close()

    initial_point = past_point
    functions.motor_accel(100, 50)

    # Toggle orange LED again
    orange_LED.on()
    time.sleep_ms(2000)
    orange_LED.off()

    while True:
        start_time = pyb.millis()
        functions.move_forward(75)

        # TODO: 09/11/18 - JEV - This is not right, I think.
        functions.imu_pid(duration, 75)

        if pyb.elapsed_millis(start_time) > 20000:
            functions.motor_deccel()
            break

    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))

        present_lat = functions.convert_latitude(my_gps.latitude)
        present_lon = functions.convert_longitude(my_gps.longitude)
        present_point = (start_lat, start_lon)

        log_sd = open('/sd/log.csv', 'a')
        log_sd.write('The Present Point is {} at {}\n'.format(present_point, pyb.elapsed_millis(absolute_start_time)))
        log_sd.close()

        #need to add a stuck function here based on the present point and past point
        #save the data to show that the stuck function was used
        dist_from_goal = functions.calculate_distance(finish_point, present_point)
        degree_to_turn = functions.bearing_difference(finish_point, initial_point, present_point)
        functions.course_correct(degree_to_turn)

        log_sd = open('/sd/log.csv', 'a')
        log_sd.write('The distance from goal is {} at {}\n'.format(dist_from_goal, pyb.elapsed_millis(absolute_start_time)))
        log_sd.write('The degree change needed to achieve this goal is {}\n'.format(degree_to_turn))
        log_sd.close()

        # Sleep for 1s
        time.sleep_ms(1000)
        past_point = present_point
        new_data=False

    elif dist_from_goal < distance_tolerance:
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

        break
    else:
        continue
