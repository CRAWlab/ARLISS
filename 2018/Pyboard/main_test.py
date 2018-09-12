##################### IMPORT LIBRARIES ####################
from motor import motor
import time
import pyb
import machine
from pyb import Pin
from configurations import *
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
    trigger_Pin_Value = trigger_Pin.value()
    time.sleep_ms(50)
    print(trigger_Pin_Value)
    if trigger_Pin_Value == 1:
        print('Ready for Launch!')
        red_LED.off()
        green_LED.on()
        break
        break
    else:
        red_LED.toggle()
time.sleep_ms(2500)
green_LED.off()
#save this to get an idea on how long the code was running
absolute_start_time = pyb.millis()
log_sd = open('/sd/log.csv','w')
log_sd.write('The start time of the code is {}\n').format(absolute_start_time)
log_sd.close()
#will be a good idea to mark the time down
start_time = pyb.millis()

###### TIME DELAY ON LOOKING FOR GPS COORDINATES ##############
while pyb.elapsed_millis(start_time) < 5*60*1000:
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
log_sd = open('/sd/log/csv', 'w')
log_sd.write('The first GPS instance is {}\n').format(pyb.elapsed_millis(absolute_start_time))
log_sd.close()

###################### CALCULATE ALTITUDE CHANGE ######################
altitude_check = 1
while pyb.elapsed_millis(start_time) < 60*1000*90 and change_in_altitude < -5:

    while my_gps_uart.any():
        my_gps.update(chr(my_gps_uart.readchar()))
    altitude_1 = my_gps.altitude

    while altitude_check == 1:
        log_sd = open('/sd/log/csv', 'w')
        log_sd.write('The first recorded altitude is {}\n').format(altitude_1)
        log_sd.write('The first recorded latitude is {}\n').format(functions.convert_latitude(my_gps.latitude))
        log_sd.write('The first recorded longitude is {}\n').format(functions.convert_longitude(my_gps.longitude))
        log_sd.close()
        altitude_check = altitude_check + 1
        print(altitude_1)
    time.sleep_ms(3000)


    while my_gps_uart.any():
        my_gps.update(chr(my_gps_uart.readchar()))
    altitude_2 = my_gps.altitude
    change_in_altitude = altitude_2 - altitude_1
    print(change_in_altitude)
    log_sd = open('/sd/log/csv', 'w')
    log_sd.write('The change in altitude is {} at {}\n').format(change_in_altitude, pyb.elasped_millis(absolute_start_time))
    log_sd.close()

############# CONFIRM DEVICE HAS LANDED USING ACCELEROMETER #############
while pyb.elapsed_millis(start_time) < 60*1000*90 and change_in_accel_abs > [2,2,2]:
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
    log_sd = open('/sd/log/csv', 'w')
    log_sd.write('The change in acceleration {} at {}\n'.format(change_in_accel, pyb.elapsed_millis(absolute_start_time)))
    log_sd.close()
    change_in_accel_abs = [abs(x) for x in change_in_accel]
    print(change_in_accel_abs)

################# BURN OFF PARACHUTE ########################
functions.burn_parachute(400)
log_sd = open('/sd/log/csv','w')
log_sd.write('The parachute was burned at {}\n').format(pyb.elapsed_millis(absolute_start_time))
functions.move_forward(100)
time.sleep_ms(10000)
functions.stop()

################# ESTABLISH LANDING POINT ######################
while True:
    if my_gps_uart.any():
        valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))
        if valid_sentence_received:
            first_lat = functions.convert_latitude(my_gps.latitude)
            first_lon = functions.convert_longitude(my_gps.longitude)
            first_point = (first_lat, first_long)
            log_sd = open('/sd/log.csv', 'w')
            log_sd.write('The first recorded (latitude, longitude) is {} at {}.\n').format(first point, pyb.elapsed_millis(absolute_start_time))
            log_sd.close()
            time.sleep_ms(1000)
            break

functions.move_forward(100)
time.sleep_ms(1000)
functions.stop()

################ ESTABLISH SECOND POINT #########################
#this is necessary to calculate the bearing

while True:
    if my_gps_uart.any():
        valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))
        if valid_sentence_received:
            second_lat = functions.convert_latitude(my_gps.latitude)
            second_lon = functions.convert_longitude(my_gps.longitude)
            second_point = (second_lat, second_long)
            log_sd = open('/sd/log.csv', 'w')
            log_sd.write('The second recorded (latitude, longitude) is {} at {}\n').format(second_point, pyb.elapsed_millis(absolute_start_time))
            log_sd.close()
            time.sleep_ms(1000)
            break
############ CALCULATE BEARING AND DISTANCE FROM GOAL ###############

dist_from_goal = functions.calculate_distance(finish_point, first_point)
degree_to_turn = function.bearing_difference(finish_point, first_point, second_point)
functions.angle_to_motor_turn(wheel_separation, wheel_radius, gain, degree_to_turn[0], degree_to_turn[1])
log_sd = open('/sd/log/csv', 'w')
log_sd.write('The first recorded distance from the goal is {} at {}\n').format(dist_from_goal, pyb.elapsed_millis(absolute_start_time))
log_sd.write('The degree change needed to achieve this goal is {}\n').format(degree_to_turn)
log_sd.close()
past_point = second_point
orange_LED.on()
time.sleep_ms(2000)
orange_LED.off()


################### MAIN NAVIGATION LOOP #########################
while True:
    iteration_number = iteration_number + 1
    log_sd = open('/sd/log.csv', 'w')
    log_sd.write('Iteration:    {}\n').format(iteration_number)
    log_sd.write('Time:     {}\n').format(pyb.elapsed_millis(absolute_start_time))
    log_sd.close()
    initial_point = past_point
    functions.motor_accel(100, 50)
    orange_LED.on()
    time.sleep_ms(2*1000)
    orange_LED.off()
    while True:
        start_time = pyb.millis()
        functions.move_forward(75)
        functions.imu_pid(duration, 75)
        if pyb.elapsed_millis(start_time) > 20000:
            functions.stop()
            break
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))

        present_lat = functions.convert_latitude(my_gps.latitude)
        present_lon = functions.convert_longitude(my_gps.longitude)
        present_point = (start_lat, start_lon)
        log_sd = open('/sd/log.csv', 'w')
        log_sd.write('The Present Point is {} at {}\n').format(present_point, pyb.elapsed_millis(absolute_start_time))
        log_sd.close()
        #need to add a stuck function here based on the present point and past point
        #save the data to show that the stuck function was used
        dist_from_goal = functions.calculate_distance(finish_point, present_point)
        degree_to_turn = functions.bearing_difference(finish_point, initial_point, present_point)
        functions.angle_to_motor_turn(wheel_separation, wheel_radius, gain, degree_to_turn[0], degree_to_turn[1])
        log_sd = open('/sd/log.csv', 'w')
        log_sd.write('The distance from goal is {} at {}\n').format(dist_from_goal, pyb.elapsed_millis(absolute_start_time))
        log_sd.write('The degree change needed to achieve this goal is {}\n').format(degree_to_turn)
        log_sd.close()
        time.sleep_ms(1000)
        past_point = present_point
        new_data=False
    elif dist_from_goal < distance_tolerance:
        functions.motor_deccel(100, 50)
        log_sd = open('/sd/log.csv', 'w')
        log_sd.write('The destination was reached at {}\n'.format(pyb.elapsed_millis(absolute_start_time)))
        log_sd.close()
        for x in range(0:1000):
            red_LED.toggle()
            time.sleep_ms(100)
            green_LED.toggle()
            time.sleep_ms(100)
            orange_LED.toggle()
            time.sleep_ms(100)
            blue_LED.toggle()
            time.sleep_ms(100)
        log_sd = open('/sd/log.csv', 'w')
        log_sd.write('WE DID IT\n')
        log_sd.close()

        break
    else:
        continue
