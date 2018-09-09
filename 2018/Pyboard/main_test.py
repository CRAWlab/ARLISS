##################### IMPORT LIBRARIES ####################
from motor import motor
import time
import pyb
import machine
from pyb import Pin
from configurations import *
from pyboard_razor_IMU import Razor
from pyb import ExtInt
#import functions
import math
from micropyGPS import MicropyGPS

################## GLOBAL FLAG FOR GPS PROCESSING ###############

new_data = False
def pps_callback(line):
    #print('Updated GPS Object...')
    global new_data
    new_data = True
pps_pin = pyb.Pin.board.X5
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, pps_callback)


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

start_time = pyb.millis()

while pyb.elapsed_millis(start_time) < 5*60*1000:
    time.sleep_ms(1000)
    #time delay on looking for gps coordinates


#this sequence test first if there are gps coordinates to be read,
#then tests the change in altitude to see if it has landed,
#then tests to see if there is any change in the accelerometer values
#the accelerometer essentially tests to make sure it is not bouncing
#from the impact
#there is a time limit condition associated with it which, if exceeded,
#will trigger the burn wire

start_time = pyb.millis()

while pyb.elapsed_millis(start_time) < 60*1000*90 and new_data == 0:
    print(new_data)
    time.sleep_ms(500)
while pyb.elapsed_millis(start_time) < 60*1000*90 and change_in_altitude < -5:
    gps.update(chr(uart.readchar()))
    altitude_1 = my_gps.altitude
    print(altitude_1)
    time.sleep_ms(3*1000)
    gps.update(chr(uart.readchar()))
    altitude_2 = my_gps.altitude
    print(altitude_2)
    change_in_altitude = altitude_2 - altitude_1
    print(change_in_altitude)

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
    change_in_accel_abs = [abs(x) for x in change_in_accel]
    print(change_in_accel)

functions.burn_parachute(10*1000)
functions.move_forward(100)
time.sleep_ms(10*1000)
functions.stop()


while 1:
    if my_gps_uart.any():
        valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))
        if valid_sentence_received:
            first_lat = functions.convert_latitude(my_gps.latitude)
            first_lon = functions.convert_longitude(my_gps.longitude)
            first_point = (start_lat, start_long)
            time.sleep_ms(1000)
            break

functions.move_forward(100)
pyb.delay(10*1000)
functions.stop()

while 1:
    if my_gps_uart.any():
        valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))
        if valid_sentence_received:
            second_lat = functions.convert_latitude(my_gps.latitude)
            second_lon = functions.convert_longitude(my_gps.longitude)
            second_point = (start_lat, start_long)
            time.sleep_ms(1000)
            break

dist_from_goal = functions.calculate_distance(finish_point, first_point)
degree_to_turn = function.bearing_difference(finish_point, first_point, second_point)
functions.angle_to_motor_turn(wheel_separation, wheel_radius, gain, degree_to_turn[0], degree_to_turn[1])

past_point = second_point
orange_LED.on()
time.sleep_ms(2*1000)
orange_LED.off()

while True:
    initial_point = past_point
    functions.move_forward(100)
    orange_LED.on()
    time.sleep_ms(2*1000)
    orange_LED.off()
    while True:
        start_time = pyb.millis()
        functions.move_forward(50)
        functions.imu.pid()
        if pyb.elapsed_millis(start_time) >20000:
            functions.stop()
            break
    valid_sentence_received = my_gps.update(chr(my_gps_uart.readchar()))
    if valid_sentence_received:
        present_lat = functions.convert_latitude(my_gps.latitude)
        present_lon = functions.convert_longitude(my_gps.longitude)
        present_point = (start_lat, start_lon)
        time.sleep_ms(1000)
        break

    dist_from_goal = functions.calculate_distance(finish_point, present_point)
    degree_to_turn = functions.bearing_difference(finish_point, initial_point, present_point)
    functions.angle_to_motor_turn(wheel_separation, wheel_radius, gain, degree_to_turn[0], degree_to_turn[1])
    time.sleep_ms(1000)
    past_point = present_point
    #elif dist_from_goal < distance_tolerance:
        #functions.stop()
        #break
    #else:
        #continue
