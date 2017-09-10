
#################### Import Libraries#######################
import pyb
from pyb import UART
from pyboard_razor_IMU import Razor
from pyb import Pin
from micropyGPS import MicropyGPS
from motor import motor
import time
import math
import functions
from pyb import ExtInt
################### Goal Coordinate Input ############################
# Insert coordinate given to team by ARLISS TEAM enter below

finish_point=(30.2107,-92.0209)

################# End User Input ##########################

################### Global Variables ######################

burn_time = 5000 # Time it takes to Burn Nychron wire completely [ms]
load_up_time_mins = 45 # mins it takes to load into rocket
load_up_time = load_up_time_mins * 60 * 1000 # Time it takes to load up fully into rocket [ms]
settling_time = 30000 # 30s settling time after rover has landed
black_rock_alt_m = 1191 # Altitude of black rock [meters]
alt_threshold = 1 # Allowable threshold altitude between GPS data and estimated Altitude
distance_tolerance = 3 # Allowable distance between goal and rover location [meters]




########### GPS / Interrupt setup ###########

'''The Adafruit GPS has a PPS pin that changes from high to low only when we are recieving data
    we will use this to our advantage by associating it with an iterrupt to change indicate we are recieving new data'''

# Global Flag to Start GPS data Processing
new_data = False

def pps_callback(line):
    print("Updated GPS Object...")
    global new_data # Use Global to trigger update
    new_data = True #

# Create an external interrupt on pin X8

pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, pps_callback)

# Grabbing GPS uart and associated object from 'functions.py'
my_gps_uart = functions.my_gps_uart
my_gps = functions.my_gps

########## End GPS / Interrupt setup ########

#Grabbing the Xbee object created in functions.py
xbee = functions.xbee

''''log.txt' will be the file that will document significant steps in the process
 'w' to create file
 'a'to append created file
 'r' to read existing file '''
# Create 'log.txt'
with open('/sd/log.txt', 'w') as log:
    log.write('Welcome to the ARLISS 2017 Log\n')

################### Begin Process #########################
process_start= pyb.millis()#Starting a time count at point of startup

#Append log that the process has started
with open('/sd/log.txt', 'a') as log:
    log.write('Process start\n Program initiated at: {}\n'.format(my_gps.timestamp))

pyb.delay(60000) # Wait a minute before sending data via xbee

#Send information via Xbee
xbee.write('\nprogram initiated at: {}'.format(my_gps.timestamp)) # Sending time stamp when process started
xbee.write('\ntarget point: {}'.format(finish_point)) # Sending Target Point

# Load up rover into rocket
pyb.delay(load_up_time) # This load up time accounts for the entire process leading up to launch

# Begin descent and monitor altitude
while True:
    # Update the GPS Object when flag is tripped
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        
        current_altitude = my_gps.altitude # Grabbing parameter designated by micropyGPS object
        pyb.delay(1000)
        new_data = False  # Clear the flag
        with open('/sd/log.csv', 'a') as log:
            log.write('{}\n'.format(current_altitude))
        if current_altitude < black_rock_alt_m + alt_threshold:
            xbee.write('Rover has landed beginning parachute burning')
            with open('/sd/log.csv', 'a') as log:
            log.write('I have landed with altitude: {}\n'.format(current_altitude))# Sending Target Point
            break


# Wait some time to let things settle
pyb.delay(settling_time)

#Burn Parachute and move for 10s
functions.burn_parachute(burn_time)
functions.move_forward(100)
pyb.delay(10000)
functions.stop()
xbee.write('Parachute burned successfully....Aquiring Location')
with open('/sd/log.csv', 'a') as log:
    log.write('Parachute successfully burned\n')

# Establish Landing Point
while True:
    xbee.write('Aquiring Location')
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        
        landing_lat = functions.convert_latitude(my_gps.latitude) # Grabbing parameter designated by micropyGPS object
        landing_lon = functions.convert_longitude(my_gps.longitude) # Grabbing parameter designated by micropyGPS object
        landing_point = (start_lat, start_long) # Creating single variable for utilization in calculations

        #Sending update via xbee
        xbee.write('Location aquired')
        xbee.write('\nLanding point: {}'.format(landing_point))
       
       #Appending log
        with open('/sd/log.csv', 'a') as log:
            log.write('{}\n'.format(landing_point))
        new_data = False  # Clear the flag
        
        #Using single part of landing point if one isnt equal to zero chances are neither is the other
        if landing_point[0] != 0:
            break


# Calculate Distance to goal
dist_from_goal = functions.calculate_distance(finish_point, landing_point)


xbee.write('\nDistance from Goal: {}'.format(dist_from_goal))
with open('/sd/log.csv', 'a') as log:
    log.write('Distance from goal: {}\n'.format(dist_from_goal))


# Establish an arbitrary point after landing point to aquire bearing
while True:
    
    functions.cruise control(10000,80) #Control motor speed to maintain bearing refer
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        start_lat = functions.convert_latitude(my_gps.latitude) # Grabbing parameter designated by micropyGPS object
        start_lon = functions.convert_longitude(my_gps.longitude) # Grabbing parameter designated by micropyGPS object
        start_point = (start_lat, start_long) # Creating single variable for utilization in calculations
        with open('/sd/log.csv', 'a') as log:
            log.write('Point to establish bearing aquired: {}\n'.format(start_point))
        new_data = False

        if landing_point[0] != 0:
            break


'''

######################## Begin Navigation loop #######################
#Establishing first location after separation of parachute

pyb.delay(10000) #Allowing for the gps to start recieving data
functions.move_forward(100)
pyb.delay(1000)
functions.stop()

#Establishing Start Point
while 1:
    # Do Other Stuff Here.......
    
    # Update the GPS Object when flag is tripped
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        
        start_lat = functions.convert_latitude(my_gps.latitude)
        start_lon = functions.convert_longitude(my_gps.longitude)
        start_point = (start_lat, start_long)
        pyb.delay(1000)
        new_data = False  # Clear the flag
        break

functions.move_forward(100)
pyb.delay(10000)
functions.stop()

#Establishing First Point
while 1:
    # Update the GPS Object when flag is tripped
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        
        first_lat = functions.convert_latitude(my_gps.latitude)
        first_lon = functions.convert_longitude(my_gps.longitude)
        first_point = (start_lat, start_long)
        pyb.delay(1000)
        new_data = False  # Clear the flag
        break

dist_from_goal = functions.calculate_distance(finish_point, start_point)
degree_to_turn = functions.bearing_difference(finish_point, start_point,first_point)
functions.angle_to_motor_turn(wheel_separation, wheel_radius, gain, degree_to_turn[0], degree_to_turn[1])


past_point = first_point
pyb.LED(3).on()
pyb.delay(2000)
pyb.LED(3).off()

#Start Movement
while True:
    #This loop relies on checking for its location and replacing the initial_point with the most recently recorded point and determining bearing, distance, corrections etc..'''
    initial_point = past_point
    functions.move_forward(100)
    pyb.LED(3).on()
    pyb.delay(2000)
    pyb.LED(3).off()
    while True:
    #This loop preforms IMU control of the motors in order to maintain the rovers bearing for 20s before the next locational check refer to functions.py for further details'''               
        start = pyb.millis() # get value of millisecond counter
        functions.move_forward(50)
        functions.imu_pid()
        if pyb.elapsed_millis(start) > 20000:
            functions.stop()
            break

    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        
            pres_lat = functions.convert_latitude(my_gps.latitude)
            pres_lon = functions.convert_longitude(my_gps.longitude)
            present_point = (start_lat, start_long)
            pyb.delay(1000)
            new_data = False  # Clear the flag
            break

    dist_from_goal = functions.calculate_distance(finish_point,present_point)
    degree_to_turn = functions.bearing_difference(finish_point, initial_point,present_point)
    
    functions.angle_to_motor_turn(wheel_separation, wheel_radius, gain, degree_to_turn[0], degree_to_turn[1])
    pyb.delay(1000)
    past_point = present_point
    #turn rover degree
    elif dist_from_goal < distance_tolerance:
        functions.stop()
        #xbee.write('\nDestination reached, present location: {}'.format(present_point))
        break
    else:
        continue
    




