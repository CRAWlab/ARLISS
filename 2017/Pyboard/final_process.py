'''main.py
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
    
    This file is a higher level process for the ARLISS robot project
    **This process relies heavily on boot.py and functions.py make sure appropriate files are in directory**
    Process Algorithm:
    -Accept goal coordinates
    -Await ascent
    -Detach from rocket
    -Detect descent
    -Once rover is no longer descending communicate that it has landed
    -Determine the orienation of how the tank has landed
    -Establish fix and get first location
    -Begin output signal to burn parachute 
    -Wait time period [burn_time] to insure parachute has finished burning off
    -End relay output signal
    -Aquire Current location 
    -Drive forward for time period [delta_foward]
    -Aquire new location and bearing of rover
    -Calculate Distance to target 
    -Turn to face target
    -Begin Travel loop
        -Maintain Bearing 
            IMU:
                -Monitor angular rotation 
                -Send Signal To motors of rotation is detected
            Motors: 
                -Use PID's to maintain desired speed 
                -Coorporate with the IMU to ensure straight travel 
        -Distance From Target 
            GPS: 
                -Check Position as a function 
                    [Position needs to be checked more frequently as the rover is further from he target]
                    
'''
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
################### User Input ############################
finish_point=
launch_point=

################# End User Input ##########################

###########################################################

''' Pinout Section for the Pyboard
    
    Motors/ Drivers:
    A=Left Track
    A_EN= 'Y3'
    A_PH= 'Y5'
    B=Right Track
    B_EN= 'Y4'
    B_PH= 'Y6'
    
    Encoder:
    A=Left Track
    enc_A_chan_A='X1'
    enc_A_chan_B='X2'
    B=Right Track
    enc_B_chan_A='Y7'
    enc_B_chan_B='Y8'
    GPS:
    -Top GPS:
    UART 3
    Baudrate=9600
    PPS = 'X8'
    -Bottom GPS:
    UART 6
    Baudrate=9600
    PPS = 'X7'
    
    IMU: 
    Internal 
    Razor IMU 
        UART 1 
        Baudrate= 57600
    
    XBEE:
    UART 2
    Baudrate=115200
    
    
    Relay = 'Y11'
'''



################### Global Variables ######################
burn_time = 10000 #Time it takes to Burn Nychron wire completely [ms]
backup_timer = 5400000
altitude_concurrent_timer = 3600000
acceptable_dist_from_launch = 1000
acceptable_altitude_change = 25
black_rock_alt_ft = 3907
black_rock_alt_m = 1191
threshold = 1
distance_tolerance = 3
wheel_separation =
wheel_radius =
gain = 0.5


################### Begin Process #########################
load_up = pyb.delay(10000) #Creation of delay in process in order to give time for setup

process_start= pyb.millis()#Starting a time count

#Sending information via XBee to monitoring station
xbee.write('\nprogram initiated at: {}'.format(my_gps.timestamp))
xbee.write('\ntarget point: {}'.format(finish_point))
xbee.write('\nlaunching at: {}'.format(launch_point))

#Altitude check to see if rover has landed
while True:
    my_gps.update(chr(uart.readchar()))
    if my_gps.altitude != 0 #Condition for gps recieving data
        altitude = mygps.altitude
        if altitude =< (black_rock_alt_m + threshold): #Condition if rover lands within threshold of blackrock
            xbee.write('\nRover has landed, Altitude: {}'.format(altitude))
            break
        else:
            xbee.write('\nRover is in descent')
            continue
    elif pyb.elapsed_millis(start) >= backup_timer: #Back up timer to start process
            break



#Checking orientation
orientation_check = functions.get_landing_orientation()
#defining which gps is in use
my_gps = orientation_check[0] #Assigning whichever gps is in use as main
# sending the orientation to observer
xbee.write(orientation_check[1]) #Sending monitor which gps is in use


#Establishing landing point
while True:
    my_gps.update(chr(uart.readchar()))
        if my_gps.latitude[0] != 0: #Recieving Data
            xbee.write('\nAquiring GPS Data')
            pyb.delay(500)
            landing_lat = functions.convert_latitude(my_gps.latitude)
            landing_long = functions.convert_longitude(my_gps.longitude)
            landing_point = (landing_lat, landing_long)
            xbee.write('\nLanding point: {}'.format(landing_point))
            break
        else: #No usable data found continue checking
            xbee.write('\nAquiring GPS Data')
            pyb.delay(500)
            continue

#Calculate Distance to goal
dist_from_goal = functions.calculate_distance(finish_point, landing_point)
xbee.write('\nDistance from Goal: {}'.format(dist_from_goal))

#Burn Parachute and move for 10s
functions.burn_parachute(burn_time)
functions.move_forward(100)
pyb.delay(10000)
functions.stop()

######################## Begin Navigation loop #######################
#Establishing first location after separation of parachute
while True:
    if new_data == True:
        while True:
            my_gps.update(chr(uart.readchar()))
            if my_gps.latitude[0] != 0:
                start_lat = functions.convert_latitude(my_gps.latitude)
                start_long = functions.convert_longitude(my_gps.longitude)
                start_point = (start_lat, start_long)
                new_data = False
                break
            else:
                new_data = False

#Sending the location of the rover this marks the navigation process
xbee.write('\nI will begin navigation at this location: {}'.format(past_point))
dist_from_goal = functions.calculate_distance(finish_point, start_point)
degree_to_turn = functions.bearing_difference(finish_point, landing_point,start_point)
functions.angle_to_motor_turn(wheel_separation, wheel_radius, gain, degree_to_turn[0], degree_to_turn[1])
past_point = start_point

#Start Movement
while True:
    '''This loop relies on checking for its location and replacing the initial_point with the most recently recorded point and determining bearing, distance, corrections etc..'''
    initial_point = past_point
    functions.move_forward(100)
    
    while True:
            '''This loop preforms IMU control of the motors in order to maintain the rovers bearing for 20s before the next locational check refer to functions.py for further details'''
                
        start = time.ticks_ms() # get value of millisecond counter
        functions.imu_pid()
        delta = time.ticks_diff(time.ticks_ms(), start) # compute time difference
        if delta == (20000):
            break

    functions.stop()
    my_gps.update(chr(uart.readchar()))
    pres_lat = convert_latitude(my_gps.latitude)
    pres_long = convert_longitude(my_gps.longitude)
    present_point = (pres_lat, pres_long)
    
    dist_from_goal = functions.calculate_distance(finish_point,present_point)
    degree_to_turn = functions.bearing_difference(finish_point, initial_point,present_point)
    
    functions.angle_to_motor_turn(wheel_separation, wheel_radius, gain, degree_to_turn[0], degree_to_turn[1])
    pyb.delay(1000)
    past_point = present_point
    #turn rover degree
    if dist_from_goal < distance_tolerance:
        functions.stop()
        xbee.write('\nDestination reached, present location: {}'.format(present_point))
        break
    else:
        continue
    




