'''ARLISS 2017
    'main.py'
    Created By: Joseph Fuentes - jaf1036@louisiana.edu -08/01/2017
        In Collaboration With: Dr. Joshua Vaughan - joshua.vaughan@louisiana.edu
                            Forrest Montgomery
                            ARLISS teams 2015, 2016
                            
                            
    Rover type: Tank
    MCU: Pyboard v1.1 
    
    Include files on local/sd/:
    'boot.py'
    'functions.py'
    'pyboard_razor_IMU.py'
    'micropyGPS.py'
    'motor.py'
    'pyboard_PID.py'
    
    Modeified: Joseph Fuentes 08/09/2017
                    Commented/ clean-up
                
    
    '''
#################### Import Libraries#########################
import pyb
import machine
import time
import math
import functions
from pyb import UART
from pyb import Pin
from pyb import ExtInt

################### Goal Coordinate Input #####################
# Coordinate given to team by ARLISS Coordinators:

finish_point=(30.2107,-92.0209)

####################### End Input #############################

################### Global Variables ##########################

burn_time = 5000 # Time it takes to Burn Nychron wire completely [ms]
load_up_time_mins = 45 # mins it takes to load into rocket
load_up_time = load_up_time_mins * 60 * 1000 # Time it takes to load up fully into rocket [ms]
settling_time = 30000 # 30s settling time after rover has landed
black_rock_alt_m = 1191 # Altitude of black rock [meters]
alt_threshold = 1 # Allowable threshold altitude between GPS data and estimated Altitude
distance_tolerance = 3 # Allowable distance between goal and rover location [meters]
new_data = False # Global Flag to Start GPS data Processing

################### End Global Variables ######################


################### GPS / Interrupt setup #####################

'''The Adafruit GPS has a PPS pin that changes from high to low only when we are recieving data
    we will use this to our advantage by associating it with an iterrupt to change indicate we are recieving new data'''
def pps_callback(line):
    '''The Adafruit GPS has a PPS pin that changes from high to low only when we are recieving data
        we will use this to our advantage by associating it with an iterrupt to change indicate we are recieving new data'''
    global new_data # Use Global to trigger update
    new_data = True # Raise flag

# Create an external interrupt on pin X8
pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, pps_callback)

# Grabbing GPS uart and associated object from 'functions.py'
my_gps_uart = functions.my_gps_uart
my_gps = functions.my_gps

################### End GPS / Interrupt setup #################

# Grabbing the Xbee object created in functions.py
xbee = functions.xbee

# Create 'log.txt'
''''log.txt' will be the file that will document significant steps in the process
 'w' to create file
 'a'to append created file
 'r' to read existing file '''

with open('/sd/log.txt', 'w') as log:
    log.write('Welcome to the ARLISS 2017 Log\n')

###################### Begin Process ##########################
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
        if current_altitude != 0:
            continue
        pyb.delay(1000)
        new_data = False  # Clear the flag
        with open('/sd/log.txt', 'a') as log:
            log.write('{}\n'.format(current_altitude))
        if current_altitude < black_rock_alt_m + alt_threshold:
            xbee.write('Rover has landed beginning parachute burning')
            with open('/sd/log.txt', 'a') as log:
            log.write('I have landed with altitude: {}\n'.format(current_altitude))# Sending Target Point
            break


# Wait some time to let things settle
pyb.delay(settling_time)

# Burn Parachute and move for 10s
functions.burn_parachute(burn_time)


functions.cruise_control(10000,80) # Duration 10s, 80% duty cycle
functions.stop()
xbee.write('Parachute burned successfully....Aquiring Location')
with open('/sd/log.txt', 'a') as log:
    log.write('Parachute successfully burned\n')

############ Establish Landing / Aquire Bearing / Correct course ########

# Establish Landing Point
while True:
    xbee.write('Aquiring Location...')
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        
        landing_lat = functions.convert_latitude(my_gps.latitude) # Grabbing parameter designated by micropyGPS object
        landing_lon = functions.convert_longitude(my_gps.longitude) # Grabbing parameter designated by micropyGPS object
        landing_point = (landing_lat, landing_lon) # Creating single variable for utilization in calculations
        
        # Using single part of landing point if one isnt equal to zero chances are neither is the other
        if landing_lat != 0:
            continue
        #Sending update via xbee
        xbee.write('Location aquired')
        xbee.write('\nLanding point: {}'.format(landing_point))
       
       #Appending log
        with open('/sd/log.txt', 'a') as log:
            log.write('{}\n'.format(landing_point))
        new_data = False  # Clear the flag
        break
        #Using single part of landing point if one isnt equal to zero chances are neither is the other



# Calculate Distance to goal
dist_from_goal = functions.calculate_distance(finish_point, landing_point)


xbee.write('\nDistance from Goal: {}'.format(dist_from_goal))
with open('/sd/log.txt', 'a') as log:
    log.write('Distance from goal: {}\n'.format(dist_from_goal))


# Establish an arbitrary point after landing point to aquire bearing
while True:
    '''Using Cruise control to drive straight
        Duration = 10 seconds
        Desired_speed = 80% duty cycle
        '''
    functions.cruise_control(10000,80)
    
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
        arb_lat = functions.convert_latitude(my_gps.latitude) # Grabbing parameter designated by micropyGPS object
        arb_lon = functions.convert_longitude(my_gps.longitude) # Grabbing parameter designated by micropyGPS object
        arbitrary_point = (arb_lat, arb_lon) # Creating single variable for utilization in calculations
        if landing_point[0] != 0:
            continue
        
        with open('/sd/log.txt', 'a') as log:
            log.write('Point to establish bearing aquired: {}\n'.format(arbitrary_point))
        new_data = False
        break

# Now that 3 points have been established we can establish bearing and correct course
functions.bearing_difference(finish_point,landing_point, arbitrary_point)
course_error_gain = 0.5
# Notifying course is changing
xbee.write('Bearing calculated, correcting course')
functions.correct_course_error(course_error_gain)
distance_to_goal = functions.calculate_distance(arbitrary_point,finish_point)

#################### Begin Navigation Loop ##################
'''The navigation loop will behave as follows:
    -Travel for a time interval
    -Record location 
    -Calculate distance / correct course
    -Compare current distance to previous distance
    -if current distance < previous distance increase travel time
    -if current distance > previous distance decrease travel time
    -Reach Goal?
        -Yes: Stop, break loop
        -No: Preform another iteration
    '''
previous_point = arbitrary_point
current_dist = distance_to_goal
with open('/sd/log.txt', 'a') as log:
    log.write('Distance to Goal: {}\n'.format(current_dist))

travel_time = 20000

while True:
    previous_dist = current_dist
    previous_point = current_point
# Travel for time interval
    functions.cruise_control(travel_time,80)
    
# Record Location
    xbee.write('Aquiring Location.....')
    if new_data:
        while my_gps_uart.any():
            my_gps.update(chr(my_gps_uart.readchar()))
        current_lat = functions.convert_latitude(my_gps.latitude)
        current_lon = functions.convert_longitude(my_gps.longitude)
        current_point = (current_lat, current_lon)
        xbee.write('Location Aquired.... Calculating Distance to goal')
        with open('/sd/log.txt', 'a') as log:
            log.write('Location: {}\n'.format(current_point))
        new_data = False
        continue
# Calculate distance / correct course
    current_dist = functions.calculate_distance(current_point,finish_point)
    functions.correct_course_error(course_error_gain)
    with open('/sd/log.txt', 'a') as log:
        log.write('Distance to Goal: {}\n'.format(current_dist))
# If current distance < previous distance increase travel time
    if current_dist < previous_dist:
        xbee.write('Distance decreased increasing travel time')
        travel_time += 10000
# If current distance > previous distance decrease travel time
    elif current_dist > previous_dist:
        xbee.write('Distance increased decreasing travel time')
        travel_time -= 10000

# Getting close to goal slow down
    elif current_dist < 20:
        xbee.write('Getting close to goal decreasing travel time')
        travel_time -= 10000
        
        # To prevent zeroing out travel time
        if travel_time < 10000:
            travel_time -= < 1000
        elif travel_time == 0:
            travel_time = 5000
                
# Goal Reached?????
# Yes!
    if distance_to_goal < distance_tolerance:
        functions.stop()
        break
# No :( preform another iteration
    else:
        continue

############## Rover within Goal tolerance #############
# Record final point
with open('/sd/log.txt', 'a') as log:
    log.write('Location: {}\n'.format(current_point))
log.close() #Closing log file
total_time = pyb.elapsed_millis(process_start/60000) #Total process time in mins

# Begin celebration!!
xbee.write('Goal reached!!!!!\n Distribute high-fives\n')
functions.turn_left(100)
pyb.delay(5000)
functions.stop()

