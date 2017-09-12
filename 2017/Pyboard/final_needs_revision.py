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
altitude = functions.monitor_descent()
with open('/sd/log.txt', 'a') as log:
    log.write('I have landed with altitude: {}\n'.format(altitude))

xbee.write('Rover has landed beginning parachute burning')


# Wait some time to let things settle
pyb.delay(settling_time)

# Burn Parachute and move for 10s
functions.burn_parachute(burn_time)

functions.move_forward(80)
cruise_speed = 80
functions.cruise_control(10000,cruise_speed) # Duration 10s, 80% duty cycle
xbee.write('Parachute burned successfully....Aquiring Location')
with open('/sd/log.txt', 'a') as log:
    log.write('Parachute successfully burned\n')

############ Establish Landing / Aquire Bearing / Correct course ########

# Establish Landing Point
xbee.write('Aquiring Location...')
landing_point = functions.get_location
xbee.write('Location aquired\n')
xbee.write('Landing point: {}\n'.format(landing_point))
with open('/sd/log.txt', 'a') as log:
    log.write('{}\n'.format(landing_point))

# Calculate Distance to goal
dist_from_goal = functions.calculate_distance(finish_point, landing_point)


xbee.write('\nDistance from Goal: {}'.format(dist_from_goal))

with open('/sd/log.txt', 'a') as log:
    log.write('Distance from goal: {}\n'.format(dist_from_goal))

functions.move_forward(80)
cruise_speed = 80
functions.cruise_control(10000,cruise_speed) # Duration 10s, 80% duty cycle
# Establish an arbitrary point after landing point to aquire bearing
arbitrary_point = functions.get_location()
with open('/sd/log.txt', 'a') as log:
    log.write('Point to establish bearing aquired: {}\n'.format(arbitrary_point))

# Now that 3 points have been established we can establish bearing and correct course
course_error_gain = 0.5
# Notifying course is changing
functions.correct_course_error(course_error_gain, finish_point,landing_point,arbitrary_point)
xbee.write('Bearing calculated, correcting course')

distance_to_goal = functions.calculate_distance(arbitrary_point,finish_point)
with open('/sd/log.txt', 'a') as log:
    log.write('Distance from goal: {}\n'.format(dist_from_goal))

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
current_point = arbitrary_point
current_dist = distance_to_goal
travel_time = 20000

while True:
    previous_dist = current_dist
    previous_point = current_point
# Travel for time interval
    functions.move_forward(80)
    cruise_speed = 80
    functions.cruise_control(travel_time,cruise_speed)
    
# Record Location
    xbee.write('Aquiring Location.....')
    current_point = functions.get_location()
        
    xbee.write('Location Aquired.... Calculating Distance to goal')
    with open('/sd/log.txt', 'a') as log:
        log.write('Location: {}\n'.format(current_point))

# Calculate distance / correct course
    current_dist = functions.calculate_distance(current_point,finish_point)
    functions.correct_course_error(course_error_gain,finish_point,previous_point,current_point)

    with open('/sd/log.txt', 'a') as log:
        log.write('Distance to Goal: {}\n'.format(current_dist))

    pyb.delay(5000)

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

