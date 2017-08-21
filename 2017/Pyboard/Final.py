"""
File: Final.py
ARLISS COMPETITION 2017
TANK DESIGN
Description: This is a program that will use a gps to control a rover to reach
a predetermined destination in the ARLISS compeition. The code will operate as follows:
    1. Rover lands on ground, determines orientation, get GPS coordinate.
    2. Rover drives straight ahead for 30s.
    3. Rover stops, takes another GPS reading.
    4. Rover determines bearing between first GPS point and finish point.
    5. Rover determines bearing between second GPS point and first point.
    6. Rover calculates the difference in degrees between the two bearings.
    7. Rover determines if turn is closer to right or left.
    8. Rover keeps one wheel still and rotates the other according to linear
    regression.
    (linear regression-- corrilates the time the wheel is rotating with rover
    direction)
    9. Rover calculates the distance from current point to finish point.
    10. If distance is larger than allowed by contest, begin looping from
    beginning.
    Files Needed: micropyGPS.py and motor.py

########## PINS ###################

Motors/ Drivers:
A=Left Track
    A_EN= 'Y4'
    A_PH= 'Y5'
B=Right Track
    B_EN= 'Y6'
    B_PH= 'Y7'

Encoder:
A=Left Track
enc_A_chan_A='X1'
enc_A_chan_B='X2'
B=Right Track
enc_B_chan_A='X9'
enc_B_chan_B='X10'
GPS:
-Top GPS:
    UART 3
    Baudrate=9600
-Bottom GPS:
    UART 6 
    Baudrate=9600

IMU: 
Internal 

XBEE:
UART 2
Baudrate=115200

Relay = 'Y11'

"""

import pyb
from pyb import UART
from pyb import ExtInt
from pyb import Pin
from micropyGPS import MicropyGPS
import motor
import time
import math


# FINISH POINT MUST BE DEFINED IN BEARING_DIFFERENCE ALSO!!!!
# finish_point = (40.849421, -119.122892)
# finish_point = (30.209884, -92.0217968)
# finish_point = (40.86339, -119.1344)
finish_point =

# LAUNCH POINT
# fill in with launch coordinate; can be programmed in before launch
# launch_point = (40.849421, -119.122892)
# launch_point = (30.209884, -92.0217968)
launch_point =



######Pin Assignment#############
xbee = UART(2, 115200)
top_gps_uart = UART(3,9600)
bot_gps_uart = UART(6,9600)

enc_A_chan_A = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_A_chan_B = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_B_chan_A = pyb.Pin('X9', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)
enc_B_chan_B = pyb.Pin('X10', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)

#Motors:
DIRA = 'Y5'
PWMA = 'Y4'
TIMA = 14
CHANA = 1

DIRB = 'Y8'
PWMB = 'Y6'
TIMB = 12
CHANB = 1

motorA = motor(PWMA, DIRA, TIMA, CHANA)
motorB = motor(PWMB, DIRB, TIMB, CHANB)

#Stopping Point 08/20/2017 -Joseph Fuentes

# define variables for landing check
backup_timer = 5400000
altitude_concurrent_timer = 3600000
acceptable_dist_from_launch = 1000
acceptable_altitude_change = 25

# Global Flag to start GPS data Processing
new_data = False



start = pyb.millis()

# Callback Function
def pps_callback(line):
        # print("Updated GPS Object...")
        global new_data  # Use Global to trigger update
        new_data = True


# Instantiate the micropyGPS object
my_gps = MicropyGPS()

# Setup the connection to your GPS here
uart = UART(6, 9600, read_buf_len=1000)

# Create an external interrupt on pin X8
pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP,
                    pps_callback)

# Create relay object
relay = Pin('Y11', Pin.OUT_PP)

# Setup rtc timer to wake up pyboard every 30 seconds during standby to check
# conditions
# rtc = pyb.RTC()
# rtc.wakeup(30000)




# #######################  GPS functions  #####################################

EARTH_RADIUS = 6370000
MAG_LAT = 82.7
MAG_LON = -114.4

direction_names = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S",
                   "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]

directions_num = len(direction_names)
directions_step = 360 / directions_num


def calculate_bearing(position1, position2):
    ''' Calculate the bearing between two GPS coordinates
    Equations from: http://www.movable-type.co.uk/scripts/latlong.html
    Input arguments:
        position1 = lat/long pair in decimal degrees DD.dddddd
        position2 = lat/long pair in decimal degrees DD.dddddd
    Returns:
        bearing = initial bearing from position 1 to position 2 in degrees
    Created: Joshua Vaughan - joshua.vaughan@louisiana.edu - 04/23/14
    Modified:
        *
    '''
    print(position1[0])
    lat1 = math.radians(position1[0])
    long1 = math.radians(position1[1])
    lat2 = math.radians(position2[0])
    long2 = math.radians(position2[1])

    dLon = long2 - long1

    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

    return bearing


def calculate_distance(position1, position2):
    ''' Calculate the distance between two lat/long coords using simple
        cartesian math

    Equation from: http://www.movable-type.co.uk/scripts/latlong.html

    Input arguments:
        position1 = lat/long pair in decimal degrees DD.dddddd
        position2 = lat/long pair in decimal degrees DD.dddddd

    Returns:
        distance = distance from position 1 to position 2 in meters


    Created: Joshua Vaughan - joshua.vaughan@louisiana.edu - 04/24/14

    Modified:
        * Forrest Montgomery -- the micropython board did not like the radians
        conversion turned into a tuple. So I separated all the lats and longs.
    '''

    R = 6373000        # Radius of the earth in m

    lat1, long1 = position1
    lat2, long2 = position2
    lat1 = math.radians(lat1)
    long1 = math.radians(long1)
    lat2 = math.radians(lat2)
    long2 = math.radians(long2)

    dLat = lat2 - lat1
    dLon = long2 - long1

    x = dLon * math.cos((lat1+lat2)/2)
    distance = math.sqrt(x**2 + dLat**2) * R

    return distance


# converting functions from Dr. Vaughan
def convert_longitude(long_EW):
    """ Function to convert deg m E/W longitude to DD.dddd (decimal degrees)
    Arguments:
      long_EW : tuple representing longitude
                in format of MicroGPS gps.longitude
    Returns:
      float representing longtidue in DD.dddd
    """

    return (long_EW[0] + long_EW[1] / 60) * (1.0 if long_EW[2] == 'E' else -1.0)


def convert_latitude(lat_NS):
    """ Function to convert deg m N/S latitude to DD.dddd (decimal degrees)
    Arguments:
      lat_NS : tuple representing latitude
                in format of MicroGPS gps.latitude
    Returns:
      float representing latitidue in DD.dddd
    """

    return (lat_NS[0] + lat_NS[1] / 60) * (1.0 if lat_NS[2] == 'N' else -1.0)



######################## 2-Wheeled Rover Functions ###############################

'''The following functions utilize the functions created in the motor class
    and provide basic movement functions for the rover.
    
    All spin-directions for motors assume left wheel is motorA and right wheel is motorB
    when looking at top of rover with front facing forward.'''

def move_forward(speed):
    #Move both wheels at same speed in same direction to move forward
    motorA.start(speed, 'CCW')
    motorB.start(speed, 'CW')

def move_backward(speed):
    #Move both wheels at same speed in same direction to move backward
    motorA.start(speed, 'CW')
    motorB.start(speed, 'CCW')

def speed_change(speed):
    motorA.set_speed(speed)

def stop():
    motorA.stop()
    motorB.stop()

def hard_stop():
    motorA.hard_stop()
    motorB.hard_stop()

def one_wheel_pid(init_speed, desired_speed):
    motorA.start(speed, 'CCW')
    kp = (2*np.pi)**2
    ki = 135.0
    kd = 10.0
    pid = PID(kp, ki, kd, 0.001, 100, 0, 0.0)
    pid.compute_output(desired_speed, motorA.currentSpeed)


def stuck():
        """
        This function is called when the rover has not moved 0.5 meters from
        its last position
        """
        motor.move_backward(100)
        time.sleep(2)
        motor.stop()
        motor.stationary_turn(100, 1)
        time.sleep(2)
        motor.stop()
        return


def bearing_difference(past, present):
        """
        This function take two points and determines the bearing between them,
        then determines how far to turn the right wheel to correct for the
        error.
        """
        finish_point = (40.86339, -119.1344)
        current_heading = calculate_bearing(past, present)
        desired_heading = calculate_bearing(present, finish_point)
        course_error = desired_heading - current_heading
        # Correct for heading 'wrap around' to find shortest turn
        if course_error > 180:
            course_error -= 360
        elif course_error < -180:
            course_error += 360
        # record the turn direction - mainly for debugging
        if course_error > 0:
            turn_direction = 1  # Turn right
        elif course_error < 0:
            turn_direction = -1  # Turn left
        else:
            turn_direction = 0  # Stay straight
        return course_error, turn_direction, current_heading, desired_heading




# #######################  Rover Loop  #########################################

# log.write('{},'.format(my_gps.timestamp))
# 'target point: {}'.format(finish_point))
# '\nlaunching at: {}'.format(launch_point))
# with open('/sd/log.csv', 'a') as log:
    # log.write('\n\n-----------------------------\n\n')
xbee.write('\nprogram initiated at: {}'.format(my_gps.timestamp))
xbee.write('\ntarget point: {}'.format(finish_point))
xbee.write('\nlaunching at: {}'.format(launch_point))


# Landing-check loop
sw = pyb.Switch()
while True:
    # Update the GPS Object when flag is tripped
    # if sw():
        # log.close()
        # break
    if new_data:
        while True:
            my_gps.update(chr(uart.readchar()))
        if my_gps.latitude[0] != 0:
            xbee.write('\nChecking GPS and altitude to see if landed...')
            initial_lat = convert_latitude(my_gps.latitude)
            initial_long = convert_longitude(my_gps.longitude)
            initial_point = (initial_lat, initial_long)
            xbee.write('\nCurrent point: {}'.format(initial_point))
            dist_from_launch = calculate_distance(launch_point, initial_point)
            xbee.write('\nDistance from launch: {}'.format(dist_from_launch))
            # acceptable distance is currently set for simple testing
            if dist_from_launch < acceptable_dist_from_launch:
                # pyb.stop()
                pyb.delay(100)
            elif dist_from_launch > acceptable_dist_from_launch:
                altitude_1 = my_gps.altitude
                pyb.delay(10000) # change to 10000 for real launch
                altitude_2 = my_gps.altitude
                altitude_change = abs(altitude_1 - altitude_2)
                xbee.write('\nAltitude change: {}'.format(altitude_change))
                if altitude_change > acceptatble_altitude_change:
                    # pyb.stop()
                    pyb.delay(100)
                elif altitude_change < acceptatble_altitude_change and pyb.elapsed_millis(start) >= altitude_concurrent_timer: # acceptable time yet to be determined
                    break
            elif pyb.elapsed_millis(start) >= backup_timer:
                break
            new_data = False
            past_point2 = None

        else:
            new_data = False
            xbee.write('\nGetting zeros from GPS')
            # pyb.stop() #not sure if needed here?
            pyb.delay(100)

    elif pyb.elapsed_millis(start) >= backup_timer: #This is 90 minutes
        break
    else:
        xbee.write('\nNo new data')
    xbee.write('\nNo data from GPS')
    pyb.delay(500)


xbee.write('\n\nAt {}, I have landed at location: {}'.format(my_gps.timestamp, initial_point))
# log.write('{},'.format(initial_point))

# Breakout from parachute mechanism
relay.high()
pyb.delay(5000)
relay.low()
motor.move_forward(100)
pyb.delay(10000)
motor.stop()

# GPS check loop: exact starting point defined here

while True:
    # Update the GPS Object when flag is tripped
    # if sw():
        # log.close()
        # break
    if new_data:
        while uart.any():
            my_gps.update(chr(uart.readchar()))

        if my_gps.latitude[0] != 0:
            past_lat = convert_latitude(my_gps.latitude)
            past_long = convert_longitude(my_gps.longitude)
            past_point = (past_lat, past_long)
            new_data = False
            past_point2 = None
            break
        else:
            new_data = False

xbee.write('\n\nI will begin navigation at this location: {}'.format(past_point))
# log.write('{},'.format(past_point))


# Navigation loop

while True:
    # if sw():
        # log.close()
        # break
    # log.write('\n\nDriving Foward!')
    # print(my_gps.altitude)
    motor.move_forward(100)
    time.sleep(6)
    motor.stop()
    # log.write("\nStopping!")
    # motor.stop()
    # time.sleep(0.1) # The sleep is for the slow stop; allows for a complete stop

    if new_data:
        while uart.any():
            my_gps.update(chr(uart.readchar()))

        pres_lat = convert_latitude(my_gps.latitude)
        pres_long = convert_longitude(my_gps.longitude)
        present_point = (pres_lat, pres_long)
        # log.write('{},{},{},'.format(my_gps.timestamp, past_point,
        #                             present_point))

        if past_point2 is None:
            direction_tuple = bearing_difference(past_point, present_point)
            angle = direction_tuple[0]
            turn = direction_tuple[1]
            bearing_current = direction_tuple[2]
            bearing_desired = direction_tuple[3]
            L_or_R_angle = angle #* turn
            with open('/sd/log.csv', 'a') as log:
                log.write('\n\n-----------------------------\n\n')
            with open('/sd/log.csv', 'a') as log:
                log.write('{}, {}, {}, {}, {}, {}'.format(my_gps.timestamp,
                                                          past_point,
                                                          present_point,
                                                          bearing_current,
                                                          bearing_desired,
                                                          L_or_R_angle))
            xbee.write('\nCurrent point: {}\nAngle: {}'.format(present_point, angle))
            stuck_distance = calculate_distance(past_point, present_point)
            if stuck_distance < 1:
                xbee.write('\nIm stuck at: {}\nI will try to escape'.format(present_point))
                # log.write('\nI got stuck at {} at: {}\nI will try to escape'.format(present_point))
                stuck() #escape function, defined above in Rover-Functions
                new_data = False
                past_point = present_point
                with open('/sd/log.csv', 'a') as log:
                    log.write(' ,')

            # log.write('\nI will turn {} degrees at {} at: {}'.format(angle, my_gps.timestamp, present_point))
            motor.angle_to_motor_turn(angle, turn)

            target_distance = calculate_distance(finish_point, present_point)
            with open('/sd/log.csv', 'a') as log:
                log.write(', {}\n'.format(target_distance))
            past_point2 = present_point

            if target_distance < 100:
                xbee.write('\n\nArrived at finish point: {}\nArrival time: {}'.format(finish_point, my_gps.timestamp))
                # log.write('\n\nArrived at finish point: {}\nArrival time: {}'.format(finish_point, my_gps.timestamp))
                # log.close()
                break

            new_data = False

        else:
            direction_tuple = bearing_difference(past_point2, present_point)
            angle = direction_tuple[0]
            turn = direction_tuple[1]
            bearing_current = direction_tuple[2]
            bearing_desired = direction_tuple[3]
            L_or_R_angle = angle #* turn
            with open('/sd/log.csv', 'a') as log:
                log.write('\n\n-----------------------------\n\n')
            with open('/sd/log.csv', 'a') as log:
                log.write('{}, {}, {}, {}, {}, {}'.format(my_gps.timestamp,
                                                          past_point2,
                                                          present_point,
                                                          bearing_current,
                                                          bearing_desired,
                                                          L_or_R_angle))
            xbee.write('\nCurrent point: {}\nAngle: {}'.format(present_point, angle))
            stuck_distance = calculate_distance(past_point2, present_point)
            if stuck_distance < 1:
                xbee.write('\nIm stuck at: {}\nI will try to escape'.format(present_point))
                # log.write('\nIm stuck at: {}\nI will try to escape'.format(present_point))
                stuck() # escape function, defined above in Rover-Functions
                new_data = False
                past_point2 = present_point
                with open('/sd/log.csv', 'a') as log:
                    log.write(' ,')

            # log.write('\nI will turn {} degrees at {} at: {}'.format(angle, my_gps.timestamp, present_point))
            motor.angle_to_motor_turn(angle, turn)

            target_distance = calculate_distance(finish_point, present_point)
            xbee.write('\nTarget Distance: {}'.format(target_distance))
            with open('/sd/log.csv', 'a') as log:
                log.write(', {}\n'.format(target_distance))
            past_point2 = present_point

            if target_distance < 100:
                xbee.write('\n\nArrived at finish point: {}\nArrival time: {}'.format(finish_point, my_gps.timestamp))
                # log.write('\n\nArrived at finish point: {}\nArrival time: {}'.format(finish_point, my_gps.timestamp))
                # log.close()
                break

            new_data = False
