#!/bin/sh

#  functions.py
#  
#
#  Created by Joseph Fuentes on 8/28/17.
#

#################### Import Libraries#######################
import pyb
import machine
from pyb import UART
from pyboard_razor_IMU import Razor
from pyb import Pin
from pyb import Timer
from micropyGPS import MicropyGPS
from motor import motor
from pyboard_PID import PID
import math
from pyb import ExtInt
################## Peripherial Setup ######################

########### Sparkfun Razor IMU set up ##
# UART 1
# Baudrate 57600 bps, 1 stop bit, no parity
# Buffer size very large (1000 chars) to accommodate all incoming characters
razor_imu = Razor(1,57600, read_buf_len = 1000)

################ Xbee Setup ########################
#UART 2
#Baudrate 115200 bps, 1 stop bit, no parity
xbee = UART(2, 115200)

################ GPS set up ########################
# UART 3
# Baudrate is 9600bps, with the standard 8 bits, 1 stop bit, no parity
# Buffer size very large (1000 chars) to accommodate all incoming characters
# each second
my_gps_uart = UART(3,9600,read_buf_len=1000)
# Instantiate the micropyGPS object
# Changing Local offset to Blackrock time -7
my_gps = MicropyGPS()


########### Quadrature encoder set up ##############
# Pin(Board Pin, Alternate function, Pull up resistor enabled, changing Alternate function to use for encoder channel)
enc_A_chan_A = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2) #Timer 2 CH1
enc_A_chan_B = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2) #Timer 2 CH2
enc_B_chan_A = pyb.Pin('Y1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF3_TIM8) #Timer 8 CH1
enc_B_chan_B = pyb.Pin('Y2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF3_TIM8) #Timer 8 CH2

# Putting the Pyboard Channels into encoder mode
enc_timer_A = pyb.Timer(2, prescaler=0, period = 65535)
enc_timer_B = pyb.Timer(8, prescaler=0, period = 65535)
encoder_A= enc_timer_A.channel(1, pyb.Timer.ENC_AB)
encoder_B = enc_timer_B.channel(2, pyb.Timer.ENC_AB)

################# Motor Set up #####################
#Motors:
DIRA = 'Y5'
PWMA = 'Y3'
TIMA = 10
CHANA = 1

DIRB = 'Y6'
PWMB = 'Y4'
TIMB = 11
CHANB = 1

#Motor Object:
motorA = motor(PWMA, DIRA, TIMA, CHANA)
motorB = motor(PWMB, DIRB, TIMB, CHANB)

####### Relay setup ########
# Used for burning parachute
# Pin Y11
# Output Pin
relay = Pin('Y11', Pin.OUT_PP)

################## End Peripheral set up #####################

########### Global Variables ##############

# Variables Relevant to GPS functionality
EARTH_RADIUS = 6370000
MAG_LAT = 82.7
MAG_LON = -114.4

direction_names = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S",
                   "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]

directions_num = len(direction_names)
directions_step = 360 / directions_num
distance = 0

bearing = 0
course_error = 0
turn_direction = 0
current_heading = 0
desired_heading = 0

# Variables Related to wheel movement
wheel_separation = 5.275 # Separation of tracks [inches]
wheel_radius =0.875 # radius of tracks [inches]
gain = 0.75 # adjustable value to account for inertial effects
course_error_gain = 2

################### Defining Functions ####################

def arduino_map(x, in_min, in_max, out_min, out_max):
    '''This function takes value x set to one range and maps it to a relevant range. Typically used for correction with PID'''
    
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def burn_parachute(burn_time):
    
    '''The parachute is wired to the Normally opened position of the relay the connected to ground. Once the output pin is set to high this will trigger the relay thus completing the circuit to generate heat to burn the parachute
        Created By: Joseph Fuentes - jaf1036@louisiana.edu 08/09/2017'''
    
    relay.high() # Set high to activate the relay
    pyb.delay(burn_time) # Determined through testing
    relay.low() # Set low to deactive the Relay

########## GPS Related Functions ###########################
new_data = False

def pps_callback(line):
    '''The Adafruit GPS has a PPS pin that changes from high to low only when we are recieving data
        we will use this to our advantage by associating it with an iterrupt to change indicate we are recieving new data'''
    
    global new_data # Use Global to trigger update
    new_data = True # Raise flag


# Create an external interrupt on pin X8
pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, pps_callback)
extint.disable()

def monitor_descent():
    extint.enable()
    global new_data
    while 1:
    # Update the GPS Object when flag is tripped
        if new_data:
            while my_gps_uart.any():
                my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
            current_altitude = my_gps.altitude # Grabbing parameter designated by micropyGPS object
            if int(current_altitude) != 0:
                continue
        
            if int(current_altitude) < black_rock_alt_m + alt_threshold:
                new_data = False #clear flag
                break
    extint.disable()
    return current_altitude



def convert_latitude(lat_NS):
    """ Function to convert deg m N/S latitude to DD.dddd (decimal degrees)
        Arguments:
        lat_NS : tuple representing latitude
        in format of MicroGPS gps.latitude
        Returns:
        float representing latitidue in DD.dddd
        Created By: Dr. Joshua Vaughan - joshua.vaughan@louisiana.edu
        """
    
    return (lat_NS[0] + lat_NS[1] / 60) * (1.0 if lat_NS[2] == 'N' else -1.0)

def convert_longitude(long_EW):
    """ Function to convert deg m E/W longitude to DD.dddd (decimal degrees)
        Arguments:
        long_EW : tuple representing longitude
        in format of MicroGPS gps.longitude
        Returns:
        float representing longtidue in DD.dddd
        Created By: Dr. Joshua Vaughan - joshua.vaughan@louisiana.edu
        """
    
    return (long_EW[0] + long_EW[1] / 60) * (1.0 if long_EW[2] == 'E' else -1.0)

def get_location():
    extint.enable()
    global new_data
    
    while 1:
        if new_data:
            while my_gps_uart.any():
                my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
            
            lat = my_gps.latitude
            lon = my_gps.longitude
            converted_lat = convert_latitude(lat) # Grabbing parameter designated by micropyGPS object
            converted_lon = convert_longitude(lon) # Grabbing parameter designated by micropyGPS object
            point = (converted_lat, converted_lon) # Creating single variable for utilization in calculations
            pyb.delay(3000)
            
            if int(converted_lat) != 0:
                location = point
                new_data = False
                break
    extint.disable()
    return location

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
    global EARTH_RADIUS
    global distance
    lat1, long1 = position1
    lat2, long2 = position2
    lat1 = math.radians(lat1)
    long1 = math.radians(long1)
    lat2 = math.radians(lat2)
    long2 = math.radians(long2)
    
    dLat = lat2 - lat1
    dLon = long2 - long1
    
    x = dLon * math.cos((lat1+lat2)/2)
    distance = math.sqrt(x**2 + dLat**2) * EARTH_RADIUS
    return distance

def calculate_bearing(position1, position2):
    ''' Calculate the bearing between two GPS coordinates
        Equations from: http://www.movable-type.co.uk/scripts/latlong.html
        Input arguments:
        position1 = lat/long pair in decimal degrees DD.dddddd
        position2 = lat/long pair in decimal degrees DD.dddddd
        Returns:
        bearing = initial bearing from position 1 to position 2 in degrees
        Created: Dr. Joshua Vaughan - joshua.vaughan@louisiana.edu
        Modified:
        *
        '''
    
    global bearing
    lat1 = math.radians(position1[0])
    long1 = math.radians(position1[1])
    lat2 = math.radians(position2[0])
    long2 = math.radians(position2[1])
    dLon = long2 - long1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
    
    return bearing

def bearing_difference(finish, previous, current):
    """
        This function take two points and determines the bearing between them,
        then determines how far to turn the right wheel to correct for the
        error.
        Created By: ARLISS 2015 Team
        """
    global course_error
    global turn_direction
    global current_heading
    global desired_heading
    
    current_heading = calculate_bearing(previous, current)
    desired_heading = calculate_bearing(previous, finish)
    course_error = int(desired_heading - current_heading)
    
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

########## End GPS Related Functions ###########################

################# Motor / Encoder Related Functions ###################
def move_forward(speed):
    #Move both wheels at same speed in opposites direction to move forward
    motorA.start(speed, 'ccw')
    motorB.start(speed, 'cw')

def move_backward(speed):
    
    #Move both wheels at same speed in sopposite directions to move backward
    motorA.start(speed, 'cw')
    motorB.start(speed, 'ccw')

def speed_change(speed):
    
    # Change Speed to desired PWM
    motorA.set_speed(speed)
    motorB.set_speed(speed)

def stop():
    motorA.stop()
    motorB.stop()

def hard_stop():
    motorA.hard_stop()
    motorB.hard_stop()

def turn_left(speed):
    # Open loop turning used for quick turning or where precision is irrelevant
    motorA.start(speed,'cw ')
    motorB.start(speed,'cw')

def turn_right(speed):
    # Open loop turning used for quick turning or where precision is irrelevant
    motorA.start(speed,'ccw')
    motorB.start(speed,'ccw')

def calculate_ang_velocity(encoder_timer):
    '''Uses data from associated encoder calculates difference in count over a sample time to output the angular velocity of the motor. 
        Created By: Joseph Fuentes - jaf1036@louisiana.edu 08/09/2017'''
    cpr = 2256 # Counts per revolution based on specifications and gear ratio of motor
    start = pyb.millis() # Begin sample time
    initial_count = encoder_timer.counter() # Grabbing initial count of encoder
    sample_time = pyb.delay(10) # Delay small but significant for sample time
    '''Since the  maximum rps is 3.5 with 2249 cpr that equates to 7.87 counts per ms maximum allowing for 50ms to be sufficient'''
    
    # velocity in counts per second
    ang_velocity_cps = 1000*(abs(encoder_timer.counter()-initial_count))/(pyb.elapsed_millis(start))
    
    #velocity in revolutions per second
    ang_velocity_revps = ang_velocity_cps/cpr
    
    return ang_velocity_revps

def cruise_control(duration, speed):
    '''Use encoder data to calulate actual angular velocity of motors based on input PWM. Then compares this actual angular velocity to a desired velocity and maps it to the appropriate PWM correction. Once the run time of the cruise control algorithm exceeds the desired duration of the function then it will stop the motors and end proceedure.
        Like a normal cruise control it is intended for use only while moving forward.....
        Created By: Joseph Fuentes - jaf1036@louisiana.edu 08/09/2017'''
    
    # Maximum possible revolution per second of motor = 3.5 [210rpm]
    
    run_time = pyb.millis() # Start timer for the run time of the cruise control
    # Duration: Desired duration of the cruise control [ms]
    # Speed: Desired speed, value of PWM
    
    desired_velocity = arduino_map(speed, 0, 100, 0, 3) #Mapping desired PWM to angular velocity
    
    # PID values [NEEDS TUNING!!!!!]
    kp_motor = 1.5
    ki_motor = 0.003
    kd_motor = 2
    
    # Creating PID object for each encoder
    A_pid = PID(kp_motor, ki_motor, kd_motor, 100000, 3, 0)
    B_pid = PID(kp_motor, ki_motor, kd_motor, 100000, 3, 0)
    
    while True:
        #Calculating actual angular velocity
        A_velocity = calculate_ang_velocity(enc_timer_A)
        B_velocity = calculate_ang_velocity(enc_timer_B)
        
        # Computing error of desired vs. actual
        A_correction= A_pid.compute_output(desired_velocity, A_velocity)
        B_correction= B_pid.compute_output(desired_velocity, B_velocity)
        
        #Mapping correction to the appropriate PWM
        conversion_A = arduino_map(A_correction, 0, 3, 0, 100)
        conversion_B = arduino_map(abs(B_correction), 0, 3, 0, 100)
        
        # Changing speed of motors to correction
        motorA.set_speed(abs(int(conversion_A)))
        motorB.set_speed(abs(int(conversion_B)))
        
        if pyb.elapsed_millis(run_time) > duration: # Condition to end function
            motorA.set_speed(0)
            motorB.set_speed(0)
            break

def correct_course_error(finish , previous , current):
    """
        This function takes the angle and issues a timed command to one motor to
        rotate the rover to a specific angle. Angle must be in degrees; Direction is
        right or left: Right = 1     Left = -1
        Gain is used to tweak values to compensate for inertial effects [gain>0]
        Created By: Joseph Fuentes - jaf1036@louisiana.edu 08/09/2017
        """
    global course_error
    global turn_direction
    global wheel_separation
    global wheel_radius
    global course_error_gain
    
    bearing_difference(finish,previous,current)
    angle = math.radians(course_error)
    
    speed = 40 # This PWM speed is approximately 1 revolution per second can be perfected with gain changes
    number_of_revolutions = (wheel_separation * angle) / (2 * math.pi * wheel_radius)
    
    one_rev_time = 1
    # work with the gain to make the rover turn correctly
    time_to_rotate = int(abs((number_of_revolutions * one_rev_time) * gain * 1000 ))# Multiply by 1000 to put in milliseconds
    print(time_to_rotate)
    
    # This is a right turn.
    if turn_direction == 1:
        motorA.start(speed, 'ccw')
        pyb.delay(time_to_rotate)
        motorA.set_speed(0)
    
    # This is a left turn
    elif turn_direction ==-1:
        motorB.start(speed, 'cw')
        pyb.delay(time_to_rotate)
        motorB.set_speed(0)

def turn_degree(angle,direction,gain):
    
    '''This function allows the user to to turn to a specified degree in the direction they choose.
        Gain is used to tweak values to compensate for inertial effects [gain>0]
        Created By: Joseph Fuentes - jaf1036@louisiana.edu 08/09/2017'''
    
    global wheel_separation
    global wheel_radius

    angle = math.radians(angle)
    turn_direction = direction # Must be 'left' or 'right'
    speed = 40 # This PWM speed is approximately 1 revolution per second can be perfected with gain changes
    number_of_revolutions = (wheel_separation * angle) / (2 * math.pi * wheel_radius)
    
    one_rev_time = 1
    # work with the gain to make the rover turn correctly
    time_to_rotate = (number_of_revolutions * one_rev_time) * gain * 1000 # Multiply by 1000 to put in milliseconds
    
    # This is a right turn.
    if turn_direction == 'right':
        motorA.start(speed, 'CCW')
        pyb.delay(abs(time_to_rotate))
        motorA.stop()
    # This is a left turn
    elif turn_direction == 'left':
        motorB.start(speed, 'CW')
        pyb.delay(abs(time_to_rotate))
        motorB.stop()


################# End motor Related Functions ###################

################# IMU related Functions #########################

def IMU_read(self):
    angle = razor_imu.get_one_frame()
    yaw, pitch, roll = angle
    return yaw, pitch, roll

def imu_pid(self,duration,speed):
    '''This functions utilizes the constant streaming function of the Razor IMU refer to 'pyboard_razor_IMU.py' for details. The function takes data from the IMU the stores it as an initial angle then uses changes in yaw to correct inconsistencies in motor speed'''
   
    run_time = pyb.millis() # Start timer for the run time of the cruise control
    duration = self.duration # Desired duration of the cruise control [ms]
    speed = self.speed # Desired speed, value of PWM


    # PID values [NEEDS TUNING!!!!!]
    kp_IMU  = 2
    ki_IMU  = 0
    kd_IMU  = 1
    dt = 10000
    
    # Max / Min is limited to the object being influenced in this case motor PWM
    max_out = 90
    min_out = 0
    
    # Creating PID object
    IMU_pid = PID(kp_IMU, ki_IMU, kd_IMU, dt, max_out, min_out)
    
    # Max / Min output angle produced by the IMU
    IMU_min =-180
    IMU_max = 180
    
    # Included due to 'shake' in IMU
    sensitivity = 3
    
    # Establishing initial yaw angle
    initial_read = razor_imu.get_one_frame()
    initial_angle = initial_read[0] # Only interested in yaw
    motorA.start(speed,'ccw')
    motorB.start(speed,'cw')
    while True:
        pyb.delay(100)
        new_read = razor_imu.get_one_frame()
        new_angle = new_read[0]
    
        if abs(new_angle) > abs((initial_angle - sensitivity)): #The tank is drifting right
            angle_correction = pid_IMU.compute_output(initial_angle,new_angle)
            speed_correction= self.arduino_map(angle_correction, IMU_min, IMU_max, 0, 100)
            current_speedA = motorA.currentSpeed
            current_speedB = motorB.currentSpeed
            motorA.set_speed(current_speedA - speed_correction)
            motorB.set_speed(current_speedB + speed_correction)
            continue

        elif abs(new_angle) < abs((initial_angle - sensitivity)): #The tank is drifting left
            angle_correction = pid_IMU.compute_output(initial_angle,new_angle)
            speed_correction = self.arduino_map(angle_correction, IMU_min, IMU_max, 0, 100)
            current_speedA = motorA.currentSpeed
            current_speedB = motorB.currentSpeed
            motorA.set_speed(current_speedA + speed_correction)
            motorB.set_speed(current_speedB - speed_correction)
            continue

        elif pyb.elapsed_millis(run_time) > duration: # Condition to end function
            motorA.stop()
            motorB.stop()
            break

        else:
            break

################# End IMU related Functions ######################





