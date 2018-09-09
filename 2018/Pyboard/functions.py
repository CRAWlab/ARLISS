'''functions.py
    ARLISS 2017
    This file provides general functionality for the Tank type rover
    Created: 08/28/2017
    Created by: Joseph Fuentes -jaf1036@louisiana.edu

    Must include the following files for full functionality of code
    'pyboard_razor_IMU.py'
    'micropyGPS.py'
    'motor.py'
    'pyboard_PID.py'


'''
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
from configurations import *

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
extint.disable() # Disabling interrupt so other high process functions can operate correctly without interrupt

def get_altitude():
    extint.enable() # Activating interuppt so new data from GPS can be processed
    global new_data
    global black_rock_alt_m
    global alt_threshold
    while 1:
        # Update the GPS Object when flag is tripped
        if new_data:
            while my_gps_uart.any():
                my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
            altitude = my_gps.altitude # Grabbing parameter designated by micropyGPS object

            if int(altitude) != 0:
                current_altitude = altitude
                new_data = False #clear flag
                break
    extint.disable()  # Disabling interrupt so other high process functions can operate correctly without interrupt
    return current_altitude
def monitor_descent():
    timer = pyb.millis()
    extint.enable() # Activating interuppt so new data from GPS can be processed
    global new_data
    global black_rock_alt_m
    global alt_threshold
    global force_start_timer
    while 1:

    # Update the GPS Object when flag is tripped
        if new_data:
            while my_gps_uart.any():

                my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally
            current_altitude = my_gps.altitude # Grabbing parameter designated by micropyGPS object
            pyb.delay(3000)
            print(current_altitude)
            if int(current_altitude) != 0:
                continue

            if int(current_altitude) < black_rock_alt_m + alt_threshold:
                new_data = False #clear flag
                break
      # Disabling interrupt so other high process functions can operate correctly without interrupt
            elif elapsed_millis(timer) > force_start_timer :
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

    extint.enable() # Activating interuppt so new data from GPS can be processed
    global new_data

    while 1:
        if new_data:
            while my_gps_uart.any():
                my_gps.update(chr(my_gps_uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally

            lat = my_gps.latitude
            lon = my_gps.longitude
            converted_lat = convert_latitude(lat) # Converting Lat (dd.mmss)
            converted_lon = convert_longitude(lon) # Converting Lon (dd.mmss
            point = (converted_lat, converted_lon) # Creating single variable for utilization in calculations
            pyb.delay(3000)
            print(point)
            if int(converted_lat) != 0: # Since converted lat is still in a string it needs to be converted to a integer for comparison
                location = point
                new_data = False
                break
    extint.disable()  # Disabling interrupt so other high process functions can operate correctly without interrupt
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
    motor_A.start(speed, 'ccw')
    motor_B.start(speed, 'cw')

def move_backward(speed):

    #Move both wheels at same speed in sopposite directions to move backward
    motor_A.start(speed, 'cw')
    motor_B.start(speed, 'ccw')

def speed_change(speed):
    # Change Speed to desired PWM
    motor_A.set_speed(speed)
    motor_B.set_speed(speed)

def stop():
    motor_A.stop()
    motor_B.stop()

def hard_stop():
    motor_A.hard_stop()
    motor_B.hard_stop()

def turn_left(speed):
    # Open loop turning used for quick turning or where precision is irrelevant
    motor_A.start(speed,'cw ')
    motor_B.start(speed,'cw')

def turn_right(speed):
    # Open loop turning used for quick turning or where precision is irrelevant
    motor_A.start(speed,'ccw')
    motor_B.start(speed,'ccw')

def motor_accel(speed, time):
    for x in range(1, speed+1):
        motor_A.set_speed(x)
        motor_B.set_speed(x)
        time.sleep_ms(time)

def motor_deccel(speed, time):
    for x in range(1,speed+1):
        motor_A.set_speed(speed-x)
        motor_B.set_speed(speed-x)
        time.sleep_ms(time)

def stuck():
    turn_degree(90, 'right', 2)
    move_forward(100)
    turn_degree(90, 'left', 2)
    move_forward(100)
    stop()

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
    kp_motor = 1.0
    ki_motor = 0
    kd_motor = 1.2

    # Creating PID object for each encoder
    A_pid = PID(kp_motor, ki_motor, kd_motor, 10000, 0.5, -0.5)
    B_pid = PID(kp_motor, ki_motor, kd_motor, 10000, 0.5, -0.5)

    while True:
        #print('Desired: {:.4f}'.format(desired_velocity))
        #Calculating actual angular velocity
        A_velocity = calculate_ang_velocity(enc_timer_A)
        B_velocity = calculate_ang_velocity(enc_timer_B)

      #  print('A vel: {:.4f}'.format(A_velocity))
     #   print('B vel: {:.4f}'.format(B_velocity))
        # Computing error of desired vs. actual
        A_correction= A_pid.compute_output(desired_velocity, A_velocity)
        B_correction= B_pid.compute_output(desired_velocity, B_velocity)

        #print('A correction: {:.4f}'.format(A_correction))
        #print('B correction: {:.4f}'.format(B_correction))

        corrected_velocity_A = desired_velocity + A_correction
        corrected_velocity_B = desired_velocity + B_correction
        #print('A corrected vel: {:.4f}'.format(corrected_velocity_A))
        #print('B corrected vel: {:.4f}'.format(corrected_velocity_B))

        #Mapping correction to the appropriate PWM
        conversion_A = arduino_map(corrected_velocity_A, 0, 3, 0, 100)
        conversion_B = arduino_map(corrected_velocity_B, 0, 3, 0, 100)
        #print('A conversion: {:.4f}'.format(conversion_A))
        #print('B conversion: {:.4f}'.format(conversion_B))
        pyb.delay(10)
        # Changing speed of motors to correction
        motor_A.set_speed(speed + conversion_A)
        motor_B.set_speed(speed + conversion_B)

        with open('/sd/log.txt', 'a') as log:
    		log.write('A vel: {:.4f}'.format(A_velocity))
    		log.write('B vel: {:.4f}'.format(B_velocity))
    		log.write('A correction: {:.4f}'.format(A_correction))
    		log.write('B correction: {:.4f}'.format(B_correction))
    		log.write('A conversion: {:.4f}'.format(conversion_A))
    		log.write('B conversion: {:.4f}'.format(conversion_B))

		if pyb.elapsed_millis(run_time) > duration: # Condition to end function
			motor_A.set_speed(0)
			motor_B.set_speed(0)
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
        motor_A.start(speed, 'ccw')
        pyb.delay(time_to_rotate)
        motor_A.set_speed(0)

    # This is a left turn
    elif turn_direction ==-1:
        motor_B.start(speed, 'cw')
        pyb.delay(time_to_rotate)
        motor_B.set_speed(0)

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
    if turn_direction == 1:
        motor_A.start(speed, 'ccw')
        pyb.delay(abs(int(time_to_rotate)))
        motor_A.stop()
    # This is a left turn
    elif turn_direction == -1:
        motor_B.start(speed, 'cw')
        pyb.delay(abs(int(time_to_rotate)))
        motor_B.stop()
def turn(angle,direction):
    initial_read = razor_imu.get_one_frame()
    initial_angle = abs(int(initial_read[0]))
    sensitivity = 5
    while True:
        pyb.delay(100)
        new_read = razor_imu.get_one_frame()
        new_angle = int(new_read[0])

        if new_angle < 0:
            new_angle+= 360
        elif new_angle > 0:
            continue

        if turn_direction == 1:
            motor_A.start(40, 'ccw')
            if int(angle) > abs( (initial_angle - new_angle ) + sensitivity):
                motor_A.set_speed(0)
                break
        elif turn_direction == -1:
            motor_B.start(40, 'cw')
            if int(angle) > abs( (initial_angle - new_angle ) + sensitivity):
                motor_B.set_speed(0)
                break
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
    motor_A.start(speed,'ccw')
    motor_B.start(speed,'cw')
    while True:
        pyb.delay(100)
        new_read = razor_imu.get_one_frame()
        new_angle = new_read[0]

        if abs(new_angle) > abs((initial_angle - sensitivity)): #The tank is drifting right
            angle_correction = pid_IMU.compute_output(initial_angle,new_angle)
            speed_correction= self.arduino_map(angle_correction, IMU_min, IMU_max, 0, 100)
            current_speedA = motor_A.currentSpeed
            current_speedB = motor_B.currentSpeed
            motor_A.set_speed(current_speedA - speed_correction)
            motor_B.set_speed(current_speedB + speed_correction)
            continue

        elif abs(new_angle) < abs((initial_angle - sensitivity)): #The tank is drifting left
            angle_correction = pid_IMU.compute_output(initial_angle,new_angle)
            speed_correction = self.arduino_map(angle_correction, IMU_min, IMU_max, 0, 100)
            current_speedA = motor_A.currentSpeed
            current_speedB = motor_B.currentSpeed
            motor_A.set_speed(current_speedA + speed_correction)
            motor_B.set_speed(current_speedB - speed_correction)
            continue

        elif pyb.elapsed_millis(run_time) > duration: # Condition to end function
            motor_A.stop()
            motor_B.stop()
            break

        else:
            break

################# End IMU related Functions ######################
