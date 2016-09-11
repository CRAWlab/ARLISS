from pyb import Pin, Timer, UART
import motor_class
import utime
import pyboard_PID as pyPID
from micropyGPS import MicropyGPS

# motorA is left motor
# clockwise for the encoder is correct, but the motor and encoder rotate in
# opposite directions

def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

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

my_gps = MicropyGPS()

gps_uart = UART(6, 9600)

# Set up motorA
DIRA = 'X7'
PWMA = 'X8'
TIMA = 14
CHANA = 1
motorA = motor_class.motor(PWMA, DIRA, TIMA, CHANA)

# Set up motorB
DIRB = 'Y4'
PWMB = 'Y3'
TIMB = 10
CHANB = 1
motorB = motor_class.motor(PWMB, DIRB, TIMB, CHANB)

# Set up encoder timers
pin_a = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                af=pyb.Pin.AF1_TIM2)

pin_b = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                af=pyb.Pin.AF1_TIM2)

pin_c = pyb.Pin('X9', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                af=pyb.Pin.AF2_TIM4)

pin_d = pyb.Pin('X10', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                af=pyb.Pin.AF2_TIM4)

enc_timer_A = pyb.Timer(2, prescaler=0, period=65535)
enc_timer_B = pyb.Timer(4, prescaler=0, period=65535)

enc_channel_A = enc_timer_A.channel(1, pyb.Timer.ENC_AB)
enc_channel_B = enc_timer_B.channel(2, pyb.Timer.ENC_AB)

while uart.any():
    my_gps.update(chr(uart.readchar()))
    if my_gps.latitude[0] != 0:
        break

init_lat = convert_latitude(my_gps.latitude)
init_long = convert_longitude(my_gps.longitude)
init_point = (init_lat, init_long)

# Start motorA
# motorA.start(30,'cw')
init_A = enc_timer_A.counter()
total_A = 0
another_A = 0
correction_A = 0
pid_A = 0

# Start motorB
# motorB.start(30,'ccw')
init_B = enc_timer_B.counter()
total_B = 0
another_B = 0
correction_B = 0
pid_B = 0

# Get inital encoderA reading every 0.1s
def encoder_A(timer):
    global init_A
    global total_A
    global correction_A
    current_encoder = enc_timer_A.counter() + (65535 * another_A)
    total_A = (current_encoder - init_A)
    init_A = current_encoder
    print(total_A)

# Add 65535 everytime the encoder is called so it never goes to zero
def plus_A(timer):
    global another_A
    another_A += 1

# Get inital encoderB reading every 0.1s
def encoder_B(timer):
    global init_B
    global total_B
    global correction_B
    current_encoder = enc_timer_B.counter() + (65535 * another_B)
    total_B = (current_encoder - init_B)
    init_B = current_encoder
    print(total_B)

# Add 65535 everytime the encoder is called so it never goes to zero
def plus_B(timer):
    global another_B
    another_B += 1

# Callbacks for the plus 65535 functions
enc_timer_A.callback(plus_A)
enc_timer_B.callback(plus_B)

# Callback for encoder A
tim_call_A = pyb.Timer(12, freq=10)
tim_call_A.callback(encoder_A)

# Callback for encoder A
tim_call_B = pyb.Timer(8, freq=10)
tim_call_B.callback(encoder_B)

# Callback for GPS
GPS_PID_call = pyb.Timer(11, freq=0.5)
GPS_PID_call.callback(GPS_bearing)


new_PID_speed = 0
def GPS_bearing(timer):
    global initial_point
    my_gps.update(chr(uart.readchar()))
    pres_lat = convert_latitude(my_gps.latitude)
    pres_long = convert_longitude(my_gps.longitude)
    pres_point = (pres_lat, pres_long)
    direction_tuple = bearing_difference(initial_point, pres_point)
    angle = direction_tuple[0]
    turn = direction_tuple[1]
    if turn == 1:
        new_PID_speed = 150
    elif turn == -1:
        new_PID_speed = 250
    else:
        new_PID_speed == 200
    initial_point = pres_point
    return new_PID_speed


kp = 1
ki = 0.0000001
kd = 0.01
pid = pyPID.PID(kp, ki, kd, 100000, 712, 135)
sw = pyb.Switch()


while True:
    correction_A = pid.compute_output(new_PID_speed, total_A)
    conversion_A = arduino_map(correction_A, 135, 712, 20, 100)
    motorA.change_speed(conversion_A)

    correction_B = pid.compute_output(new_PID_speed, total_B)
    conversion_B = arduino_map(correction_B, 135, 712, 20, 100)
    motorB.change_speed(conversion_B)
    if sw():
        motorA.stop()
        motorB.stop()
        break
