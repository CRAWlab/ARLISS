from pyb import Pin, Timer, UART
import motor_class
import utime, math
import pyboard_PID as pyPID
from micropyGPS import MicropyGPS
import pyboard_RazorIMU as IMU

def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# target_coordinates = (40.885218,-119.116094)
fake_target = (40.870245, -119.106353)

max_angular_velocity = 10
min_angular_velocity = -10

desired_heading = 0
current_heading = 0

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
    # print(position1[0])
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

def calculate_heading(data):
        ''' function calculates heading assuming X is forward and Z is up '''
        heading = (360.0 - math.atan2(data[1],
                                      data[0]) * 180.0/math.pi) % 360
        return heading

def bearing_difference(past, present):
        """
        This function take two points and determines the bearing between them,
        then determines how far to turn the right wheel to correct for the
        error.
        """
        finish_point = (40.870245, -119.106353)
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

gps_uart = UART(6, 9600, read_buf_len=1000)
xbee_uart = UART(2, 9600)

Razor_IMU = IMU.Razor(3, 57600)
# Razor_IMU.set_angle_output()
Razor_IMU.set_all_calibrated_output()
# while True:
#     a,b,c = Razor_IMU.get_one_frame()
#     print(a,b,c)

# Countdown Timer
start = pyb.millis()
backup_timer = 5400000

# Don't do anything until GPS is found
while gps_uart.any() >= 0:
    my_gps.update(chr(gps_uart.readchar()))
    print('No GPS signal!!!\n')
    print(my_gps.latitude)
    if my_gps.latitude[0] != 0:
        init_lat = convert_latitude(my_gps.latitude)
        init_long = convert_longitude(my_gps.longitude)
        initial_point = (init_lat, init_long)
        print('Initial Point: {}'.format(initial_point))
        break

# initial_point = (40.870242, -119.106354)

# while True:
#     print('not landed yet')
#     if pyb.elapsed_millis(start) >= backup_timer: #This is 90 minutes
#         break

# Parachute
# parachute = Pin('Y5', Pin.OUT_PP)
# parachute.high()
# utime.sleep(2)
# parachute.low()

# Buzzer
# buzzer = Pin('Y12')
# tim = Timer(8, freq=1000)
# ch = tim.channel(3, Timer.PWM, pin=buzzer)
# ch.pulse_width_percent(90)
# utime.sleep(0.5)
# ch.pulse_width_percent(0)
# utime.sleep(0.5)
# ch.pulse_width_percent(90)
# utime.sleep(0.5)
# ch.pulse_width_percent(0)
# utime.sleep(0.5)

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

# Start motorA
motorA.start(30,'cw')
init_A = enc_timer_A.counter()
total_A = 0             # current encoder reading - last encoder reading (counts per 0.1s)
another_A = 0           # running counter to handle integer overflow (counts per 0.1s)
correction_A = 0        # desired counts per 0.1s
pid_A = 0               #

# Start motorB
motorB.start(30,'ccw')
init_B = enc_timer_B.counter()
total_B = 0             # current encoder reading - last encoder reading (counts per 0.1s)
another_B = 0           # running counter to handle integer overflow (counts per 0.1s)
correction_B = 0        # desired counts per 0.1s
pid_B = 0

GPS_read_counter = 0    # counter incremented in interrupt loop, determines when to read GPS
IMU_read_counter = 0    # counter incremented in interrupt loop, determines when to read IMU

kp_IMU = 1
ki_IMU  = 0
kd_IMU  = 0

pidIMU = pyPID.PID(kp_IMU, ki_IMU, kd_IMU, 100000, 712, 110)

# Get inital encoderA reading every 0.1s
def encoder_timer(timer):
    global init_A
    global total_A
    global correction_A

    global init_B
    global total_B
    global correction_B

    global GPS_read_counter
    global IMU_read_counter
    global max_angular_velocity
    global min_angular_velocity

    global desired_heading
    # global current heading

    # get current encoder count
    current_encoder_A = enc_timer_A.counter() + (65535 * another_A)

    total_A = (current_encoder_A - init_A)    # calculate change in encoders since last read
    init_A = current_encoder_A                # save current encoder reading for next loop

    # get current encoder count
    current_encoder_B = enc_timer_B.counter() + (65535 * another_B)

    total_B = (current_encoder_B - init_B)     # calculate change in encoders since last read
    init_B = current_encoder_B                 # save current encoder reading for next loop
    # print(total_B)

    # increment GPS and IMU counters
    GPS_read_counter = GPS_read_counter + 1
    IMU_read_counter = IMU_read_counter + 1
    # print('GPS addition: {}'.format(GPS_addition))
    # yaw, b, c = Razor_IMU.get_one_frame()
    # pidA.compute_output(300, total_A)


# Add 65535 everytime the encoder is called so it never goes to zero
def plus_A(timer):
    global another_A
    another_A += 1

# Get inital encoderB reading every 0.1s
# def encoder_B(timer):
#     global init_B
#     global total_B
#     global correction_B
#
#     # get current encoder count
#     current_encoder = enc_timer_B.counter() + (65535 * another_B)
#
#     total_B = (current_encoder - init_B)     # calculate change in encoders since last read
#     init_B = current_encoder                 # save current encoder reading for next loop
#     # print(total_B)

# Add 65535 everytime the encoder is called so it never goes to zero
def plus_B(timer):
    global another_B
    another_B += 1

heading = 0
mag_array = []
def IMU_read():
    global heading
    #my_gps.update(chr(gps_uart.readchar()))

    # Read the IMU, asking for one frame of data
    Razor_IMU.uart.write('#f')

    # read the first line of the frame returned from the IMU
    data = Razor_IMU.uart.readline()
    print(data)
    if data:
        mag_check = data[0:5].decode('utf-8')
        # print(mag_check)

        if mag_check == '#M-C=':
            length = len(data[5:-1].decode('utf-8'))
            if length != 3:
                mag1 = 0
                mag2 = 0
                mag3 = 0
            else:
                mag1, mag2, mag3 = int(data[5:-1].decode('utf-8'))
        elif mag_check == '#A-C=' or mag_check == '#G-C=':
            # read the next line of the frame
            data = Razor_IMU.uart.readline()
            mag_check = data[0:5].decode('utf-8')
            length = len(data[5:-1].decode('utf-8'))
            if length != 3:
                mag1 = 0
                mag2 = 0
                mag3 = 0
            else:
                mag1, mag2, mag3 = int(data[5:-1].decode('utf-8'))

            # print(mag_check)

            if mag_check == '#M-C=':
                length = len(data[5:-1].decode('utf-8'))
                if length != 3:
                    mag1 = 0
                    mag2 = 0
                    mag3 = 0
                else:
                    mag1, mag2, mag3 = int(data[5:-1].decode('utf-8'))
            elif mag_check == '#A-C=' or mag_check == '#G-C=':
                data = Razor_IMU.uart.readline()
                mag_check = data[0:5].decode('utf-8')
                length = len(data[5:-1].decode('utf-8'))
                if length != 3:
                    mag1 = 0
                    mag2 = 0
                    mag3 = 0
                else:
                    mag1, mag2, mag3 = int(data[5:-1].decode('utf-8'))

                # print(mag_check)
                if mag_check == '#M-C=':
                    length = len(data[5:-1].decode('utf-8'))
                    if length != 3:
                        mag1 = 0
                        mag2 = 0
                        mag3 = 0
                    else:
                        mag1, mag2, mag3 = int(data[5:-1].decode('utf-8'))
        else:
            mag1 = 0
            mag2 = 0
            mag3 = 0
    else:
        mag1 = 0
        mag2 = 0
        mag3 = 0
    heading = calculate_heading([mag1, mag2, mag3])
    print(calculate_heading([mag1, mag2, mag3]))
    return calculate_heading([mag1, mag2, mag3])

    # print('yaw:{}'.format(yaw))
    IMU_read_counter = 0

bearing = 0
heading = 0
target_distance = 0
new_PID_speed = 0

def GPS_bearing():
    # print('In the GPS function')
    global new_PID_speed
    global initial_point
    global bearing
    global heading

    while gps_uart.any():
        my_gps.update(chr(gps_uart.readchar()))

    pres_lat = convert_latitude(my_gps.latitude)
    pres_long = convert_longitude(my_gps.longitude)
    pres_point = (pres_lat, pres_long)

    course_error, turn_direction, current_heading, desired_heading = bearing_difference(initial_point, pres_point)
    # print('course_error {}, turn_direction {}, current_heading {}, desired_heading {}'.format(course_error, turn_direction, current_heading, desired_heading))
    target_distance = calculate_distance(pres_point, initial_point)
    GPS_read_counter = 0
    # yaw,b,c = Razor_IMU.get_one_frame()

    # if turn_direction == 1:
    #     print('Turn Right\n')
    #     new_PID_speed = 200
    # elif turn_direction == -1:
    #     print('Turn Left\n')
    #     new_PID_speed = 100
    # else:
    #     new_PID_speed == 150
    #     print('Stay Straight\n')
    # initial_point = pres_point
    # GPS_addition = 0
    # return new_PID_speed

# Callbacks for the plus 65535 functions
enc_timer_A.callback(plus_A)
enc_timer_B.callback(plus_B)

# Callback for encoder A
tim_call_A = pyb.Timer(12, freq=10)
tim_call_A.callback(encoder_timer)

# Callback for encoder B
# tim_call_B = pyb.Timer(8, freq=10)
# tim_call_B.callback(encoder_B)

# Callback for GPS
# GPS_PID_call = pyb.Timer(11, freq=1)
# GPS_PID_call.callback(GPS_bearing)

# gains for low-level, motor-speed PID controller
kp_motor = 0.4
ki_motor = 0.00000001
kd_motor = 2

# g_kp = 1.3
# g_ki = 0.0000001
# g_kd = 5

pidA = pyPID.PID(kp_motor, ki_motor, kd_motor, 100000, 712, 110)
pidB = pyPID.PID(kp_motor, ki_motor, kd_motor, 100000, 712, 110)

# GPS_pid = pyPID.PID(g_kp, g_ki, g_kd, 100000, 180, 0)
sw = pyb.Switch()

while True:
    # if the interrupt has been called 10 times, read the GPS
    if GPS_read_counter >= 10:
        GPS_bearing()

    # if the interrupt has been called 1 time, read the heading
    if IMU_read_counter >= 1:
        IMU_read()

    # calculate new motor speed based on desired counts per 0.1s determined
    # by the navigation algorithm
    correction_A = pidA.compute_output(150, total_A)
    conversion_A = arduino_map(correction_A, 110, 563, 20, 100)
    motorA.change_speed(conversion_A)

    # calculate new motor speed based on desired counts per 0.1s determined
    # by the navigation algorithm
    correction_B = pidB.compute_output(150, total_B)
    conversion_B = arduino_map(correction_B, 110, 587, 20, 100)
    motorB.change_speed(conversion_B)

    # read the status of the onboard switch, stop the motors if pressed
    # used for testing
    if sw():
        motorA.stop()
        motorB.stop()
        break
