################## Configurations File ########################
################## Imports ##########################
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

################## User Input #####################
finish_point = (30.2107, -92.0209)
distance_tolerance = 3
wheel_separation = 8
wheel_radius = 2.71
gain = 0.75
course_error_gain = 2



################### END USER INPUT ################

###################### GPS SETUP ######################
my_gps_uart = UART(6, 9600, read_buf_len=1000)
my_gps = MicropyGPS(-7)

####################### MOTOR SETUP #######################
enc_A_chan_A = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_A_chan_B = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_timer_A = pyb.Timer(2, prescaler = 0, period = 65535)
encoder_A = enc_timer_A.channel(1, pyb.Timer.ENC_AB)
DIR_A = 'X7'
PWM_A = 'X8'
TIM_A = 14
CHAN_A = 1
motor_A = motor(PWM_A, DIR_A, TIM_A, CHAN_A)

enc_B_chan_A = pyb.Pin('X9', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_B_chan_B = pyb.Pin('X10', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
enc_timer_B = pyb.Timer(3, prescaler = 0, period = 65535)
encoder_B = enc_timer_B.channel(1, pyb.Timer.ENC_AB)
DIR_B = 'Y4'
PWM_B = 'Y3'
TIM_B = 10
CHAN_B = 1
motor_B = motor(PWM_B, DIR_B, TIM_B, CHAN_B)

############ PARACHUTE RELAY ##################
#pin Y11
#Output pin
relay = Pin('Y11', Pin.OUT_PP)



#################light Setup ################
red_LED = pyb.LED(1)
green_LED = pyb.LED(2)
orange_LED = pyb.LED(3)
blue_LED = pyb.LED(4)


###################### IMU SETUP ######################
#UART 1
#Baudrate 57600 bps, 1 stop bit, no parity
#Buffer size very large (1000 chars) to accomodate incoming characters
razor_imu = Razor(3, 57600, read_buf_len = 1000)
razor_imu.start_streaming()

################### GLOBAL VARIABLES #########################

FORCE_START_TIMER = 90*1000*60

################# VARIABLE FOR GPS ####################
EARTH_RADIUS = 6370000
MAG_LAT = 82.7
MAG_LON = -114.4
black_rock_alt_m = 1191
alt_threshold = 1

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

############### PYBOARD ACCELEROMETER ####################
accel = pyb.Accel()

################ VARIABLES FOR WHILE LOOPS ################
change_in_altitude = -100
change_in_accel = [10,10,10]

pyb_switch = pyb.Switch()
