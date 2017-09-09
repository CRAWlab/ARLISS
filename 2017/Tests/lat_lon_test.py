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
from pyb import UART
from pyb import ExtInt
from pyb import Pin
from micropyGPS import MicropyGPS


# Global Flag to Start GPS data Processing
new_data = False
finish_point=(30.2107,-92.0209)

# Callback Function
def pps_callback(line):
    print("Updated GPS Object...")
    global new_data  # Use Global to trigger update
    new_data = True

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


print('GPS Interrupt Tester')

# Instantiate the micropyGPS object
my_gps = MicropyGPS()

# Setup the connection to your GPS here
# This example uses UART 3 with RX on pin Y10
# Baudrate is 9600bps, with the standard 8 bits, 1 stop bit, no parity
# Also made the buffer size very large (1000 chars) to accommodate all the characters that stack up
# each second
uart = UART(3, 9600, read_buf_len=1000)

# Create an external interrupt on pin X8
pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, pps_callback)

# Main Infinite Loop
while 1:
    # Do Other Stuff Here.......
    
    # Update the GPS Object when flag is tripped
    if new_data:
        while uart.any():
            my_gps.update(chr(uart.readchar()))  # Note the conversion to to chr, UART outputs ints normally

        lat = my_gps.latitude
        lon = my_gps.longitude
        converted_lat = convert_latitude(lat)
        converted_lon = convert_longitude(lon)
        print(converted_lat)
        print(converted_lon)
        pyb.delay(1000)
        new_data = False  # Clear the flag
        break

