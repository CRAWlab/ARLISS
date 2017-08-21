import pyb
from pyb import UART
from micropyGPS import MicropyGPS
import time
# Setup the connection to your GPS here
# This example uses UART 3 with RX on pin Y10
# Baudrate is 9600bps, with the standard 8 bits, 1 stop bit, no parity
uart = UART(3, 9600)
my_gps = MicropyGPS(-7)

while True:
    my_gps.update(chr(uart.readchar()))
    if my_gps.latitude[0] != 0:
        initial_lat = my_gps.latitude
        initial_long = my_gps.longitude
        initial_point = (initial_lat, initial_long)
        print(initial_point)
        time.sleep(3)
