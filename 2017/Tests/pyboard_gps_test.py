#Gps Testing to get NMEA Data
#PyBoard
#Created by: Joseph Fuentes [08/09/2017]
import pyb
from pyb import UART
import micropyGPS

#Use GPS on UART 6
#Connect RX of GPS to 'Y1', TX to 'Y2' 
uart = UART(6, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters

ultimate_GPS= MicropyGPS()

while uart.any()
    ultimate_gps.update(chr(uart.readchar()))

    pres_lat = convert_latitude(ultimate_gps.latitude)
    pres_long = convert_longitude(ultimate_gps.longitude)
    present_point = (pres_lat, pres_long)
    print(present_point)
