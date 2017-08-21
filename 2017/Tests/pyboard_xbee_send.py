#XBee Test code send packets to another xbee
#PyBoard
#Created by: Joseph Fuentes [08/09/2017]

import pyb
from pyb import UART
from pyb import ExtInt
from pyb import Pin
import time
import math

#Connect RX of XBee to 'X3', TX to 'X4'
xbee = UART(2, 115200)
xbee.write('\nChecking GPS and altitude to see if landed...')
