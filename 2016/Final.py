'''
## Left Motor

### Encoder -- Timer 2:
X1: White encoder wire
X2: Yellow encoder wire

### Motor Driver
X7: DIR
X8: PWM -- Timer 14 -- Channel 1


## Right Motor

### Encoder -- Timer 4
X9: Yellow encoder wire
X10 Orange encoder wire

### Motor Driver
Y4: DIR
Y3: PWM -- Timer 10 -- Channel 1

## UART 2
X3: XBee -- DIN
X4: XBee -- DOUT
9600bps

## UART 6
Y1: GPS -- TX
Y2: GPS -- RX
9600bps

## UART 3
Y9:  IMU -- RX
Y10: IMU -- Tx
57600bps

## ETC
X5: Parachute release
Y12: Buzzer -- Timer 8
Y11: Relay
'''

import pyb
from pyb import UART
from pyb import ExtInt
from pyb import Pin
from micropyGPS import MicropyGPS
import motor
import time
import math

finish_point =

launch_point =

backup_timer =

XBEE = UART(2, 115200)
GPS = UART(6, 9600)

my_gps = MicropyGPS()

relay = Pin('Y11', Pin.OUT_PP)
