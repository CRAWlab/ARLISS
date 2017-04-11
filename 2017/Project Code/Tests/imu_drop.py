from machine import UART
import time
from machine import SD
import os

#os.listdir('/sd')

uart=UART(1, baudrate = 57600)
#IMU = uart.init(baudrate=57600, pins=('G14','G13' ))
#enable_streaming= uart.write('#o')
while 1:
#    out_raw_data = uart.write('#osct')
#   data=IMU.readline(0xD1)
    data=uart.read()
#    try:
##        request_synch_token = uart.write('#s<xy>')
#        x_raw ,  y_raw,  z_raw = data.decode('utf-8').split(',')
#        x = int(x_raw.replace('x=', ''))
#        y = int(y_raw.replace('y=', ''))
#        z = int(z_raw.replace('z=', ''))
##        data_log = open('/sd/data_log.txt', 'w')
##        data_log.write(x,  y, z)
#        print('A_x =', x)
#        print('A_y=',  y)
#        print('A_z=',  z)
#        time.sleep(1)
#    except ValueError:
#        pass
#    except AttributeError:
#        pass
##    print (yaw, pitch, roll)
#    time.sleep(2)
#    
