from machine import UART
import time
uart = UART(1, 9600)                         # init with given baudrate

data=uart.readall()
while(True):
    print(data)
    time.sleep(1)
