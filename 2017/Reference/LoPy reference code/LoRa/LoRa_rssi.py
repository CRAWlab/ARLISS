from network import LoRa
import socket
import time
import pycom
import csv

csvout = csv.writer(openz('rssidata.csv', 'wb'))
csvout.writerow('strength')

# initialize LoRa in LORA mode
# more params can also be given, like frequency, tx power and spreading factor
lora = LoRa(mode=LoRa.LORA, frequency=915000000)
# create a raw LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
pycom.heartbeat(False)
pycom.rgbled(0x7f0000) # red
s.setblocking(False)
while True:
    #LED Will Be Orange while Transimitting Info
    pycom.rgbled(0xFF8000)
    # send some data
    s.send('Start')
    print('Start')
  
    #Recieve Data
    data = s.recv(64)
    print(data)
    
    #Reporting the Signal Strength of the last signal recieved
    signal_strength=[lora.rssi()]
    for item in signal_strength:
        csv.writerow()
        
    print(signal_strength)
   
   #Set condition for program to continue
    if  data == b'Lets Go':
         pycom.rgbled(0x007f00) # green
         print('Lets Go!')
         time.sleep(5)
    time.sleep(5)
