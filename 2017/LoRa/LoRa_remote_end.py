from network import LoRa
import socket
import pycom
import time

# initialize LoRa in LORA mode
# more params can also be given, like frequency, tx power and spreading factor
lora = LoRa(mode=LoRa.LORA, frequency=915000000)

# create a raw LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
pycom.heartbeat(False)
pycom.rgbled(0x7f0000) # red
s.setblocking(False)

while True:
    #Allow For Data to be recieved
    #Set LED Blue while Recieving
    pycom.rgbled(0x0000CC)
    data = s.recv(64)
    #Get Signal strength in units of dB closer to 0 is strongest
    signal_strength=lora.rssi()
    print(signal_strength)
   
    if data== b'Start':
         pycom.rgbled(0x00FF00) # green
         send = s.send('Lets Go')
         time.sleep(5)
    

