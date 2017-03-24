from network import LoRa
import socket
import time
import pycom
from machine import SD,  Timer
import os
##############################Prepping SD#################################
sd = SD()
os.mount(sd, '/sd')
# check the content
os.listdir('/sd')

###################################Creating Chrono Objject###################
chrono = Timer.Chrono()
##########################LORA Socket Initalization###############################################
# initialize LoRa in LORA mode
# more params can also be given, like frequency, tx power and spreading factor
lora = LoRa(mode=LoRa.LORA, frequency=915000000)
# create a raw LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)

###############Run Loop######################
while True:
    #Require confirmation 
    run_prompt= input('Would you like to begin communication with the rover?')
    
    if run_prompt in ( 'yes',  'ye',  'y'):
        data_log = open('/sd/data_log.txt', 'w')
        data_log.write(run_prompt)
        continue

    start_command = s.send('Start')
    data_log.write(start_command)
    
    confirmation = s.recv(64)
    data_log.write(confirmation)
    
    
    if confirmation == b'Ready for command':
        print('Rover is ready for Destination input.')
        data_log.write(confirmation)
        break
data_log.close()


time.sleep(2)
##################Destination Input Loop#######################
while True:
    coordinate_prompt = input('''Please enter destination coordinates in format lat: deg,min,sec, 'hemi', long: deg, min, sec, 'hemi' ''')
    destination_coordinates= coordinate_prompt
    dest_lat_deg = destination_coordinates [0]
    dest_lat_min = destination_coordinates [1]
    dest_lat_sec = destination_coordinates [2]
    dest_lat_hemi = destination_coordinates [3]
    dest_long_deg = destination_coordinates [4]
    dest_long_min = destination_coordinates [5]
    dest_long_sec = destination_coordinates [6]
    dest_long_hemi = destination_coordinates [7]
    print(destination_coordinates)
    data_log = open('/sd/data_log.txt', 'w')
    data_log.write(destination_coordinates)
    
    while True:
        s.send(destination_coordinates)
        coordinate_confirmation = s.recv(64)
        if coordinate_confirmation ==b'coordnates recieved':
            data_log.write(destination_coordinates)
            break
    


#################################Descent loop#############################
while True:
    chrono.start()
    elapsed_time= chrono.read()
    force_start = 60*60  #60 minutes
    accelerometer_data = s.recv(64)
    data_log = open('/sd/data_log.txt', 'w')
    data_log.write(accelerometer_data)
    if force_start== elapsed_time:
        s.send('Begin Movement')
        data_log.write('Force start intiated')
        break
    elif accelerometer_data==0:
        s.send('Begin Movement')
        print('Rover Landed Beginning Movement')
        break

#########################Recording Gps Coordinates from rover##################################
while True: 
    current_coordinates = s.recv(64)
    data_log.write(curr)
    print(current_coordinates)
    if current_coordinates == destination_coordinates:
        #need to tweak so that a range is acceptable
        #look into numpy for this logic
        print('Destination Reached')
        break
print('Goal Reached distribute high fives')
