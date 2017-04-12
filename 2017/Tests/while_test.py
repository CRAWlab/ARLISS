
import time

while True:
    #Require confirmation 
    run_prompt= input('Would you like to begin communication with the rover? ' )
    
    if run_prompt in ('yes','ye','y'):
        break
    else:
        raise ValueError('Please enter yes to begin')
    
    
time.sleep(3)

while True:
    coordinate_prompt = input('''Please enter destination coordinates in format lat: deg,min,sec, 'hemi', long: deg, min, sec, 'hemi''')
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
    break
time.sleep(3)
print('Coordinates Recieved')

