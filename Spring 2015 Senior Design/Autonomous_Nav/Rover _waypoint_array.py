#! /usr/bin/env python

##########################################################################################
# Husky_Waypoint_Array.py
#
# Code to read and process the CHRobotics UM6 IMU with attached GPS, decide the 
#   nearest waypoint, the heading to a given waypoint, and the desired heading correction
#
# Adapted from the same code for the Anaconda
#
# IMU parsing modified from Clearpath Robotics GitHub
#   https://github.com/clearpathrobotics/imu_um6
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 06/17/14
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#       - 
#
##########################################################################################

import numpy as np
# from matplotlib.pyplot import *

import os, sys

import serial
import struct
import math
from select import select

import time
import datetime
import csv

import threading, logging

# Project Specific Imports
import Husky_IMUwithGPS
import geographic_calculations as geoCalc
import Husky_PID
import Husky_base as Husky



#######################################################
#					ARLISS IMU
#######################################################


import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)
############ IMU print function
def imu_print():	
	if imu.IMURead():
		data = imu.getIMUData()
		fusionPose = data["fusionPose"]
		print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
		time.sleep(poll_interval*1.0/1000.0)




#B# -- represents my change


# Set up logging
# Debug level logging
# log_filename = 'Husky_DebugLog_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S') + '.log'
# logging.basicConfig(filename=log_filename, level=logging.DEBUG,
#                     format='From %(threadName)-10s - %(module)s Module: %(message)s')

## Info level logging
log_filename = 'Husky_InfoLog_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S') + '.log'
logging.basicConfig(filename = log_filename, level=logging.INFO,
                    format='From %(threadName)-10s - %(module)s Module: %(message)s')

## Critical level logging
# log_filename = 'Husky_CriticalLog_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S') + '.log'
# logging.basicConfig(filename = log_filename, level=logging.CRITICAL,
#                     format='From %(threadName)-10s - %(module)s Module:: %(message)s')

# Define File Constants
WAYPOINT_TOLERANCE = 3         # Distance to waypoint before looking at next one
USE_GPS_HEADING = False         # Whether or not to use the GPS heading or IMU

def setup_data_file(header):
    # Set up the csv file to write to.and write header
    # The filename contains a date/time string of format gpsData_YYYY-MM-DD_HHMMSS.csv
    data_filename = 'Control_imuData_withGPS' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')+'.csv'
    
    with open(data_filename, 'a') as data_file:  # Just use 'w' mode in 3.x
        writer = csv.writer(data_file)
        writer.writerow(header)

    return data_filename

def append_to_data_file(data_filename, write_data):
    with open(data_filename, 'a') as data_file:  # Just use 'w' mode in 3.x
        writer = csv.writer(data_file)            
        writer.writerow(write_data)

if __name__ == '__main__':
    logging.info('Trial Started at ' + datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    
    # Back left pole
    # waypoints = np.array([[30.210615158081055, -92.02279663085938]])
    
    # corner of parking lot toward Johnston St.
    # waypoints = np.array([[30.21042252, -92.0229873657226]])

    # middle pole
    # waypoints = np.array([[30.2102903556, -92.0227317699]])
    
#     waypoints = np.array([[30.21042252, -92.0229873657226],
#                           [30.210615158081055, -92.02279663085938],
#                           [30.2102903556, -92.0227317699]])
                          
    waypoints = np.array([[30.210599, -92.021245],
                          [30.210773468017578, -92.0213623046875]])


    # initialize waypoint counter, increments as we pass through them
    current_waypoint_number = 0
    
    # Define the first waypoint as the first waypoint
    next_waypoint = waypoints[current_waypoint_number]

    ### EXAMPLE USAGE ###
    print '\nStarting up...'
    
    # Create the PID controller
    kp = 3.0
    ki = 0.0
    kd = 0.0
    dt = 0.05
    max_output = 100.0
    min_output = -100.0
    pid = Husky_PID.PID(kp, ki, kd, dt, max_output, min_output)
    
    # Set initial control output to 0
    control_output = 0.0
    
    # Initialize the Husky
    #B#husky = Husky.Husky(VEL_SCALE = 1.0)
    # husky_port = '/dev/tty.NoZAP-PL2303-0030111D'
    #B#husky_port = '/dev/tty.USA19H1a121P1.1'
    #B#husky.connect_to_Husky(husky_port)
    
    #############################################################################################
    #############################################################################################
    #############################################################################################
    # Set up IMU
    # Serial port to which the IMU is attached, likely need to change
    # IMUport = '/dev/tty.USA19H142P1.1'  # Keyspan USB-> Serial - right side of rMBP
    #B#IMUport = '/dev/tty.usbserial-FTGSQ1XH' # UM6 JST -> USB 
    imu_print()
    
  
  
  
    #######################the husky has the imu GPS together#######################################
    #############################################################################################
    #############################################################################################
    # Create the IMU object
    #B#imu = Husky_IMUwithGPS.IMU(IMUport)

    #B#imu.setup_data_file()    
    
    # Create a thread that reads data from the IMU continually
    #B#imu.create_thread()
    
    # Set up control data recording
    header = ('Elapsed Time (s)', 
              'IMU Heading (deg from N)',
              'Latitude (+/- = North/South)', 'Longitude (+/- = East/West)',
              'GPS Heading (deg from N)', 'Speed (m/s)',
              'Current Waypoint Number', 
              'Current Waypoint Latitude (+/- = North/South)',
              'Current Waypoint Longitude (+/- = East/West)',
              'Distance to Waypoint (m)', 'Bearing to Waypoint (deg)',
              'Needed Course Correction (deg)',
              'Turn Direction (-1 Left, 0 Straight, 1 Right)',
              'Control Action from PID')

    control_filename = setup_data_file(header)
    
    try:
        print '\nTo begin test, press any key\n'
        raw_input() # Wait for user input to begin trial
        
        while True:

            # Always full speed (unless scaled on the Husky side fo communication) 
            #   TODO: change? 06/17/14
            throttle = 1
        
            # Clear the terminal (optional)
            os.system('clear')
            
            #B#imu.show_imu_data()
            imu_print()
            
    #####################SELECT GPS OR IMU#################################################
		try:
    	report = session.next()
		# Wait for a 'TPV' report and display the current time
		# To see all report data, uncomment the line below
		# print report
        if report['class'] == 'TPV':
            if hasattr(report, 'time'):
                print report.time
          	except KeyError:
          		pass
      		except KeyboardInterrupt:
        		quit()
       		except StopIteration:
				session = None
				print "GPSD has terminated"
		
			current_location = (imu.data['DATA_GPS'][0], imu.data['DATA_GPS'][1])
            

            #B#if imu.data <> {}:
             #B#   imu.append_to_data_file()
             #B#   current_location = (imu.data['DATA_GPS'][0], imu.data['DATA_GPS'][1])
            
                distance_to_nextWaypoint = geoCalc.calculate_distance(current_location, next_waypoint)
                desired_heading = geoCalc.calculate_bearing(current_location, next_waypoint)
                
                
    #####################SELECT GPS OR IMU#################################################
    #############################################################################################
    #############################################################################################
                if USE_GPS_HEADING:
                    current_heading = imu.data['DATA_GPS'][2]
                else:
                    current_heading = imu.data['Heading']
                
                course_correction = desired_heading - current_heading
                
                # Correct for heading 'wrap around' to find shortest turn
                if course_correction > 180:
                    course_correction -= 360
                elif course_correction < -180:
                    course_correction += 360
                
                # If the current heading is finite, then calculate the steering input
                if np.isfinite(current_heading):
                    # Just act on the error, so we set the current condition to 0
                    #   and the desired to the course correction
                    control_output = pid.compute_output(course_correction, 0.0)
                else:
                    control_output = 0.0
                
                # record the turn direction - mainly for debugging
                if course_correction > 0:
                    turn_direction = 1  # Turn right
                elif course_correction < 0:
                    turn_direction = -1 # Turn left
                else:
                    turn_direction = 0  # Stay straight

                course_correction_text = ('Left','None','Right')
                
                
                # Scale the control input to match the format required by the Husky
                #  from (-100, 100) --> (-1, 1)
                angular_velocity = control_output/100
                
                # Send the data over serial to the Husky
                husky.scale_and_send_velocity(throttle, angular_velocity)
                
                if current_waypoint_number < len(waypoints):
                    print ''
                    print ''
                    print '                          Control Decisions                   '
                    print '======================================================================'
                    print ''          
                    print 'Moving Toward Waypoint {:d} of {:d}'.format((current_waypoint_number+1),len(waypoints))
                    print ''
                    print 'Desired Heading (deg)                                       {:10.4f}'.format(desired_heading)
                    print 'Distance to Next Waypoint (m)                               {:10.4f}'.format(distance_to_nextWaypoint)        
                    print ''      
                    print 'Course Correction (deg)                                     {:10.4f}'.format(course_correction)
                    print '  Turning {}                                                        '.format(course_correction_text[turn_direction+1])
                    print ''
                    print 'PID Controller Output (-100 < Ang Vel < 100)                {:10.4f}'.format(control_output)
                    print ''
                    
                    # If we are within range of the current waypoint move to the next one
                    if distance_to_nextWaypoint < WAYPOINT_TOLERANCE:
                        current_waypoint_number += 1
                        if current_waypoint_number < len(waypoints):
                            next_waypoint = waypoints[current_waypoint_number]
                else:
                    logging.info('Arrived at Destination at ' + datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
                    
                    print ''
                    print ''
                    print '                          Control Decisions                           '
                    print '======================================================================'
                    print '' 
                    print '                        Arrived at Destination!                       '
                    
                    # Send the data - "Neutral Position"
                    husky.scale_and_send_velocity(0, 0)
                    break
            else:
                # Send the data - "Neutral Position"
                husky.scale_and_send_velocity(0, 0)  
                
            data = [imu.data['Time'], 
                      imu.data['Heading'],
                      imu.data['DATA_GPS'][0], imu.data['DATA_GPS'][1],
                      imu.data['DATA_GPS'][2], imu.data['DATA_GPS'][3],
                      current_waypoint_number, next_waypoint[0], next_waypoint[1],
                      distance_to_nextWaypoint, desired_heading, 
                      course_correction, turn_direction, control_output]    
            
            append_to_data_file(control_filename, data)
            
            # 20Hz update (GPS will update at 10Hz)
            time.sleep(0.05)
            
    except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
        # Send the data - "Neutral Position"
        husky.scale_and_send_velocity(0, 0)  

logging.info('Shutting down at ' + datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
# husky.close()
imu.stop_thread()
time.sleep(1)
logging.info('Closing at ' + datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))