#! /usr/bin/env python

##########################################################################################
# GPS_Visualization_Folium.py
#
# Script to read control decision data collected using during single waypoint trials
#  Adapted from a similar script used to process data from the Anaconda and Husky
#
# Uses Folium to generate maps of a GPS path 
#  - https://github.com/python-visualization/folium
#  - Conda install - https://anaconda.org/ioos/folium
#
# NOTE: Plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 06/11/14
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 07/10/14 - Joshua Vaughan - joshua.vaughan@louisiana.edu
#       - condensed batch processing and single run into this script, choose via boolean
#       - condensed "only IMU" data and "Control" data scripts into this one
#       - general code cleanup
#   * 09/12/15 - JEV - joshua.vaughan@louisiana.edu
#       - conversion to Python 3
#       - begin conversion away from Anaconda data
#
##########################################################################################

import numpy as np

import folium
import glob
import tkinter as tk
from tkinter.filedialog import askopenfilename, askdirectory

import geographic_calculations as geoCalc


PRODUCE_FOLIUMMAP = True         # Produce a Folium-based map?
DRAW_WAYPOINTS = False           # Draw the waypoints?
BATCH = False                    # Batch processing?


def create_map(data_filename):
    ''' Actually creates the map '''
    waypoints = None
    # TODO: be more efficient
    with open(data_filename, 'rb') as data_file:  
         data = np.genfromtxt(data_file, delimiter=',', skip_header = 1, dtype = 'float')

    if np.shape(data)[1] == 14: # _controlHistory... file
        data_ok = True
        time = data[:,0]
        imu_heading = data[:,1]
        latitude = data[:,2]
        longitude = data[:,3]
        gps_heading = data[:,4]
        gps_speed = data[:,5]
        waypoint_number = data[:,6]             
        waypoint_latitude = data[:,7]           
        waypoint_longitude = data[:,8]          
        distance_to_waypoint = data[:,9]        
        bearing_to_waypoint = data[:,10] 
        course_correction = data[:,11]
        turn_direction = data[:,12]
        control_from_PID = data[:,13]
    
        _, waypoint_indices = np.unique(waypoint_number, return_index = True)
    
        waypoints = np.vstack((waypoint_latitude[waypoint_indices], 
                               waypoint_longitude[waypoint_indices]))
                           
        waypoints = waypoints.T

    elif np.shape(data)[1] == 19:  # _rawIMUGPS... file
        data_ok = True
        time = data[:,0]
        quart0 = data[:,1]
        quart1 = data[:,2]
        quart2 = data[:,3]
        quart3 = data[:,4]
        x_accel = data[:,5]
        y_accel = data[:,6]
        z_accel = data[:,7]
        x_mag = data[:,8]
        y_mag = data[:,9]
        z_mag = data[:,10]
        roll = data[:,11]
        pitch = data[:,12]
        yad = data[:,13]
        imu_heading = data[:,14]
        latitude = data[:,15]
        longitude = data[:,16]
        gps_heading = data[:,17]
        gps_speed = data[:,18]
    
        waypoints = None
        
    else:
        data_ok = False
        print('\nImproper data length in file {}.'.format(data_filename))
        print('Skippping it... \n\n')

    if data_ok: # If we have meaningful data, make the map
        # Define the start, target, and midpoint locations
        start = np.array([latitude[0], longitude[0]])

        if waypoints is not None:
            target = waypoints[-1,:]    # last waypoint is the target location
        else:
            target = np.array([latitude[-1], longitude[-1]])


        midpoint = geoCalc.calculate_midpoint(start, target)


        if PRODUCE_FOLIUMMAP:
            ''' Create a folium map'''
            # Set up base map, centered on the midpoint between start and finish
            mymap = folium.Map(location = [midpoint[0], midpoint[1]], zoom_start=16)
    
            lat_shaped = latitude.reshape(len(latitude),1)
            long_shaped = longitude.reshape(len(latitude),1)

            # Draw a green circle with popup information at the start location
            mymap.circle_marker(location = [start[0], start[1]], radius = 10, 
                                            popup = 'Start -- Lat, Lon: {:4.4f}, {:4.4f}'.format(start[0], start[1]), 
                                            line_color = '#00FF00', 
                                            fill_color = '#00FF00')
            
            # Draw a red circle with popup information at the target location
            mymap.circle_marker(location = [target[0], target[1]], radius = 10, 
                                            popup = 'Target -- Lat, Lon: {:4.4f}, {:4.4f}'.format(target[0], target[1]), 
                                            line_color = '#FF0000', 
                                            fill_color = '#FF0000')    

            if DRAW_WAYPOINTS:
                for index, waypoint in enumerate(waypoints):
                    if index < len(waypoints)-1:
                        # Draw white circles with popup information at each waypoint
                        mymap.circle_marker(location = [waypoint[0],waypoint[1]], 
                                            radius = 8, 
                                            popup='Waypoint Num: {:.0f} -- Lat, Lon: {:4.4f}, {:4.4f}'.format(index+1, waypoint[0], waypoint[1]), 
                                            line_color = '#FFFFFF', 
                                            fill_color = '#FFFFFF')

                #----- Draw the trial on a  map ---------------------------------------------------
            path = np.hstack((lat_shaped,long_shaped))

            # if path is large, downsample for plotting, plot only ~1000 points
            if np.shape(path)[0] > 1000:
                path = path[0::np.shape(path)[0]//1000]
    
            # Uncomment below to draw the path line in addition to the data point bubbles above
            # mymap.line(path, line_color='#FF0000', line_weight=5)
    
            # for each point on the path, draw a circle that contains system information
            #  in a popup when clicked on
            for index, current_pos in enumerate(path):
                mymap.circle_marker(location = [current_pos[0], current_pos[1]], radius = 1, 
                                    popup = 'Time: {:3.2f} s -- Lat, Lon: {:4.4f}, {:4.4f} -- Speed: {:3.2f} m/s -- Actual Heading: {:3.0f} deg -- Desired Heading: {:3.0f} deg -- Distance to Waypoint: {:.0f} m'.format(time[index], latitude[index], longitude[index], gps_speed[index], imu_heading[index], bearing_to_waypoint[index], distance_to_waypoint[index]), 
                                    line_color = '#0000FF', fill_color = '#0000FF')


            # define filename - assumes that original datafile was .csv
            #   TODO: make this more robust
            map_filename = data_filename.replace('csv', 'html')
            mymap.create_map(map_filename)
        


if __name__ == "__main__":
    if BATCH:
        root = tk.Tk()
        root.withdraw()
        file_path = askdirectory()
    
        filename_pattern = file_path + "/*_controlHistory.csv"

        for data_filename in glob.glob(filename_pattern):
            print(data_filename)
            create_map(data_filename)

    else:
        root = tk.Tk()
        root.withdraw()

        data_filename = askopenfilename()
        create_map(data_filename)
