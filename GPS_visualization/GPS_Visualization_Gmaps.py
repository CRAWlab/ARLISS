#! /usr/bin/env python

##########################################################################################
# GPS_Visualization_Gmaps.py
#
# Modified from script to post-process GPS data from Anaconda trials
# 
# Plots data on Google Maps using pygmaps
#
# Data loaded from csv file with columns that are: 
#   Time, Date, Fix, Fix Quality, Num Sats, Latitude DDMM.mmmm, Longitude DDMM.mmmm, Speed, Heading, Alt
#
# NOTE: Plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 04/11/14
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 09/12/15 - JEV - joshua.vaughan@louisiana.edu
#       - conversion to Python 3
#       - begin conversion away from Anaconda data
#
##########################################################################################


import numpy as np
from matplotlib.pyplot import *
from scipy import signal

import tkinter as tk
from tkinter.filedialog import askopenfilename, askdirectory

import csv
import datetime

import pygmaps 

# Define what kind of output you want
save_for_web = False
produce_pymap = True
is_pyNMEA = False


try:
    root = tk.Tk()
    root.withdraw()

    file_path = askopenfilename()

    # # Process the time into a "trial time"
    # trialTime_array = np.zeros(len(trialTime))
    # 
    # for index in range(len(trialTime)):
    #     trialTime_parsed = datetime.datetime.strptime(trialTime[index],"%H:%M:%S.%f")
    #     trialTime_seconds = (trialTime_parsed.minute*60. + trialTime_parsed.second) + trialTime_parsed.microsecond/(10.**6)
    #     trialTime_array[index] = trialTime_seconds
    #     
    # trialTime_array = trialTime_array - trialTime_array[0]

    # Grab the rest of the data as numpy arrays
    #  Probably slower but more convenient
    data = np.genfromtxt(file_path, delimiter=',', skip_header = 1)

    # Parse the data (inelegantly for now)
    # time = data[:,0]
    if is_pyNMEA:
        # set up pyNMEA parsing
        pass
    else:
        date = data[:,0]

        latitude = data[:,1]
        longitude = data[:,2]
        speed = data[:,7]
        heading = data[:,8]

    #         altitude = data[:,9]
        fix_quality = data[:,13]
        num_sats = data[:,14]

    if save_for_web:
        #-----  Save data for export to http://www.gpsvisualizer.com ----------------------------
        header = ['Trackpoint','Latitude','Longitude','speed','course']

        trackpoint = (np.arange(1,len(latitude)+1)).reshape(len(latitude),1)
        latitude = latitude.reshape(len(latitude),1)
        longitude = longitude.reshape(len(latitude),1)
        speed = speed.reshape(len(latitude),1)
        heading = heading.reshape(len(latitude),1)

        map_data = np.hstack((trackpoint, latitude, longitude, speed, heading))

        with open('gps_data.csv','w') as f:
            f_csv = csv.writer(f)
            f_csv.writerow(header)
            for row in map_data:
                for el in row:
                    f.write(repr(el)+', ')
                f.write('\n')

        np.savetxt('map_data.csv', map_data, delimiter = ',', comments='', header = 'trackpoint,latitude,longitude,speed,course')

    if produce_pymap:
        ########## CONSTRUCTOR: pygmaps.maps(latitude, longitude, zoom) ##############################
        # DESC:         initialize a map  with latitude and longitude of center point  
        #               and map zoom level "15"
        # PARAMETER1:   latitude (float) latittude of map center point
        # PARAMETER2:   longitude (float) latittude of map center point
        # PARAMETER3:   zoom (int)  map zoom level 0~20
        # RETURN:       the instant of pygmaps
        #========================================================================================
        mymap = pygmaps.gmaps.maps(latitude[0], longitude[0], 15)
        
        lat_shaped = latitude.reshape(len(latitude),1)
        long_shaped = longitude.reshape(len(latitude),1)

        #----- Draw the trial on a Google map ---------------------------------------------------
        path = np.hstack((lat_shaped,long_shaped)).tolist()
    
        red = 255
        blue = 0
        green = 0
    
        color = '#%02X%02X%02X' % (red,green,blue)
        mymap.add_path(path, color)
        
        # ########## FUNCTION:  addpoint(latitude, longitude, [color])#############################
        # # DESC:         add a point into a map and dispaly it, color is optional default is red
        # # PARAMETER1:   latitude (float) latitude of the point
        # # PARAMETER2:   longitude (float) longitude of the point
        # # PARAMETER3:   color (string) color of the point showed in map, using HTML color code
        # #               HTML COLOR CODE:  http://www.computerhope.com/htmcolor.htm
        # #               e.g. red "#FF0000", Blue "#0000FF", Green "#00FF00"
        # # RETURN:       no return
        # #========================================================================================
        # mymap.addpoint(29.722168, -91.208519, 'Swiftships')


        ########## FUNCTION:  draw(file)######################################################
        # DESC:         create the html map file (.html)
        # PARAMETER1:   file (string) the map path and file
        # RETURN:       no return, generate html file in specified directory
        #========================================================================================
        
        # define filename - assumes that original datafile was .csv
        #   TODO: make this more robust
        map_filename = file_path.replace('csv','html')
        mymap.draw(map_filename)

except (KeyboardInterrupt, SystemExit): # when you press ctrl+c
    pass








