#!/bin/sh

#  convert_coordinates.py
#  
#
#  Created by Joseph Fuentes on 9/13/17.
#

def convert_latitude(lat_NS):
    """ Function to convert deg m N/S latitude to DD.dddd (decimal degrees)
        Arguments:
        lat_NS : tuple representing latitude
        in format of MicroGPS gps.latitude
        Returns:
        float representing latitidue in DD.dddd
        Created By: Dr. Joshua Vaughan - joshua.vaughan@louisiana.edu
        """
    
    return (lat_NS[0] + lat_NS[1] / 60) * (1.0 if lat_NS[2] == 'N' else -1.0)

def convert_longitude(long_EW):
    """ Function to convert deg m E/W longitude to DD.dddd (decimal degrees)
        Arguments:
        long_EW : tuple representing longitude
        in format of MicroGPS gps.longitude
        Returns:
        float representing longtidue in DD.dddd
        Created By: Dr. Joshua Vaughan - joshua.vaughan@louisiana.edu
        """
    
    return (long_EW[0] + long_EW[1] / 60) * (1.0 if long_EW[2] == 'E' else -1.0)

latitude = (40,52.782, 'N')
longitude = (119, 7.308, 'W')
lat = convert_latitude(latitude)
lon = convert_longitude(longitude)
point = (lat, lon)
print(point)
