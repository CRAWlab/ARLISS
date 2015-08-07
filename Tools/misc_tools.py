#! /usr/bin/env python

##########################################################################################
# misc_tools.py
#
# Functions to perform miscellaneous actions, such as setting up files, saving data, etc.
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 08/22/14
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   *
#
##########################################################################################


import csv

def setup_data_file(header, data_filename):
    ''' Sets up the csv file to write to.and writes the header'''
    with open(data_filename, 'a') as data_file:  # Just use 'w' mode in 3.x
        writer = csv.writer(data_file)
        writer.writerow(header)

    return data_filename


def append_to_data_file(data_filename, write_data):
    ''' function appends data to the end of existing text file, data_filename'''
    
    with open(data_filename, 'a') as data_file:  # Just use 'w' mode in 3.x
        writer = csv.writer(data_file)            
        writer.writerow(write_data)