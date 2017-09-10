#!/bin/sh

#  turn_degree_test.py
#  
#
#  Created by Joseph Fuentes on 9/10/17.
#

import pyb
import functions

#Purpose of this test is to find optimal value for gain to perfect turning

angle = 90
direction = 'left'
gain = 0 # Adjust gain until expected output is preformed

functions.turn_degree(angle, direction, gain)

