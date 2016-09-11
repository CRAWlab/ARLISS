#!/usr/bin/python
###############################################################################
# Filename    : PID_Test_plot.py 
# Created     : September 9, 2016
# Author      : Forrest
'''
Description   :
This is a plot of the PID output of the PyBoard with the Pololu 47:1 motors
'''
# Modified    :
###############################################################################

import  numpy as np
import matplotlib.pyplot as plt
import numpy as np
data = np.genfromtxt('pid.csv', delimiter=',')
true = data[:,0]
size = np.size(data[:,0])
time = np.linspace(0,size,size)
desired = np.linspace(400,400,400)
plt.plot(time, true)
plt.show()
