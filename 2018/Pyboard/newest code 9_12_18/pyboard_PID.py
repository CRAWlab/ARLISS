#! /usr/bin/env python

##########################################################################################
# Anaconda_PID.py
#
# PID module for use on the Anaconda
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 05/14/14
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * Forrest Montgomery 2016-08-30
#       - Editing for the pyboard to control pololu 47:1 DC motors
#
##########################################################################################

# import numpy as np
import time
import utime

# logger = logging.getLogger(__name__)



class PID(object):
    ''' Class to implement a basic PID controller for use on the Anaconda'''

    def __init__(self, kp, ki, kd, dt, max_out, min_out, start_time = None):
        ''' Initializing
        Arguments:
            kp = proportional gain, float > 0
            kd = derivative gain, float >= 0
            ki = integral gain, float >=-
            dt = sample time (ms)
            max_out = the maximum output of the controller
            min_out = the minimum output of the controller
        '''
        print('Creating PID controller...')
        self.kp = kp
        self.kd = kd / dt
        self.ki = ki * dt
        self.dt = dt

        self.max_output = max_out
        self.min_output = min_out

        if start_time is None:
            self.current_time = utime.ticks_us()
            self.previous_time = self.current_time
        else:
            self.current_time = 0.0
            self.previous_time = 0.0

        self.last_Error = 0.0
        self.last_state = 0.0
        self.integral_term = 0.0

        self.output = 0.0


    def change_gains(self, kp, kd, ki):
        self.kp = kp
        self.kd = kd / self.dt
        self.ki = ki * self.dt


    def change_sample_time(self, dt):
        self.sample_time_ratio = dt / self.dt

        # update gain terms to reflect new sample time
        self.ki *= self.sample_time_ratio
        self.kd /= self.sample_time_ratio

        # update sample time
        self.dt = dt


    def compute_output(self, desired_state, current_state, current_time = None, previous_time = None):
        if current_time is None:
            # print('Current Time is None')
            self.current_time = utime.ticks_us()
        else:
            self.current_time = current_time

        if previous_time is not None:
            self.previous_time = previous_time

        # print('Current State: {}\t Desired State: {}'.format(current_state, desired_state))
        # print('Current Time: {}\t Previous Time: {}'.format(self.current_time, self.previous_time))

        if utime.ticks_diff(self.current_time, self.previous_time) >= self.dt:
            self.error = desired_state - current_state

#
# Correct for heading 'wrap around' to find shortest turn
#             if self.error > 180:
#                 self.error -= 360
#             elif self.error < -180:
#                 self.error += 360

            self.integral_term += self.ki * self.error

            self.state_change = current_state - self.last_state
            self.last_state = current_state

            # print('Error: {:}\t Error_dot: {:.4f}'.format(self.error, self.state_change))

            self.output = self.kp * self.error + self.integral_term - self.kd * self.state_change

            # limit the output to within the range of possible values
            if self.output > self.max_output:
                self.output = self.max_output
                # print('Limiting PID ouput to maximum')
            elif self.output < self.min_output:
                # print('Limiting PID output to minimum')
                self.output = self.min_output

            self.previous_time = self.current_time


        # print('PID output: {:}\n'.format(self.output))
        # print('{},{}\n'.format(current_state, self.output))

        return self.output
