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
            print('Current Time is None')
            self.current_time = utime.ticks_us()
        else:
            self.current_time = current_time

        if previous_time is not None:
            self.previous_time = previous_time

        print('Current State: {}\t Desired State: {}'.format(current_state, desired_state))
        print('Current Time: {}\t Previous Time: {}'.format(self.current_time, self.previous_time))

        if (self.current_time - self.previous_time >= self.dt):
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

            print('Error: {:}\t Error_dot: {:.4f}'.format(self.error, self.state_change))

            self.output = self.kp * self.error + self.ki * self.integral_term - self.kd * self.state_change

            # limit the output to within the range of possible values
            if self.output > self.max_output:
                self.output = self.max_output
            elif self.output < self.min_output:
                self.output = self.min_output

            self.previous_time = self.current_time

        print('PID output: {:}\n'.format(self.output))

        return self.output

# # Example use
# if __name__ == '__main__':
#     from matplotlib.pyplot import *
#     from scipy.integrate import odeint


#     # Debug level logging
#     # logging.basicConfig(level=logging.DEBUG,
#     #                     format='From %(threadName)-10s: %(message)s',
#     #                     )

#     # logging.basicConfig(level=logging.CRITICAL,
#     #                     format='From %(threadName)-10s: %(message)s',
#     #                     )

#     # Create the PID controller
#     kp = (2*np.pi)**2
#     ki = 135.0
#     kd = 10.0
#     pid = PID(kp, ki, kd, 0.001, 100, -100, 0.0)

#     def eq_of_motion(w, t, p):
#         """
#         Defines the differential equations for the coupled spring-mass system.

#         Arguments:
#             w :  vector of the state variables:
#             t :  time
#             p :  vector of the parameters:
#         """
#         x, x_dot = w
#         m, desired, PID_force = p

#         logging.debug('PID_force = {:.4f}'.format(PID_force))

#         #print 'In ODE loop: ' + str(PID_force)

#         # Create sysODE = (x',y_dot')
#         #  We ignore the xd_dot term, as it is only an impulse as the start of the step
#         sysODE = [x_dot,
#                   1.0/m * PID_force - 5.0]
#         return sysODE

#     # ODE solver parameters
#     abserr = 1.0e-9
#     relerr = 1.0e-9
#     max_step = 0.001
#     stoptime = 5.0
#     numpoints = 5001

#     # Create the time samples for the output of the ODE solver.
#     t = np.linspace(0,stoptime,numpoints)


#     # Define and pack up the parameters and initial conditions:
#     m = 1.0
#     desired = 1.0
#     x_init = 0.0
#     x_dot_init = 0.0
#     p = [m, desired]
#     x0 = [x_init, x_dot_init]

#     # Define an empty response array to fill in the for loop below
#     resp = np.zeros((len(t)-1,2))


#     # Call the ODE solver one step at a time to simulate a "real time" loop
#     for ii in xrange(len(t)-1):
#         PID_force = pid.compute_output(desired, x0[0], t[ii+1], t[ii])
#         logging.debug('In Time loop: {:4f}'.format(PID_force))
#         logging.debug('Current resp: {:.4f}'.format(x0[0]))
#         logging.debug('Times {:.4f}, {:.4f}'.format(t[ii], t[ii+1]))

#         p = [m, desired, PID_force]
#         _, resp[ii,:] = odeint(eq_of_motion, x0, (t[ii], t[ii+1]), args=(p,), atol = abserr, rtol = relerr, hmax=max_step)

#         # Update the initial guess for the next time through this loop
#         x0 = resp[ii,:]



#     # Plot the results
#     # Set the plot size - 3x2 aspect ratio is best
#     fig = figure(figsize=(6,4))
#     ax = gca()
#     subplots_adjust(bottom=0.17,left=0.17,top=0.96,right=0.96)

#     # Change the axis units to CMUSerif-Roman
#     setp(ax.get_ymajorticklabels(),family='CMUSerif-Roman',fontsize=18)
#     setp(ax.get_xmajorticklabels(),family='CMUSerif-Roman',fontsize=18)

#     ax.spines['right'].set_color('none')
#     ax.spines['top'].set_color('none')

#     ax.xaxis.set_ticks_position('bottom')
#     ax.yaxis.set_ticks_position('left')

#     # Turn on the plot grid and set appropriate linestyle and color
#     ax.grid(True,linestyle=':',color='0.75')
#     ax.set_axisbelow(True)

#     # Define the X and Y axis labels
#     xlabel('Time (s)',family='CMUSerif-Roman',fontsize=22,weight='bold',labelpad=5)
#     ylabel('Y label (units)',family='CMUSerif-Roman',fontsize=22,weight='bold',labelpad=10)

#     plot(t,desired * np.ones_like(t), linewidth=2, linestyle = '--', label=r'Setpoint')
#     plot(t[0:-1], resp[:,0], linewidth=2, linestyle='-', label=r'Response')

#     # uncomment below and set limits if needed
#     # xlim(0,5)
#     # ylim(0,10)

#     # Create the legend, then fix the fontsize
#     leg = legend(loc='upper right', fancybox=True)
#     ltext  = leg.get_texts()
#     setp(ltext,family='CMUSerif-Roman',fontsize=16)

#     # Adjust the page layout filling the page using the new tight_layout command
#     tight_layout(pad=0.5)


#
#     # Set the plot size - 3x2 aspect ratio is best
#     fig = figure(figsize=(6,4))
#     ax = gca()
#     subplots_adjust(bottom=0.17,left=0.17,top=0.96,right=0.96)
#
#     # Change the axis units to CMUSerif-Roman
#     setp(ax.get_ymajorticklabels(),family='CMUSerif-Roman',fontsize=18)
#     setp(ax.get_xmajorticklabels(),family='CMUSerif-Roman',fontsize=18)
#
#     ax.spines['right'].set_color('none')
#     ax.spines['top'].set_color('none')
#
#     ax.xaxis.set_ticks_position('bottom')
#     ax.yaxis.set_ticks_position('left')
#
#     # Turn on the plot grid and set appropriate linestyle and color
#     ax.grid(True,linestyle=':',color='0.75')
#     ax.set_axisbelow(True)
#
#     # Define the X and Y axis labels
#     xlabel('Time (s)',family='CMUSerif-Roman',fontsize=22,weight='bold',labelpad=5)
#     ylabel('PID output',family='CMUSerif-Roman',fontsize=22,weight='bold',labelpad=10)
#
#     plot(t,desired * np.ones_like(t), linewidth=2, linestyle = '--', label=r'Setpoint')
#
#     pid_output = np.zeros_like(t)
#     for ii in xrange(len(resp)):
#         pid_output[ii] = pid.compute_output(desired, resp[ii,0], t[ii])
#
#     plot(t,pid_output, linewidth=2, linestyle='-', label=r'PID output')
#
#     # uncomment below and set limits if needed
#     # xlim(0,5)
#     # ylim(0,10)
#
#     # Create the legend, then fix the fontsize
#     leg = legend(loc='upper right', fancybox=True)
#     ltext  = leg.get_texts()
#     setp(ltext,family='CMUSerif-Roman',fontsize=16)
#
#     # Adjust the page layout filling the page using the new tight_layout command
#     tight_layout(pad=0.5)
#
#     # save the figure as a high-res pdf in the current folder
# #     savefig('plot_filename.pdf')
#

    # show the figure
    # show()
