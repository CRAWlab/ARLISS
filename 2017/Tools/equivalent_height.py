#! /usr/bin/env python

###############################################################################
# equivalent_drop_height.py
#
# Script to plot the height from which to drop an object to reach a 
#  velocity by ground impact. Ignores any air resistance.
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 08/13/15
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   *
#
###############################################################################
import numpy as np
import matplotlib as plt

PLOT_SI = True              # plot the curve in SI units?
PLOT_IMPERIAL = True        # fplot the curve in imperial units?

g = 9.81                    # accel due to gravity


# Impact velocities between 0.1 and 10m/s
impact_velocity = np.arange(0.1, 10, 0.1)

# Use conservation of energy, ignore aerodynamic effects
height = impact_velocity**2 / (2 * g) 


# Plot in SI?
if PLOT_SI:
    # Set the plot size - 3x2 aspect ratio is best
    fig = plt.Figure(figsize=(6,4))
    ax = plt.gca()
    plt.subplots_adjust(bottom=0.17, left=0.17, top=0.96, right=0.96)

    # Change the axis units font
    plt.setp(ax.get_ymajorticklabels(),fontsize=18)
    plt.setp(ax.get_xmajorticklabels(),fontsize=18)

    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')

    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    # Turn on the plot grid and set appropriate linestyle and color
    ax.grid(True,linestyle=':', color='0.75')
    ax.set_axisbelow(True)


    # Define the X and Y axis labels
    plt.xlabel('Impact Velocity (m/s)', fontsize=22, weight='bold', labelpad=5)
    plt.ylabel('Drop Height (m)', fontsize=22, weight='bold', labelpad=10)
    plt.plot(impact_velocity, height, linewidth=2, linestyle='-', label=r'Height (m)')

    # uncomment below and set limits if needed
    # plt.xlim(0,5)
    # plt.ylim(0,10)

    # Adjust the page layout filling the page using the new tight_layout command
    plt.tight_layout(pad=0.5)

    # save the figure as a high-res pdf in the current folder
    plt.savefig('equivalent_drop_height_SI.pdf')


# Also plot in imperial units?
if PLOT_IMPERIAL:
    # Set the plot size - 3x2 aspect ratio is best
    fig = plt.figure(figsize=(6,4))
    ax = plt.gca()
    plt.subplots_adjust(bottom=0.17, left=0.17, top=0.96, right=0.96)

    # Change the axis units font
    plt.setp(ax.get_ymajorticklabels(),fontsize=18)
    plt.setp(ax.get_xmajorticklabels(),fontsize=18)

    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')

    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    # Turn on the plot grid and set appropriate linestyle and color
    ax.grid(True,linestyle=':', color='0.75')
    ax.set_axisbelow(True)


    # Define the X and Y axis labels
    plt.xlabel('Impact Velocity (mph)', fontsize=22, weight='bold', labelpad=5)
    plt.ylabel('Drop Height (ft)', fontsize=22, weight='bold', labelpad=10)

    plt.plot(impact_velocity * 2.23694, height * 3.28084, linewidth=2, linestyle='-', label=r'Height (m)')
    
    # uncomment below and set limits if needed
    # plt.xlim(0,5)
    # plt.ylim(0,10)

    # Adjust the page layout filling the page using the new tight_layout command
    plt.tight_layout(pad=0.5)

    # save the figure as a high-res pdf in the current folder
    plt.savefig('equivalent_drop_height_imperial.pdf')


# show the figures
plt.show()
