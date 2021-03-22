#!/usr/bin/env python3

"""
File:   run_test.py
Author: pdsherman
Date:   March. 2021
"""

import sys
import time
import math

import numpy as np
import matplotlib.pyplot as plt

import u6                 # LabJack python library (loadcell sampling)
from RosMotor import *    # Interfacing with benchtop motor

def v2N(volt):
    return 40.0975*volt - 246.9117

def linreg(x, y):
    xavg = np.mean(x)
    yavg = np.mean(y)

    n   = len(x)
    sx  = sum(x)
    sx2 = sum([i*i for i in x])
    sy  = sum(y)
    sxy = sum([i*j for (i, j) in zip(x, y)])

    a1 = (n*sxy - sx*sy)/(n*sx2 - sx**2)
    a0 = yavg - a1*xavg

    sy2 = sum([i*i for i in y])
    r = (n*sxy - sx*sy)/(math.sqrt(n*sx2-sx**2)*math.sqrt(n*sy2-sy**2))

    return ([a0, a1], r)

def voltage_sample(lj):
    channel             = 0
    resolution_index    = 4
    gain_index          = 0
    settling_factor     = 0
    differential        = False

    cmd = u6.AIN24( channel,
                    resolution_index,
                    gain_index,
                    settling_factor,
                    differential)

    slope  = lj.calInfo.ain10vSlope
    offset = lj.calInfo.ain10vOffset
    # Following U6 data sheet directions for calculating AIN value
    # All readings and the calibration constants are 16-bit aligned.
    # This means that 24-bit values must be justified to 16-bit values before applying a calibration.
    raw = float(lj.getFeedback(cmd)[0])/256.0
    return (slope * raw) + offset

def write_to_file(results):
    filename = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/force_calibration_data.csv"
    with open(filename, 'w') as f:
        f.write("current,force,voltage\n")
        for val in results:
            f.write("{},{},{}\n".format(val[0], val[1], val[2]))

def plot_results(results):
    plt.figure()

    # Plot Raw Results
    c = [x[0] for x in results]
    f = [x[1] for x in results]
    plt.plot(c, f, "bo")

    # Linear fit with data (Include equation on plot)
    (a, r) = linreg(c, f)
    c_r = np.linspace(min(c), max(c))
    f_r = [x*a[1] + a[0] for x in c_r]
    plt.plot(c_r, f_r, 'r-')

    eq = r'y={0:.3f}*x + {1:.3f}'.format(a[1], a[0])
    plt.text(min(c_r)+0.2*(max(c_r)-min(c_r)), min(f_r)+0.9*(max(f_r)-min(f_r)), eq, fontsize='large', color='k')

    # Miscellaneous plot formatting
    plt.title("Motor Force Curve")
    plt.xlabel("Current (Amps)")
    plt.ylabel("Force (N)")
    plt.subplots_adjust(left=0.1, right=0.97, top=0.9)
    plt.grid()
    plt.show()

def delay(x):
    start_time = time.time()
    rate = rospy.Rate(20)
    while(time.time() < start_time + 2.0 and not rospy.is_shutdown()):
        rate.sleep()

#-------------------------------#
#--           MAIN            --#
#-------------------------------#


# Setup:
rospy.init_node("force_test", anonymous=False)
mtr = RosMotor() # Interface to benchtop motor
mtr.enable_motor()
lj = u6.U6() # LabJack with force sensor.
lj.getCalibrationData()


# Weird bug I'm too lazy to figure out. First drive current command
# isn't be published correctly. So just publish an initial command
# of zero that can be safely ignore.
mtr.drive_current(0.0)
delay(2.0)

NUM_CYCLES   = 30 # Number of times to test each current point
results      = [] # Will fill with data points in format (Current, Force, Voltage)

AMPS_TO_TEST = np.linspace(-1.2, 1.2, 31)

for i in range(NUM_CYCLES):
    for amp in AMPS_TO_TEST:
        # Turn on motor at target current
        # Wait short period before taking a current and force measurement
        mtr.drive_current(amp)
        delay(0.5)

        current = mtr.read_current()
        volts = voltage_sample(lj)
        results.append((current, v2N(volts), volts))
        delay(0.1)
        current = mtr.read_current()
        volts = voltage_sample(lj)
        results.append((current, v2N(volts), volts))

        # Drive with 0 current to avoid constant motor on time.
        mtr.drive_current(0.0)
        delay(0.9)

# Disable Motor
mtr.disable_motor()

# Save & Plot Results
write_to_file(results)
plot_results(results)
