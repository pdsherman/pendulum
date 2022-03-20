#!/usr/bin/env python3

"""
File:   run_test.py
Author: pdsherman
Date:   March. 2021
"""

from pendulum.srv import LoggingStart, LoggingStartRequest, LoggingStartResponse
from pendulum.srv import LoggingStop, LoggingStopRequest, LoggingStopResponse
from pendulum.srv import LoggingBufferCheck, LoggingBufferCheckRequest, LoggingBufferCheckResponse
from pendulum.msg import LoggingData

import u6                 # LabJack python library (loadcell sampling)
from RosMotor import *    # Interfacing with benchtop motor

import numpy as np
import matplotlib.pyplot as plt

import argparse
import sys
import time
import math

def v2N(volt):
    return 38.697*volt - 238.027

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

def logging_stop(table_name):
    log_stop_name = "/sqlite/stop_log"
    try:
        rospy.wait_for_service(log_stop_name, timeout=5)
        rospy.ServiceProxy(log_stop_name, LoggingStop)(table_name)
    except:
        rospy.logwarn("Unable to stop logging service.")
        return False
    return True

def logging_start(table_name):
    log_start_name = "/sqlite/start_log"
    topic_name = "/force_values"
    header     = ["trial", "timestamp", "current", "force", "voltage"]

    try:
        if logging_stop(table_name):
            rospy.wait_for_service(log_start_name, timeout=5)
            resp = rospy.ServiceProxy(log_start_name, LoggingStart)(table_name, topic_name, header)
            if resp.success:
                return rospy.Publisher(topic_name, LoggingData, queue_size=100)
    except:
        pass

    rospy.logwarn("Unable to start logging service.")
    return None


def take_sample(lj, mtr, msg, pub, test_name):
    # Sample
    timestamp = time.time()
    current   = mtr.read_current()
    volts     = voltage_sample(lj)
    force     = v2N(volts)

    # Record
    msg.header.stamp = rospy.Time.now()
    msg.header.seq  += 1
    msg.test_name    = test_name
    msg.data         = [timestamp, current, force, volts]

    pub.publish(msg)

#-------------------------------#
#--           MAIN            --#
#-------------------------------#

parser = argparse.ArgumentParser(description="Pendulum Assembly Force Test")
parser.add_argument('test_name', default="force_test")
parser.add_argument('--min_amp', required=False, type=float, default=-1.0, help="Minimum amps")
parser.add_argument('--max_amp', required=False, type=float, default=1.0, help="Maximum amps")
parser.add_argument('--num_steps', required=False, type=int, default=5, help="Number of steps in amp")
parser.add_argument('--num_cycles', required=False, type=int, default=20, help="How many cyles per amp")
args = parser.parse_args()

# Setup:
rospy.init_node("force_test", anonymous=False)
mtr = RosMotor() # Interface to benchtop motor
mtr.enable_motor()
lj = u6.U6() # LabJack with force sensor.
lj.getCalibrationData()

table_name = "MotorForceTest"
pub = logging_start(table_name)
if(pub == None):
    sys.exit(0)

msg = LoggingData()

# Weird behavior I'm too lazy to figure out. First drive current command
# isn't be published correctly. So just publish an initial command
# of zero that can be safely ignored.
mtr.drive_current(0.0)
delay(2.0)

NUM_CYCLES   = args.num_cycles # Number of times to test each current point
AMPS_TO_TEST = np.linspace(args.min_amp, args.max_amp, args.num_steps)

for i in range(NUM_CYCLES):
    for amp in AMPS_TO_TEST:
        # Turn on motor at target current
        # Wait short period before taking a current and force measurement
        mtr.drive_current(amp)
        delay(0.5)

        # Take 2 samples just because
        take_sample(lj, mtr, msg, pub, args.test_name)
        delay(0.1)
        take_sample(lj, mtr, msg, pub, args.test_name)

        # Drive with 0 current to avoid constant motor on time.
        mtr.drive_current(0.0)
        delay(0.8)

    take_sample(lj, mtr, msg, pub, args.test_name)
    delay(0.1)

# Disable Motor
mtr.disable_motor()

# Stop Logging
logging_stop(table_name)
