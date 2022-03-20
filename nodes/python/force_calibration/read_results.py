#!/usr/bin/env python3

"""
File:   run_test.py
Author: pdsherman
Date:   March. 2021
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import sys
import time
import math
import sqlite3

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

def read_from_file(table_name, trial_name, filters = []):

    data = {"current": [], "force": []}
    database = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/PendulumDatabase.db"
    conn = sqlite3.connect(database)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()

    command  = "SELECT * FROM {}".format(table_name)
    command += " where trial = \"{}\"".format(trial_name)
    for f in filters:
        command += f
    command += " ORDER BY current"

    c.execute(command)
    for row in c:
        data["current"].append(row["current"])
        data["force"].append(row["force"])

    return data

def force_to_current(x):
    if x > 0.0:
        return 0.0156 * x + 0.105
    elif x < 0.0:
        return 0.0156 * x - 0.105
    else:
        return 0.0

#-------------------------------#
#--           MAIN            --#
#-------------------------------#

data1 = read_from_file("MotorForceTest", "trial2", [" AND current < -0.1"])
data2 = read_from_file("MotorForceTest", "trial2", [" AND current > 0.1"])

data = read_from_file("MotorForceTest", "trial3", [" AND current > -0.25", " AND current < 0.25"])
data3 = {"current": [], "force": []}
for (c, f) in zip(data["current"], data["force"]):
    if not (abs(c) < 0.05 and f > 5):
        data3["current"].append(c)
        data3["force"].append(f)

plt.figure()

plt.plot(data3["force"], data3["current"], "bo")
plt.plot(data1["force"], data1['current'], "bo")
plt.plot(data2["force"], data2["current"], "bo")

x_r = np.linspace(-100.0, 100.0, 250)
y_r = [force_to_current(z) for z in x_r]
plt.plot(x_r, y_r, "r-")

plt.title("Motor Force Curve")
plt.ylabel("Current (Amps)")
plt.xlabel("Force (N)")

plt.tight_layout()
plt.grid()
plt.show()
