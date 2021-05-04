#!/usr/bin/env python3

"""
File:   first.py
Author: pdsherman
Date:   April 2021
"""

import math
import numpy as np
import control
import control.matlab
import sqlite3

import matplotlib
import matplotlib.pyplot as plt

def get_real_data(table_name, test_name, filter=None):
    data = {"time": [], "force": [], "position": []}
    database = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/PendulumDatabase.db"
    command  = "SELECT * FROM {} where test = \"{}\"".format(table_name, test_name)
    if filter:
        command += filter

    conn = sqlite3.connect(database)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    c.execute(command)

    for row in c:
        data["time"].append(row["test_time"])
        data["force"].append(row["input"])
        data["position"].append(row["x"])

    return data

m = 0.617
b = 40.1

num = [1]
den = [m, b, 0]
sys = control.TransferFunction(num, den)


t = np.linspace(0.0, 3.0, 150)
u = []
for i in range(150):
    if(t[i] > 0.5 and t[i] < 1.0):
        u.append(25.0)
    else:
        u.append(0.0)

(T, xout) = control.forced_response(sys, t, u)

d1 = get_real_data("BaseModelTest", "test1", " AND x >= 0.0 AND x < 1.1 AND test_time <= 3.0")
d2 = get_real_data("BaseModelTest", "test2", " AND x >= 0.0 AND x < 1.1 AND test_time <= 3.0")
d3 = get_real_data("BaseModelTest", "test3", " AND x >= 0.0 AND x < 1.1 AND test_time <= 3.0")

fig, ax1 = plt.subplots()
ax1.plot(T, xout)
ax1.set_ylabel("Position (m)", color='b')

ax2 = ax1.twinx()
ax2.plot(T, u, 'm')
ax2.set_ylabel("Input (N)", color="m")

fig.tight_layout()

fig2, ax3 = plt.subplots()
ax3.plot(d1["time"], d1["position"])
ax3.plot(d2["time"], d2["position"], 'r')
ax3.plot(d3["time"], d3["position"], 'g')
ax3.set_ylabel("Position (m)", color='b')

ax4 = ax3.twinx()
ax4.plot(d1["time"], d1["force"], 'm')
ax4.set_ylabel("Input (N)", color="m")

fig.tight_layout()


plt.show()
