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

import matplotlib
import matplotlib.pyplot as plt


m = 0.61732
b = 10.5

num = [1]
den = [m, b, 0]
sys = control.TransferFunction(num, den)


t = np.linspace(0.0, 3.0, 150)
u = []
for i in range(150):
    if(t[i] > 0.5 and t[i] < 1.0):
        u.append(15.0)
    else:
        u.append(0.0)

(T, xout) = control.forced_response(sys, t, u)

fig, ax1 = plt.subplots()
ax1.plot(T, xout)
ax1.set_ylabel("Position (m)", color='b')

ax2 = ax1.twinx()
ax2.plot(T, u, 'm')
ax2.set_ylabel("Input (N)", color="m")

fig.tight_layout()
plt.show()
