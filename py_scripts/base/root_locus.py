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


# ------------------------------- #
# ---  System Constants       --- #
# ------------------------------- #

m = 0.61732
b = 40.1

num = [1.0/m]
den = [1.0, b/m, 0.0]
G = control.TransferFunction(num, den)

num = [1.0, 50.0]
den = [1.0, 5.0]
D   = control.TransferFunction(num, den)

L = control.series(D, G)

rl = control.matlab.rlocus(L, plot=True)
plt.show()
