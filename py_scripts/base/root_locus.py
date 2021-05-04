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

M = 0.61732
b_b = 45.1

num = [1]
den = [m, b, 0]
sys = control.TransferFunction(num, den)
