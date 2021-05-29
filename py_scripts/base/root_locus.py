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


def leadlag():
    m = 0.61732
    b_b = 40.1

    num = [1.0/m]
    den = [1.0, b_b/m, 0.0]
    G = control.TransferFunction(num, den)

    K = 2.5
    a = 10.0
    b = 5.0
    T = 0.05

    num = [K, K*a]
    den = [1.0, b]

    D   = control.TransferFunction(num, den)
    D_discrete = control.matlab.c2d(D, T, method="backward_diff")

    print("Continuous System")
    print(D)

    print("Discrete Lag Controller")
    print(D_discrete)


def pid():

    Kp = 31.5
    Ki = 50.0
    Kd = 5.0
    T = 0.05

    num = [Kd, Kp, Ki]
    den = [0.0, 1.0, 0.0]

    D   = control.TransferFunction(num, den)
    D_discrete = control.matlab.c2d(D, T, method="bilinear")

    print("Continuous System")
    print(D)

    print("Discrete Lag Controller")
    print(D_discrete)

    # print("Calculated")
    # x1 = K*( 2.0 + a*T)
    # x2 = K*(-2.0 + a*T)
    # x3 =  2.0 + b*T
    # x4 = (-2.0 + (b*T)
    # c = x3
    # x1 /= c
    # x2 /= c
    # x3 /= c
    # x4 /= c
    #
    # print("{:.2f} z + {:.2f}".format(x1, x2))
    # print("----------------")
    # print("   z + {:.2f}".format(x4))
# ------------------------------- #
# ---  System Constants       --- #
# ------------------------------- #

leadlag()
