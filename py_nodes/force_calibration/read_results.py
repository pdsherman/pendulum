#!/usr/bin/env python3

"""
File:   run_test.py
Author: pdsherman
Date:   March. 2021
"""

import sys
import time
import math

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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

def read_from_file():
    filename = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/force_calibration/force_calibration_data_1.csv"
    df = pd.read_csv(filename, sep=",", header=0)
    return df

def plot_results(c1, f1, c2, f2):
    plt.figure()

    f1 = [x-12.5 for x in f1]
    f2 = [x-12.5 for x in f2]

    # Plot Raw Results
    plt.plot(c1, f1, "bo")
    plt.plot(c2, f2, "bo")

    # Linear fit with data (Include equation on plot)
    for i in range(2):
        if i == 0:
            current = c1; force = f1
            (a, r) = linreg(current, force)
            c_r = np.linspace(min(current), 0.0)
            f_r = [x*a[1] + a[0] for x in c_r]
        elif i == 1:
            current = c2; force = f2
            (a, r) = linreg(current, force)
            c_r = np.linspace(0.0, max(current))
            f_r = [x*a[1] + a[0] for x in c_r]


        plt.plot(c_r, f_r, 'r-')

        eq = r'y={0:.3f}*x + {1:.3f}'.format(a[1], a[0])
        plt.text(min(c_r)+0.4*(max(c_r)-min(c_r)), min(f_r)+(0.9)*(max(f_r)-min(f_r)), eq, fontsize='large', color='k')

    # Miscellaneous plot formatting
    plt.title("Motor Force Curve")
    plt.xlabel("Current (Amps)")
    plt.ylabel("Force (N)")
    plt.subplots_adjust(left=0.1, right=0.97, top=0.9)
    plt.grid()
    plt.show()

#-------------------------------#
#--           MAIN            --#
#-------------------------------#

df = read_from_file()
sec1 = df[df["current"] < -0.35][df["current"] > -1.5]
sec2 = df[df["current"] > 0.0]
plot_results(sec1["current"].tolist(), sec1["force"].tolist(), sec2["current"].tolist(), sec2["force"].tolist())
