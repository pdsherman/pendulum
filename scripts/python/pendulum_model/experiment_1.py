#!/usr/bin/env python3

"""
File:   experiment_1.py
Author: pdsherman
Date:   May. 2020
"""

import math
import time
import argparse
import sqlite3

import matplotlib.pyplot as plt

Mass    = 0.222    # Mass of pendulum (kilogram)
Length  = 0.254    # Length to midpoint (meters)
Gravity = 9.80665  # Acceleration from gravity (m/s^2)
Inertia = 0.01978    # Intertia of Swinging pendulum
b_p     = 0.0      # Friction Coefficient

def x_dot_equations(x):
    x_dot = [0.0] * 2
    x_dot[0] = x[1]

    c = 1.0/(Mass*Length*Length + Inertia)
    x_dot[1] = c*(-2.0*b_p*x[1] - Mass*Gravity*Length*math.cos(x[0]))

    return x_dot

def RK_optimal(h, x0, print_flag = False):
        A = [[0.0, 0.0, 0.0],
             [0.4, 0.0, 0.0],
             [0.29697761, 0.15875964, 0.0],
             [0.21810040, -3.05096516, 3.83286476]]
        B = [0.17476028, -0.55148066, 1.20553560, 0.17118478]
        return RK(h, x0, A, B, print_flag)

def RK(h, x0, A, B, print_flag):
        n  = len(x0)
        x  = [0.0] * n
        x1 = [0.0] * n
        f = []

        if print_flag:
            print("-"*51)
            print("{:^5} | {:^9} {:^9} | {:^9} {:^9} |".format("Stage", "X0", "X1", "f0", "f1"))
            print("-"*51)

        for i in range(len(B)):
            dx = [0.0] * n
            for j in range(i):
                for k in range(n):
                    dx[k] += A[i][j]*f[j][k]

            for k in range(n):
                x[k] = x0[k] + h*dx[k]

            x_dot = x_dot_equations(x)
            f.append(x_dot)

            if print_flag:
                print_line(i+1, x, f[-1])

        if print_flag:
            print("-"*51)

        # Explicit Equation
        dx = [0.0, 0.0]
        for i in range(len(B)):
            for k in range(n):
                dx[k] += B[i]*f[i][k]

        for k in range(n):
            x1[k] = x0[k] + h*dx[k]

        return x1

def get_sensor_data():
    data = {"t": [], "x": [], "theta": []}

    database = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/EncoderData.db"
    conn = sqlite3.connect(database)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()

    command = "SELECT * FROM {}".format("TestTable")
    command += " where test_time_s > {} and test_time_s < {} and theta < 10.0".format(
            5.26, 65.5)
    command += " ORDER BY timestamp"

    c.execute(command)
    for row in c:
        data["t"].append(row['test_time_s'])
        data["x"].append(row['x'])
        data["theta"].append(row['theta'])
    data["t"] = [x - data["t"][0] for x in data["t"]]

    conn.close()
    return data

# ********************** #
# **  Start of Script ** #
# ********************** #

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pendulum Parameter")
    parser.add_argument('-I', required=False, type=float, default=0.01, help="Inertia")
    parser.add_argument('-b', required=False, type=float, default=0.00, help="Friction")
    args = parser.parse_args()

    h       = 0.01
    Inertia = args.I
    b_p     = args.b

    sensor_data = get_sensor_data()
    T_MAX = sensor_data["t"][-1]

    t     = [0.0]
    x     = [sensor_data["theta"][0], 0.0]
    theta = [x[0]]
    while True:
        if(t[-1] + h > T_MAX):
            break
        t.append(t[-1] + h)
        x = RK_optimal(h, x)
        theta.append(x[0])

    plt.figure(figsize=[18.0, 4.8])
    plt.plot(sensor_data["t"], sensor_data["theta"],'r')
    plt.plot(t, theta,'b')
    plt.tight_layout()
    plt.show()
