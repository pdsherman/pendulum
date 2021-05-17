#!/usr/bin/env python3

"""
File:   ode.py
Author: pdsherman
Date:   May 2021
"""

import math
import numpy as np

import matplotlib
import matplotlib.pyplot as plt

dr  = 0.05
wn = 5.5
y0 = 5.0

acdr = math.acos(dr)
sdr  = math.sqrt(1.0 - dr**2)
wd   = wn * sdr
a    = dr*wn

alpha = 2*dr*wn
beta  = wn**2.0

def y_actual(t):
    n1 = math.sin(wd*t + acdr)
    n2 = math.exp(-a*t)/sdr
    return y0*n1*n2

def x_dot_equations(x):
    return [x[1], -alpha*x[1]-beta*x[0]]

def print_line(s, x, f):
    print("{:^5} | {:^9.4f} {:^9.4f} | {:^9.4f} {:^9.4f} |".format(s, x[0], x[1], f[0], f[1]))

def RK3(h, x0, print_flag = False):
        A = [[0.0, 0.0], [0.5, 0.0], [-1.0, 2.0]]
        B = [1./6.0, 2.0/3.0, 1.0/6.0]
        return RK(h, x0, A, B, print_flag)

def RK4(h, x0, print_flag = False):
        A = [[0.0, 0.0, 0.0],
             [0.5, 0.0, 0.0],
             [0.0, 0.5, 0.0],
             [0.0, 0.0, 1.0]]
        B = [1./6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0]
        return RK(h, x0, A, B, print_flag)

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

## ------------------------- ##
## --    SCRIPT START     -- ##
## ------------------------- ##
if __name__ == "__main__":
    plt.figure()
    T_MAX = 10.0
    t = np.linspace(0.0, T_MAX, 250)

    # Actual solution
    y = [y_actual(x) for x in t]

    # Runga-Kutta
    t_rk = [0.0]

    x_rk3   = [y0, 0.0]
    y_rk3   = [x_rk3[0]]

    x_rk4 = [y0, 0.0]
    y_rk4 = [x_rk4[0]]

    x_rk_opt = [y0, 0.0]
    y_rk_opt = [x_rk_opt[0]]

    h = 0.08
    while True:
        if(t_rk[-1] + h > T_MAX):
            break
        t_rk.append(t_rk[-1] + h)
        y_t = y_actual(t_rk[-1])

        x_rk3 = RK3(h, x_rk3)
        y_rk3.append(x_rk3[0])

        x_rk4 = RK4(h, x_rk4)
        y_rk4.append(x_rk4[0])

        x_rk_opt = RK_optimal(h, x_rk_opt)
        y_rk_opt.append(x_rk_opt[0])


    print("\n")
    print("3-stage: {:.7f}".format(y_rk3[-1]))
    print("4-stage: {:.7f}".format(y_rk4[-1]))
    print("Optimal: {:.7f}".format(y_rk_opt[-1]))

    # Display
    plt.plot(t, y)
    plt.plot(t_rk, y_rk3, 'r-o', markersize=3)
    plt.plot(t_rk, y_rk4, 'g-o', markersize=3)
    plt.plot(t_rk, y_rk_opt, 'k-o', markersize=3)
    plt.show()
