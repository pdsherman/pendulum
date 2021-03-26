#!/usr/bin/env python3

"""
File:   calc_control_law.py
Author: pdsherman
Date:   March. 2021
"""
import system

import math
import numpy as np
# from control.matlab import *
import control
import control.matlab

def display_complex(nums):
    print("-"*20)
    print("{:^10}{:^10}".format("real", "imag"))
    print("-"*20)
    for x in nums:
        if isinstance(x, complex):
            print("{0.real:<+10.5f} {0.imag:<+6.5f}j".format(x))
        else:
            print("{:<+10.5f}".format(x))
    print("-"*20)

def calc_ref_input_gains(F, G, H, J):
    A = np.vstack((np.hstack((F, G)), np.hstack((H, J))))
    B=np.zeros((A.shape[0], 1))
    B[-1] = 1
    N = np.linalg.inv(A) @ B
    return (N[:-1], Nu[-1])

#-------------------------------#
#--           MAIN            --#
#-------------------------------#

if __name__ == "__main__":

    sys = system.System()
    (F, G, H, J) = sys.state_space()
    sys = control.matlab.ss(F, G, H, J)
    print(control.matlab.pole(sys))

    # Find current poles of system
    vals, vecs = np.linalg.eig(F)
    print("Poles of System:")
    display_complex(vals)


    # s = np.array([-2*w0+0.j, -2*w0-0.j], dtype=complex)

    # CALCULATE RESULTS

    # (Nx, Nu) = calc_ref_input_gains(F, G, H, J)
    # K = acker(F, G, s)

    # print("--- Calculated Results ---")
    # print(K:\n{}".format(str(K)))
    # print(Nx:\n{}".format(str(Nx)))
    # print(Nu:\n{}".format(str(Nu)))
