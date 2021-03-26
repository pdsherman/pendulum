#!/usr/bin/env python3

"""
File:   system.py
Author: pdsherman
Date:   March. 2021
"""

import numpy as np

class System:
    def __init__(self):
        # Parameters of system
        self.M   = 5.5
        self.b_b = 1.0

        self.m   = 2.7
        self.l   = 0.5
        self.I   = 0.00474
        self.b_p = 0.00061

        # Real world constants
        self.g = 9.80665

    def state_space(self):
        d = self.M + self.m
        k = self.m*self.l
        w = self.b_p/self.l
        z = self.m*self.l + self.I/self.l
        den = d*z - self.m*k

        F = np.zeros((4,4), dtype=np.float64)
        # Row 0
        F[0][1] = 1
        # Row 1
        F[1][1] = -z*self.b_b/den
        F[1][2] = self.m*k*self.g
        F[1][3] = (2*k+z)*w/den
        # Row 2
        F[2][3] = 1
        # Row 3
        F[3][1] = -self.m*self.b_b/den
        F[3][2] = self.m*d*self.g/den
        F[3][3] = -(2*d + self.m)*w/den

        G = np.zeros((4, 1), dtype=np.float64)
        G[1][0] = z / den
        G[3][0] = self.m / den

        H = np.zeros((1, 4), dtype=np.float64)
        H[0][2] = 1

        J = np.zeros((1, 1), dtype=np.float64)

        return (F, G, H, J)
