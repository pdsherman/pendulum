#!/usr/bin/env python3
"""
File:   PendulumImage.py
Author: pdsherman
Date:   Jan. 2020
"""

from math import cos, sin, radians, pi

# meter to pixel scaling
SCALING  = 1000.0
X_OFFSET = 100.0

def get_corner_points(center, width, height, theta=0.0):
    # Top Right Corner
    tr_x = center[0] + 0.5*width*cos(theta) + 0.5*height*cos(theta + pi/2.0)
    tr_y = center[1] - 0.5*width*sin(theta) - 0.5*height*sin(theta + pi/2.0)

    # Bottom Right Corner
    br_x = tr_x + height*cos(theta - pi/2.0)
    br_y = tr_y - height*sin(theta - pi/2.0)

    # Top Left Corner
    tl_x = tr_x - width*cos(theta)
    tl_y = tr_y + width*sin(theta)

    # Bottom Left Corner
    bl_x = br_x - width*cos(theta)
    bl_y = br_y + width*sin(theta)

    return [tr_x, tr_y, br_x, br_y, bl_x, bl_y, tl_x, tl_y]

class RectangleImage:
    def __init__(self, x0, width, height):
        self.width  = width
        self.height = height
        self.y      = 275.0

        self.set_state(x0)

    def set_state(self, x):
        self.x = x[0]*SCALING + X_OFFSET

    def get_corner_points(self):
        return get_corner_points([self.x, self.y], self.width, self.height)

    def get_state(self):
        return self.x

class PendulumImage:
    def __init__(self, x0):
        # State (x, theta)
        self.x = [0.0, 0.0]
        self.set_state(x0)

        # Dimensions (pixels) for my fake pendulum
        self.length = 245.0
        self.width  = 15.0

        # Dimensions (pixels) for base object
        self.base_width  = 80.0
        self.base_height = 30.0
        self.base_y      = 275.0

    def pendulum_center(self):
        x_p = self.x[0] + 0.45*self.length*cos(self.x[1])
        y_p = self.base_y - 0.45*self.length*sin(self.x[1])
        return [x_p, y_p]

    def get_base_points(self):
        return get_corner_points([self.x[0], self.base_y], self.base_width, self.base_height)

    def get_pendulum_points(self):
        return get_corner_points(self.pendulum_center(), self.length, self.width, self.x[1])

    def get_rotation_circle_points(self):
        radius = 5
        return [self.x[0]-radius, self.base_y-radius, self.x[0]+radius, self.base_y+radius]

    def set_state(self, x):
        self.x[0] = x[0]*SCALING + X_OFFSET
        self.x[1] = x[1]


class SpringImage:
    def __init__(self, x0, y):
        self.l1 = 20
        self.y  = y
        self.h  = 15

        self.x = [0.0, 0.0]
        self.set_state(x0)

    def set_state(self, x):
        if x[0] < x[1]:
            self.x[0] = x[0]*SCALING + X_OFFSET
            self.x[1] = x[1]*SCALING + X_OFFSET
        else:
            self.x[0] = x[1]*SCALING + X_OFFSET
            self.x[1] = x[0]*SCALING + X_OFFSET

    def get_points(self):
        L = self.x[1] - self.x[0]
        p = [self.x[0], self.y]

        if L > 2*self.l1:
            p += [self.x[0]+self.l1, self.y]
            l2 = (L - 2*self.l1)/9
            for i in range(1, 9):
                x = self.x[0] + self.l1 + i*l2
                y = self.h if (i%2 == 0) else -self.h
                p += [x, y+self.y]
            p += [self.x[1]-self.l1, self.y]
        else:
            x_mid = (self.x[1] + self.x[0])/2
            p += [x_mid, self.y, x_mid, self.y+self.h, x_mid, self.y-self.h, x_mid, self.y]

        p += [self.x[1], self.y]
        return p

    def get_state(self):
        return self.x
