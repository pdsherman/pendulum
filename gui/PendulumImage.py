#!/usr/bin/env python3
"""
File:   PendulumImage.py
Author: pdsherman
Date:   Jan. 2020
"""

import rospy
from math import cos, sin, radians, pi

from pendulum.msg import State

class PendulumImage:
    def __init__(self, x0, theta0, name):

        # Dimensions (pixels) for base object
        self.base_width  = 80.0
        self.base_height = 30.0

        # Vertical position (pixels) of moving base
        self.base_y = 275.0

        # Dimensions (pixels) for my fake pendulum
        self.length = 245.0
        self.width  = 15.0

        # State variable x is in meters
        # Sacling converts x to pixels (1 m = 500 pixels)
        self.scaling = 500.0

        # Iitialize State
        set_state(x0, theta0)

        # ROS subscriber for updates to state
        self.sub = rospy.Subscriber(name, State, self.set_state)

    def set_state(self, state):
        self.x = state.x*self.scaling

        # Theta comes in radians
        self.theta = state.theta

    def get_state(self):
        return State(x=self.x, theta=self.theta)

    def pendulum_center(self):
        x_p = self.x + 0.45*self.length*cos(self.theta)
        y_p = self.base_y - 0.45*self.length*sin(self.theta)
        return [x_p, y_p]

    def get_base_points(self):
        return self.get_points([self.x, self.base_y], self.base_width, self.base_height)

    def get_pendulum_points(self):
        return self.get_points(self.pendulum_center(), self.length, self.width, self.theta)

    def get_rotation_circle_points(self):
        radius = 5
        return [self.x-radius, self.base_y-radius, self.x+radius, self.base_y+radius]

    def get_points(self, center, width, height, theta=0.0):

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
