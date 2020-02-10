#!/user/bin/env python3
"""
File:   ImageCanvas.py
Author: pdsherman
Date:   Jan. 2020

Description: Canvas to draw the representation of the pendulum
"""

from tkinter import *

from PendulumImage import PendulumImage

class ImageCanvas(Canvas):
    def __init__(self, parent=None, width=350, height=300, **options):
        Canvas.__init__(self, parent, **options)

        self.config(width=width, height=height, bd=3, relief=RIDGE)
        self.config(bg='#333333')
        self.pack(fill=BOTH, expand=True, side=LEFT)

        # Collection of pendulums to display
        self.pendulums = {}

    def add_pendulum(self, name, x0, theta0):
        pendulum = PendulumImage(x0, theta0, name)
        base_id = self.create_polygon(pendulum.get_base_points(), fill='red')
        pend_id = self.create_polygon(pendulum.get_pendulum_points(), fill='blue')
        circle_id = self.create_oval(pendulum.get_rotation_circle_points(), fill='green')
        self.pendulums[name] = {"base_id":base_id,
                    "pend_id":pend_id,
                    "circle_id":circle_id,
                    "pendulum":pendulum}

    def update_drawing(self):
        for key in self.pendulums:
            pendulum = self.pendulums[key]["pendulum"]
            self.coords(self.pendulums[key]["base_id"], pendulum.get_base_points())
            self.coords(self.pendulums[key]["pend_id"], pendulum.get_pendulum_points())
            self.coords(self.pendulums[key]["circle_id"], pendulum.get_rotation_circle_points())
