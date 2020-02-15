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

        # Single Line Showing x-axis
        self.create_line(0, 275, 1000, 275, fill='green', dash=[20, 10])

        # Collection of pendulums to display
        self.pendulums = {}

        # Names of all pendulums to display
        self.legend = {}

    def add_pendulum(self, name, x0, theta0, base_fill, pend_fill):
        if(self.pendulums.get(name) != None):
            return False

        # Add pendulum
        pendulum = PendulumImage(x0, theta0, name)
        base_id = self.create_polygon(pendulum.get_base_points(), fill=base_fill)
        pend_id = self.create_polygon(pendulum.get_pendulum_points(), fill=pend_fill)
        circle_id = self.create_oval(pendulum.get_rotation_circle_points(), fill='green')
        self.pendulums[name] = {"base_id":base_id,
                    "pend_id":pend_id,
                    "circle_id":circle_id,
                    "pendulum":pendulum}

        # Add to legend
        place = len(self.legend)+1
        y_offset = 25*place
        text_id = self.create_text(30, y_offset, anchor=W, fill="white", text=name)
        box_id = self.create_rectangle([15, y_offset-5, 25, y_offset+5], fill=pend_fill)
        self.legend[name] = {"text_id": text_id, "box_id": box_id, "place": place}

        return True

    def remove_pendulum(self, name):
        if(self.pendulums.get(name) == None):
            return False

        # Delete items from canvas first, then remove from dict
        self.delete(self.pendulums[name]["base_id"])
        self.delete(self.pendulums[name]["pend_id"])
        self.delete(self.pendulums[name]["circle_id"])
        del self.pendulums[name]

        self.delete(self.legend[name]["text_id"])
        self.delete(self.legend[name]["box_id"])
        place = self.legend[name]["place"]
        del self.legend[name]

        for key in self.legend:
            if(self.legend[key]["place"] > place):
                self.move(self.legend[key]["text_id"], 0, -25)
                self.move(self.legend[key]["box_id"], 0, -25)
                self.legend[key]["place"] -= 1

    def update_drawing(self):
        for key in self.pendulums:
            pendulum = self.pendulums[key]["pendulum"]
            self.coords(self.pendulums[key]["base_id"], pendulum.get_base_points())
            self.coords(self.pendulums[key]["pend_id"], pendulum.get_pendulum_points())
            self.coords(self.pendulums[key]["circle_id"], pendulum.get_rotation_circle_points())
