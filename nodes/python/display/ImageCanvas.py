#!/user/bin/env python3
"""
File:   ImageCanvas.py
Author: pdsherman
Date:   Jan. 2020

Description: Canvas to draw the representation of the pendulum
"""

from tkinter import *

from SystemImages import PendulumImage, MassOnlyImage

class ImageCanvas(Canvas):
    def __init__(self, parent=None, width=350, height=300, **options):
        Canvas.__init__(self, parent, **options)

        self.config(width=width, height=height, bd=3, relief=RIDGE, bg='#333333')
        self.pack(fill=BOTH, expand=True, side=LEFT)

        # Single Line Showing x-axis
        self.create_line(0, 275, 1000, 275, fill='green', dash=[20, 10])

        # Collection of pendulums to display
        self.pendulums = {}

        # Collection of mass only objects to display
        self.masses = {}

        # Names of all objects to display
        self.legend = {}

    def add_image(self, img_type, name, x0, theta0, base_fill, pend_fill):
        if(self.pendulums.get(name) != None or self.masses.get(name) != None):
            return False

        if(img_type == "pendulum"):
            # Add pendulum
            pendulum = PendulumImage(x0, theta0, name)
            base_id = self.create_polygon(pendulum.get_base_points(), fill=base_fill)
            pend_id = self.create_polygon(pendulum.get_pendulum_points(), fill=pend_fill)
            circle_id = self.create_oval(pendulum.get_rotation_circle_points(), fill='green')
            self.pendulums[name] = {"base_id":base_id,
                        "pend_id":pend_id,
                        "circle_id":circle_id,
                        "pendulum":pendulum}
        elif(img_type == "mass_only"):
            # Add mass object
            mass = MassOnlyImage(x0, name)
            base_id = self.create_polygon(mass.get_base_points(), fill=base_fill)
            self.masses[name] = {"base_id":base_id, "mass":mass}
        else:
            return False

        # Add to legend
        place = len(self.legend)+1
        y_offset = 25*place
        text_id = self.create_text(30, y_offset, anchor=W, fill="white", text=name)
        box_id = self.create_rectangle([15, y_offset-5, 25, y_offset+5], fill=pend_fill)
        self.legend[name] = {"text_id": text_id, "box_id": box_id, "place": place}

        return True

    def remove_image(self, name):
        # Delete items from canvas first, then remove from dict
        if(self.pendulums.get(name) != None):
            self.delete(self.pendulums[name]["base_id"])
            self.delete(self.pendulums[name]["pend_id"])
            self.delete(self.pendulums[name]["circle_id"])
            del self.pendulums[name]
        elif(self.masses.get(name) != None):
            self.delete(self.masses[name]["base_id"])
            del self.masses[name]
        else:
            return False

        self.delete(self.legend[name]["text_id"])
        self.delete(self.legend[name]["box_id"])
        place = self.legend[name]["place"]
        del self.legend[name]

        for key in self.legend:
            if(self.legend[key]["place"] > place):
                self.move(self.legend[key]["text_id"], 0, -25)
                self.move(self.legend[key]["box_id"], 0, -25)
                self.legend[key]["place"] -= 1

        return True

    def update_drawing(self):
        for key in self.pendulums:
            pendulum = self.pendulums[key]["pendulum"]
            self.coords(self.pendulums[key]["base_id"], pendulum.get_base_points())
            self.coords(self.pendulums[key]["pend_id"], pendulum.get_pendulum_points())
            self.coords(self.pendulums[key]["circle_id"], pendulum.get_rotation_circle_points())
        for key in self.masses:
            mass = self.masses[key]["mass"]
            self.coords(self.masses[key]["base_id"], mass.get_base_points())
