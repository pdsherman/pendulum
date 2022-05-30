#!/user/bin/env python3
"""
File:   ImageCanvas.py
Author: pdsherman
Date:   Jan. 2020

Description: Canvas to draw stuff on
"""

from tkinter import *

import rospy
from SystemImages import PendulumImage, RectangleImage, SpringImage
from pendulum.srv import DrawSystem, DrawSystemResponse, DrawSystemRequest
from pendulum.msg import State

class ImageCanvas(Canvas):
    def __init__(self, parent=None, width=350, height=300, **options):
        Canvas.__init__(self, parent, **options)

        self.config(width=width, height=height, bd=3, relief=RIDGE, bg='#333333')
        self.pack(fill=BOTH, expand=True, side=LEFT)

        # Single Line Showing x-axis
        self.y_offset = 275
        self.create_line(0, self.y_offset, 1000, self.y_offset, fill='green', dash=[20, 10])

        self.images = {} # Collection of object to display as images on canvas
        self.legend = {} # Names of all objects to display

    def add_image(self, img_type, name, x, color):
        if(self.images.get(name) != None):
            return False

        # Add correct image type to canvas
        if(img_type == DrawSystemRequest.PENDULUM):
            rospy.loginfo("Adding new Pendulum image: " + name)

            # Add pendulum
            pendulum = PendulumImage(x)
            base_id = self.create_polygon(pendulum.get_base_points(), fill=color[0])
            pend_id = self.create_polygon(pendulum.get_pendulum_points(), fill=color[1])
            circle_id = self.create_oval(pendulum.get_rotation_circle_points(), fill='black')
            self.images[name] = {
                "base_id":   base_id,
                "pend_id":   pend_id,
                "circle_id": circle_id,
                "pendulum":  pendulum,
                "type":      img_type,
                "sub":       rospy.Subscriber(name, State, lambda s: pendulum.set_state(s.x))}
        elif(img_type == DrawSystemRequest.SINGLE_MASS):
            rospy.loginfo("Adding new Mass image: " + name)

            # Add mass object
            mass = RectangleImage(x, 50.0, 50.0)
            base_id = self.create_polygon(mass.get_corner_points(), fill=color[0])
            self.images[name] = {
                "base_id": base_id,
                "mass":    mass,
                "type":    img_type,
                "sub":     rospy.Subscriber(name, State, lambda s: mass.set_state(s.x))}
        elif(img_type == DrawSystemRequest.SPRING):
            rospy.loginfo("Adding new Spring image: " + name)

            # Add spring object
            spring = SpringImage(x, self.y_offset)
            spring_id = self.create_line(spring.get_points(), fill=color[0], width=2)
            self.images[name] = {
                "spring_id": spring_id,
                "spring":    spring,
                "type":      img_type,
                "sub":       rospy.Subscriber(name, State, lambda s: spring.set_state(s.x))}
        else:
            return False

        # Add to legend
        place = len(self.legend)+1
        y_offset = 25*place
        text_id = self.create_text(30, y_offset, anchor=W, fill="white", text=name)
        box_id = self.create_rectangle([15, y_offset-5, 25, y_offset+5], fill=color[0])
        self.legend[name] = {"text_id": text_id, "box_id": box_id, "place": place}

        return True

    def remove_image(self, name):
        # Delete items from canvas first, then remove from dict
        if(self.images.get(name) != None):
            rospy.loginfo("Deleting Image: " + name)
            if(self.images[name]["type"] == DrawSystemRequest.PENDULUM):
                self.delete(self.images[name]["base_id"])
                self.delete(self.images[name]["pend_id"])
                self.delete(self.images[name]["circle_id"])
            elif(self.images[name]["type"] == DrawSystemRequest.SINGLE_MASS):
                self.delete(self.images[name]["base_id"])
            elif(self.images[name]["type"] == DrawSystemRequest.SPRING):
                self.delete(self.images[name]["spring_id"])
            del self.images[name]
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
        for key in self.images:
            if(self.images[key]["type"] == DrawSystemRequest.PENDULUM):
                pendulum = self.images[key]["pendulum"]
                self.coords(self.images[key]["base_id"], pendulum.get_base_points())
                self.coords(self.images[key]["pend_id"], pendulum.get_pendulum_points())
                self.coords(self.images[key]["circle_id"], pendulum.get_rotation_circle_points())
            elif(self.images[key]["type"] == DrawSystemRequest.SINGLE_MASS):
                mass = self.images[key]["mass"]
                self.coords(self.images[key]["base_id"], mass.get_corner_points())
            elif(self.images[key]["type"] == DrawSystemRequest.SPRING):
                spring = self.images[key]["spring"]
                self.coords(self.images[key]["spring_id"], spring.get_points())
