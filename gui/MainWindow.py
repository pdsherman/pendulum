#!/user/bin/env python3

"""
File:   gui.py
Author: pdsherman
Date:   Jan. 2020

Description: Simple gui to display pendulums using basic shapes
"""
# 3rd party GUI library
from tkinter import *
from tkinter.messagebox import askokcancel

# ROS
import threading
import rospy
from pendulum.srv import AddPendulum, AddPendulumResponse

# Custom Frame objects
from ButtonBar import ButtonBar
from ImageCanvas import ImageCanvas

class MainWindow:
    def __init__(self):
        # GUI Root
        self.root = Tk()
        self.root.geometry("1000x600")
        self.root.title("Pendulum GUI")

        # Buttons for bottom of GUI
        btns = []
        btns.append(("Quit", self.quit))
        self.btnBar = ButtonBar(self.root, btns, width=500, height=500)

        self.img = ImageCanvas(self.root) # Display of pendulum
        self.quit = False # Flag for killing GUI

        # ROS service server: Because AddPendulum will be serviced in a
        # separate thread and Tkinter library functions can only be called
        # from main thread. Use a queue to hold info form service and then
        # call the actual add_pendulum method in update.
        s = rospy.Service('add_pendulum', AddPendulum, self.create_new_pendulum)
        self.lock = threading.Lock()
        self.service_queue = []

    def update(self):
        self.lock.acquire()
        try:
            r = self.service_queue.pop(0)
            self.img.add_pendulum(
                r.name, r.x, r.theta, r.base_fill_color, r.pendulum_fill_color)
        except IndexError:
            pass
        self.lock.release()

        # https://stackoverflow.com/questions/29158220/tkinter-understanding-mainloop
        self.img.update_drawing()
        self.root.update_idletasks()
        self.root.update()

    def create_new_pendulum(self, req):
        self.lock.acquire()
        self.service_queue.append(req)
        self.lock.release()
        return AddPendulumResponse(True)

    def quit(self):
        if askokcancel("Verify Exit", "Really Quit?"):
            self.quit = True

    def exit_requested(self):
        return self.quit
