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
from pendulum.srv import DrawSystem, DrawSystemResponse, DrawSystemRequest
from pendulum.srv import DeleteSystem, DeleteSystemResponse

# Custom Frame objects
from button_bar.ButtonBar import ButtonBar
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
        btns.append(("Clear", self.reset_window))
        self.btnBar = ButtonBar(self.root, btns, width=500, height=500)

        self.img = ImageCanvas(self.root) # Display window
        self.quit_flag = False # Flag for killing GUI

        # ROS service server: Because ROS services are handled in a
        # separate thread and Tkinter library functions can only be called
        # from main thread, use a queue to hold info from service and then
        # call the actual gui methods in update.
        self.lock = threading.Lock()
        self.service_queue = []

        rospy.Service('/gui/draw_system', DrawSystem, self.create_new_pendulum)
        rospy.Service('/gui/delete_system', DeleteSystem, self.remove_pendulum)

    def update(self):
        self.check_service_calls()

        # Update Image
        self.img.update_drawing()
        # https://stackoverflow.com/questions/29158220/tkinter-understanding-mainloop
        self.root.update_idletasks()
        self.root.update()

    def check_service_calls(self):
        self.lock.acquire()
        if len(self.service_queue) > 0:
            r = self.service_queue.pop(0)
            if(r[0] == "add"):
                if r[1].img_type == DrawSystemRequest.PENDULUM:
                    self.img.add_image("pendulum",
                        r[1].name, r[1].x, r[1].theta, r[1].base_color, r[1].pendulum_color)
                elif r[1].img_type == DrawSystemRequest.MASS_ONLY:
                    self.img.add_image("mass_only",
                        r[1].name, r[1].x, None, r[1].base_color, None)
            elif r[0] == "remove":
                self.img.remove_image(r[1].name)
        self.lock.release()

    def create_new_pendulum(self, req):
        self.lock.acquire()
        self.service_queue.append(("add", req))
        self.lock.release()
        return DrawSystemResponse(True)

    def remove_pendulum(self, req):
        self.lock.acquire()
        self.service_queue.append(("remove", req))
        self.lock.release()
        return DeleteSystemResponse(True)

    def quit(self):
        if askokcancel("Verify Exit", "Really Quit?"):
            self.quit_flag = True

    def reset_window(self):
        print("Reset Place Holder...")

    def exit_requested(self):
        return self.quit_flag
