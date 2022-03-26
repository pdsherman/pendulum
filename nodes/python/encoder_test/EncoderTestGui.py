#!/user/bin/env python3

"""
File:   EncoderTestGui.py
Author: pdsherman
Date:   March. 2022

Description: Simple GUI for interface to encoder test node
"""
import time
import rospy

import tkinter as tk # 3rd party GUI library
from tkinter.messagebox import askokcancel

from button_bar.ButtonBar import ButtonBar
from EncoderGuiHelpers import *

from pendulum.msg import State
from pendulum.srv import EncoderTest, EncoderTestRequest, EncoderTestResponse

"""
GUI to interact with encoder node and start/stop tests and logging
"""
class EncoderTestGui:

    """
    Initialization Method. Creates GUI with all the desired
    buttons and features.
    """
    def __init__(self):
        # GUI Root
        self.root = tk.Tk()
        self.root.geometry("650x420")
        self.root.title("Encoder Testing")

        # Flag for shutting down gui
        self.quit_flag = False;

        # Title Bar
        l = tk.Label(self.root)
        l.config(text="Encoder Testing", font=("Ubuntu", 14))
        l.config(bd=4, relief=tk.RIDGE)
        l.pack(side=tk.TOP, ipady=15, ipadx=50, pady=[0, 10], fill=tk.X)

        # Buttons for bottom of GUI
        btns = []
        btns.append(("Quit", self.quit))
        btns.append(("Subscribe", self.ros_subscribe))
        btns.append(("Zero", self.zero_encoder))
        btns.append(("Set Offset", self.set_encoder_offset))
        btns.append(("Start Test", self.start_test))
        btns.append(("Stop Test", self.stop_test))
        self.btnBar = ButtonBar(self.root, btns, width=500, height=500)

        # Boxes to show measurement
        self.init_main_window()
        self.measurement = 0.0

    """
    Create components to show current measurements and state of encoder
    """
    def init_main_window(self):
        f = tk.Frame(self.root)

        l = tk.Label(f)
        l.config(text="Angle (radians)", font=("Ubuntu", 10))
        l.config(relief=tk.RIDGE)
        l.grid(row=0, column=1, ipadx=9, ipady=2, pady=[0, 5])

        self.radian = tk.Label(f)
        self.radian.config(text="0.00", font=("Courier", 20))
        self.radian.config(bg='white', height=3, width=8, relief=tk.RIDGE)
        self.radian.grid(row=1, column=1, padx=30)

        l = tk.Label(f)
        l.config(text="Display", font=("Ubuntu", 10))
        l.config(relief=tk.RIDGE)
        l.grid(row=0, column=0, ipadx=9, ipady=2, pady=[0, 5])

        self.display = EncoderDisplay(f, side_length=150)
        self.display.grid(row=1, column=0, padx=10)

        f.config(bd=2)
        f.pack(side=tk.TOP, fill=tk.X, pady=[25, 5])

    """
    Update the encoder angle (display and text)
    @note: Intended for use as ROS subscriber callback
    """
    def update_angle(self, state):
        self.measurement = state.theta

    """
    Run update cycles. Should be called in main application loop
    """
    def cycle(self):
        self.display.update_mark(self.measurement)
        self.radian.config(text="{:.2f}".format(self.measurement))

        self.root.update_idletasks()
        self.root.update()

    """
    Check if user has requested to quit application
    """
    def quit_requested(self):
        return self.quit_flag


    """
    Helper Callback for ros suscribe button
    Will begin GUI subsricing to encoder state topic
    """
    def subscribe_callback(self, topic, type):
        # Create a subsrciber, connect it to update angle method
        if(type == "pendulum/State"):
            self.sub = rospy.Subscriber(topic, State, self.update_angle)

    # ------------------------------------------------- #
    # ------------ BUTTON CALLBACK METHODS ------------ #
    # ------------------------------------------------- #

    """
    """
    def ros_subscribe(self):
        rospy.loginfo("Subscribe to new ROS topic")
        EncoderPublishList(self.root, self.subscribe_callback)

    """
    """
    def start_test(self):
        rospy.loginfo("Start an encoder test")
        try:
            req = EncoderTestRequest(encoder=1, command=EncoderTestRequest.START_LOGGING)
            rospy.wait_for_service("/encoder_test", timeout=5)
            rospy.ServiceProxy("/encoder_test", EncoderTest)(req)
        except rospy.ROSException:
            rospy.logwarn("Unable to call service.")

    """
    """
    def stop_test(self):
        rospy.loginfo("Stop an encoder Test")
        try:
            req = EncoderTestRequest(encoder=1, command=EncoderTestRequest.STOP_LOGGING)
            rospy.wait_for_service("/encoder_test", timeout=5)
            rospy.ServiceProxy("/encoder_test", EncoderTest)(req)
        except rospy.ROSException:
            rospy.logwarn("Unable to call service.")

    """
    """
    def zero_encoder(self):
        rospy.loginfo("Zero the encoder")
        try:
            req = EncoderTestRequest(encoder=1, command=EncoderTestRequest.ZERO)
            rospy.wait_for_service("/encoder_test", timeout=5)
            rospy.ServiceProxy("/encoder_test", EncoderTest)(req)
        except rospy.ROSException:
            rospy.logwarn("Unable to call service.")

    """
    """
    def set_encoder_offset(self):
        rospy.loginfo("Set encoder offset")

    """
    Opens pop-window to ask user if they are ready to quit GUI.
    If yes, set flag to be checked by main application
    """
    def quit(self):
        if askokcancel("Verify Exit", "Really Quit?"):
            self.quit_flag = True
