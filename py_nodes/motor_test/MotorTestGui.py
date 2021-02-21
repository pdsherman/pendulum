#!/user/bin/env python3

"""
File:   RosMotor.py
Author: pdsherman
Date:   Feb. 2020

Description: Simple object for ROS interface to motor object
"""

from RosMotor import *

# 3rd party GUI library
import tkinter as tk

class MotorTestGui:
    def __init__(self):

        # Motor Object
        self.mtr = RosMotor()

        # GUI Root
        self.root = tk.Tk()
        self.root.geometry("400x450")
        self.root.title("Motor Test")

        # Flag for shutting down gui
        self.quit = False;

        # Title Bar
        l = tk.Label(self.root)
        l.config(text="Motor Control GUI", font=("Ubuntu", 16))
        l.config(bd=4, relief=tk.RIDGE)
        l.pack(side=tk.TOP, ipady=20, ipadx=50, pady=[10, 10])

        # Frame at bottom w/ buttons
        self.init_buttons()

        # Text Boxes to show target and measured current
        self.init_text_boxes()

        # Slider bar to adjust current
        self.sldr = tk.Scale(self.root, from_=-5.0, to=5.0, resolution=0.05,
            tickinterval=2.5, showvalue=0, length=300, orient=tk.HORIZONTAL,
            command=self.slider_change)
        self.sldr.pack(side=tk.BOTTOM, pady=[5, 30])

    def init_buttons(self):

        self.button_frame = tk.Frame(self.root)

        # Create Motor Enable Button
        enable_btn = tk.Button(self.button_frame, text="Enable", command=self.mtr_enable, height=3, width=9)
        enable_btn.pack(side=tk.LEFT)

        # Create Motor Disable Button
        disable_btn = tk.Button(self.button_frame, text="Disable", command=self.mtr_disable, height=3, width=9)
        disable_btn.pack(side=tk.LEFT)

        # Quit Button
        qbtn = tk.Button(self.button_frame, text="Quit", command=self.quit_set, height=3, width=9)
        qbtn.pack(side=tk.RIGHT)

        self.button_frame.config(bd=3, relief=tk.RIDGE)
        self.button_frame.pack(side=tk.BOTTOM, fill=tk.X)

    def init_text_boxes(self):
        f = tk.Frame(self.root)

        l = tk.Label(f)
        l.config(text="Target\nCurrent (Amps)", font=("Ubuntu", 12))
        l.config(relief=tk.RIDGE)
        l.grid(column=0, row=0, ipadx=9, ipady=2, pady=[0, 5])

        self.target_amp = tk.Label(f)
        self.target_amp.config(text="0.00", font=("Courier", 20))
        self.target_amp.config(bg='white', height=3, width=8, relief=tk.RIDGE)
        self.target_amp.grid(column=0, row=1, padx=30)

        l = tk.Label(f)
        l.config(text="Measured\nCurrent (Amps)", font=("Ubuntu", 12))
        l.config(relief=tk.RIDGE)
        l.grid(column=1, row=0, ipadx=9, ipady=2, pady=[0, 5])

        self.measured_amp = tk.Label(f)
        self.measured_amp.config(text="0.00", font=("Courier", 20))
        self.measured_amp.config(bg='white', height=3, width=8, relief=tk.RIDGE)
        self.measured_amp.grid(column=1, row=1, padx=30)

        f.config(bd=2)
        f.pack(side=tk.TOP, fill=tk.X, pady=[25, 5])

    def update(self):
        self.root.update_idletasks()
        self.root.update()

        self.measured_amp.config(text="{:.2f}".format(self.mtr.read_current()))

    def quit_set(self):
        self.quit = True

    def mtr_enable(self):
        self.mtr.enable_motor()

    def mtr_disable(self):
        self.mtr.disable_motor();

    def quit_requested(self):
        return self.quit

    def slider_change(self, value):
        value_f = 0.0
        try:
            value_f = float(value)
        except:
            value_f = 0.0
        self.mtr.drive_current(value_f)
        self.target_amp.config(text="{:.2f}".format(value_f))
