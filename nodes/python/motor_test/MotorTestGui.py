#!/user/bin/env python3

"""
File:   RosMotor.py
Author: pdsherman
Date:   Feb. 2020

Description: Simple object for ROS interface to motor object
"""

from RosMotor import *
import tkinter as tk # 3rd party GUI library
import time

class ControlTuningGui(tk.Toplevel):
    """
    Window to adjust control gains and request step inputs
    """
    def __init__(self, parent, mtr):
        """
        Initialization method. Will create pop-up window.
        -------------
        parent: Window creating this object
        mtr:    ROS motor object
        """
        tk.Toplevel.__init__(self, parent)
        self.title("Current Control Tuning")
        self.geometry("350x340")

        self.mtr = mtr

        ##
        # Top Label
        ##
        f = tk.Frame(self)
        lb = tk.Label(f, text="Current Control Tuning", font=("Courier", 16))
        lb.grid(padx=[25, 25], pady=[5, 5])
        f.config(bd=2, relief=tk.RIDGE)
        f.pack(side=tk.TOP, fill=tk.X)

        # Frame to set Control Gains
        self.build_control_gains_entry()

        # Frame to set pulse height and width
        self.build_pulse_entry()

    def build_control_gains_entry(self):
        f = tk.Frame(self)

        l = tk.Label(f, text="Cp:", font=("Courier, 16"))
        l.grid(row=0, column=0)

        # Use validation function to make sure gains entered are
        # valid integers (empty string is okay to)
        validate_cmd = self.register(self.int_entry_ok)

        self.Cp = tk.StringVar()
        e = tk.Entry(f, width=6, font=("Courier, 16"), textvariable=self.Cp,
            validate='key', validatecommand=(validate_cmd, '%P'))
        e.grid(row=1, column=0, ipady=5, padx=[25, 10], pady=5)
        self.Cp.set("0")

        l = tk.Label(f, text="Ci:", font=("Courier, 16"))
        l.grid(row=0, column=1)

        self.Ci = tk.StringVar()
        e = tk.Entry(f, width=6, font=("Courier, 16"), textvariable=self.Ci,
            validate='key', validatecommand=(validate_cmd, '%P'))
        e.grid(row=1, column=1, ipady=5, padx=10, pady=5)
        self.Ci.set("0")

        b = tk.Button(f, text="Set Gains", command=self.set_gains)
        b.grid(row=2, column=0, padx=25, pady=10)

        f.config(bd=2, relief=tk.RIDGE)
        f.pack(fill=tk.X)

    def build_pulse_entry(self):
        f = tk.Frame(self)

        # Use validation function to make sure gains entered are
        # valid integers (empty string is okay to)
        validate_cmd = self.register(self.float_entry_ok)

        l = tk.Label(f, text="Height (Amps): ", font=("Courier, 12"))
        l.grid(row=0, column=0, padx=[20,10], pady=5)

        self.pulse_height = tk.StringVar()
        e = tk.Entry(f, width=10, font=("Courier, 12"), textvariable=self.pulse_height,
            validate='key', validatecommand=(validate_cmd, '%P'))
        e.grid(row=0, column=1)
        self.pulse_height.set(0.0)

        l = tk.Label(f, text="Width (s): ", font=("Courier, 12"))
        l.grid(row=1, column=0, padx=[20,10], pady=5)

        self.pulse_width = tk.StringVar()
        e = tk.Entry(f, width=10, font=("Courier, 12"), textvariable=self.pulse_width,
            validate='key', validatecommand=(validate_cmd, '%P'))
        e.grid(row=1, column=1)
        self.pulse_width.set(0.0)


        l = tk.Label(f, text="Num Pulses: ", font=("Courier, 12"))
        l.grid(row=2, column=0, padx=[20,10], pady=5)

        validate_cmd = self.register(self.int_entry_ok)
        self.num_pulses = tk.StringVar()
        e = tk.Entry(f, width=10, font=("Courier, 12"), textvariable=self.num_pulses,
            validate='key', validatecommand=(validate_cmd, '%P'))
        e.grid(row=2, column=1)
        self.num_pulses.set(0)

        b = tk.Button(f, text="Send Pulse", command=self.send_pulse)
        b.grid(row=3, column=0, padx=25, pady=10)

        f.config(bd=2, relief=tk.RIDGE)
        f.pack(fill=tk.X)

    def set_gains(self):
        if self.Cp.get():
            cp = int(self.Cp.get())
        else:
            cp = 0

        if self.Ci.get():
            ci = int(self.Ci.get())
        else:
            ci = 0
        self.mtr.set_current_gains(cp, ci)

    def send_pulse(self):
        if (not self.pulse_width.get()) or (not self.pulse_height.get()) or (not self.num_pulses.get()):
            return

        on_time = float(self.pulse_width.get())
        if on_time < 0.0 or on_time > 10.0:
            return

        amp = float(self.pulse_height.get())

        # Enable Motor
        self.mtr.enable_motor()

        # Wait for 0.5 seconds
        self.mtr.drive_current(0.0)
        time.sleep(0.5)
        self.mtr.drive_current(0.0)


        for i in range(int(self.num_pulses.get())):
            # Drive at desired amp for some length of time
            start_time = time.time()
            while(time.time() < start_time + (on_time/2.0)):
                self.mtr.drive_current(amp)
                time.sleep(0.05)

            start_time = time.time()
            while(time.time() < start_time + (on_time/2.0)):
                self.mtr.drive_current(-1.0*amp)
                time.sleep(0.05)

        self.mtr.drive_current(0.0)
        time.sleep(0.5)

        # Disable
        self.mtr.disable_motor()

    def int_entry_ok(self, post_change):
        if post_change:
            return post_change.isdigit()
        return True

    def float_entry_ok(self, post_change):
        if post_change:
            try:
                float(post_change)
            except ValueError:
                return False
        return True

class MotorTestGui:
    """
    GUI to test motor on the benchtop. Allows user to enable/disable
    motor and adjust current. Includes functionality to adjust current
    control gains.
    """
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
        l.pack(side=tk.TOP, ipady=20, ipadx=50, pady=[10, 10], fill=tk.X)

        # Frame at bottom w/ buttons
        self.init_buttons()

        # Text Boxes to show target and measured current
        self.init_text_boxes()

        # Slider bar to adjust current
        self.sldr = tk.Scale(self.root, from_=-1.0, to=1.0, resolution=0.001,
            tickinterval=0.5, showvalue=0, length=300, orient=tk.HORIZONTAL,
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

        # Launch Control Gains Window
        disable_btn = tk.Button(self.button_frame, text="Control", command=self.launch_tuning, height=3, width=9)
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

    def launch_tuning(self):
        ControlTuningGui(self.root, self.mtr)

    def quit_requested(self):
        return self.quit

    def slider_change(self, value):
        value_f = 0.0
        try:
            value_f = float(value)
        except ValueError:
            value_f = 0.0
        self.mtr.drive_current(value_f)
        self.target_amp.config(text="{:.2f}".format(value_f))
