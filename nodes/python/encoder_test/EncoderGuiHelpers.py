#!/user/bin/env python3

"""
File:   EncoderDisplay.py
Author: pdsherman
Date:   March. 2022

Description: Display an image of an angle
"""

import tkinter as tk
import math
import rospy

deg2rad = math.pi/180.0

class EncoderDisplay(tk.Canvas):

    def __init__(self, parent, side_length=10):
        tk.Canvas.__init__(self, parent)
        self.config(width=side_length, height=side_length, bd=3, relief=tk.RIDGE, bg='#c1d1d1')

        self.center = side_length/2 # Center coordinate (x AND y)
        self.create_oval(
            self.center-2, self.center-2,
            self.center+2, self.center+2,
            fill="black")

        # Create angle marks
        self.side_length = side_length
        l1 = side_length/2 * 0.8
        l2 = side_length/2 * 0.9
        for deg in range(0, 360, 15):
            rad = deg * deg2rad
            x1 = self.center + l1 * math.cos(rad)
            y1 = self.center + l1 * math.sin(rad)
            x2 = self.center + l2 * math.cos(rad)
            y2 = self.center + l2 * math.sin(rad)
            self.create_line(x1, y1, x2, y2)

        # Create mark
        (x1, x2, x3) = self.get_mark_coords(0.00)
        self.mark = self.create_polygon(x1[0], x1[1], x2[0], x2[1], x3[0], x3[1], fill="#e65705" )

    def get_mark_coords(self, angle):
        b = 6.0
        h = self.side_length/2 * 0.85
        x1 = [self.center + h*math.cos(angle), self.center - h*math.sin(angle)]
        x2 = [self.center + b*math.sin(angle), self.center + b*math.cos(angle)]
        x3 = [self.center - b*math.sin(angle), self.center - b*math.cos(angle)]
        return (x1, x2, x3)

    def update_mark(self, angle):
        (x1, x2, x3) = self.get_mark_coords(angle)
        self.coords(self.mark, x1[0], x1[1], x2[0], x2[1], x3[0], x3[1])

class EncoderPublishList(tk.Toplevel):

    def __init__(self, parent, select_callback):
        tk.Toplevel.__init__(self, parent)
        self.title("Encoder Publisher Selection")

        self.callback = select_callback

        ###
        # Top Label
        ###
        f = tk.Frame(self)
        lb = tk.Label(f, text="Select Publisher", font=("Courier", 12))
        lb.grid(padx=[100,100], pady=[5,5])
        f.config(bd=2, relief=tk.RIDGE)
        f.pack(expand=tk.YES, fill=tk.X)


        ###
        # Create Scroll bar and textbox with available motors
        ###
        f = tk.Frame(self)

        self.sbar = tk.Scrollbar(f)
        self.list = tk.Listbox(f, relief=tk.SUNKEN, font=("Courier", 11))

        # One moves the other
        self.sbar.config(command=self.list.yview)
        self.list.config(yscrollcommand=self.sbar.set)

        self.sbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.list.pack(side=tk.LEFT, expand=tk.YES, fill=tk.BOTH)

        for [topic, type] in rospy.get_published_topics():
            self.list.insert(tk.END, "{}:{}".format(topic, type))
        f.pack(expand=tk.YES, fill=tk.BOTH)

        ###
        # Selection Button
        ###
        b = tk.Button(self, text="Select", command=self.selection,
            font="Courier", height=2, width=10)
        b.pack(side=tk.BOTTOM)

    def selection(self):
        """
        Select button callback method. Gets current selection
        from list and call the parent callback with motor if valid.
        """
        index = self.list.curselection()

        if len(index) > 0:
            line  = self.list.get(index).split(":")
            topic = line[0]
            type  = line[1]
            self.callback(topic, type)
            self.destroy()
        else:
            rospy.logwarn("No selection made")

class EncoderLoggingStart(tk.Toplevel):
    def __init__(self, parent, select_callback):
        tk.Toplevel.__init__(self, parent)
        self.title("Logging Start")

        # Called when input is made and selected
        self.callback = select_callback

        # Create Frame
        frm1 = tk.Frame(self)
        lb = tk.Label(frm1, text="Input Logging Info", font=("Courier", 16))
        lb.grid(padx=[100,100], pady=[5,5])
        frm1.config(bd=2, relief=tk.RIDGE)
        frm1.pack(expand=tk.YES, fill=tk.X)

        # Input Boxes
        frm2 = tk.Frame(self)

        l = tk.Label(frm2, text="Table Name:", font=("Courier", 12))
        l.grid(row=0, column=0,  padx=[5, 15], pady=5)

        self.tbl = tk.StringVar()
        e = tk.Entry(frm2, width=25, font=("Courier", 12), textvariable=self.tbl)
        e.grid(row=0, column=1)
        self.tbl.set("table_name")

        l = tk.Label(frm2, text=" Test Name:", font=("Courier", 12))
        l.grid(row=1, column=0,  padx=[5, 15], pady=5)

        self.test = tk.StringVar()
        e = tk.Entry(frm2, width=25, font=("Courier", 12), textvariable=self.test)
        e.grid(row=1, column=1)
        self.test.set("test_name")

        frm2.config(bd=2, relief=tk.RIDGE)
        frm2.pack(expand=tk.YES, fill=tk.X)

        # Selection Button
        b = tk.Button(self, text="Select", command=self.selection, font="Courier", height=2, width=10)
        b.pack(side=tk.BOTTOM)

    def selection(self):
        table_name = self.tbl.get()
        test_name  = self.test.get()
        if self.valid_sqlite_name(table_name) and self.valid_sqlite_name(test_name):
            self.callback(table_name, test_name)
        else:
            rospy.logwarn("Invalid names")
        self.destroy()

    def valid_sqlite_name(self, s):
        return len(s) != 0 \
                and (not " " in s) \
                and (not "-" in s) \
                and s[0].isalpha()

class EncoderOffset(tk.Toplevel):
    def __init__(self, parent, select_callback):
        tk.Toplevel.__init__(self, parent)
        self.title("Offset")

        # Called when input is made and selected
        self.callback = select_callback

        # Create Frame
        frm1 = tk.Frame(self)
        lb = tk.Label(frm1, text="Input Target Offset", font=("Courier", 16))
        lb.grid(padx=[100,100], pady=[5,5])
        frm1.config(bd=2, relief=tk.RIDGE)
        frm1.pack(expand=tk.YES, fill=tk.X)

        # Input Boxes
        frm2 = tk.Frame(self)

        l = tk.Label(frm2, text="Target Offset (deg):", font=("Courier", 12))
        l.grid(row=0, column=0,  padx=[5, 15], pady=15)

        self.offset = tk.StringVar()
        validate_cmd = self.register(self.float_entry_ok)
        e = tk.Entry(frm2, width=15, font=("Courier", 12), textvariable=self.offset,
                            validate='key', validatecommand=(validate_cmd, '%P'))
        e.grid(row=0, column=1)
        self.offset.set("0.0")

        frm2.config(bd=2, relief=tk.RIDGE)
        frm2.pack(expand=tk.YES, fill=tk.X)

        # Selection Button
        b = tk.Button(self, text="Select", command=self.selection, font="Courier", height=2, width=10)
        b.pack(side=tk.BOTTOM)

    def selection(self):
        offset = float(self.offset.get())
        self.callback(offset)
        self.destroy()

    def float_entry_ok(self, post_change):
        if post_change:
            try:
                float(post_change)
            except ValueError:
                return False
        return True
