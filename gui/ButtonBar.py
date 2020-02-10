"""
File:   ButtonBar.py
Author: pdsherman
Date:   Jan 2020

Description: Frame containing a row of buttons.
"""

from tkinter import *

class ButtonBar(Frame):
    def __init__(self, parent=None, buttons=None, btn_height = 2, btn_width=8, **options):
        Frame.__init__(self, parent, **options)

        # Add all buttons to the bar
        for (name, func) in buttons:
            b = Button(self, text=name, command=func, height=btn_height, width=btn_width)
            b.pack(side=RIGHT)

        self.config(bd=3, relief=RIDGE)
        self.pack(side=BOTTOM, fill=X)
