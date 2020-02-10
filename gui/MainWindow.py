#!/user/bin/env python3

"""
File:   gui.py
Author: pdsherman
Date:   Jan. 2020

Description: Simple gui to show pendulum with basic shapes
"""
# 3rd party GUI library
from tkinter import *
from tkinter.messagebox import askokcancel

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

    def update(self):
        # https://stackoverflow.com/questions/29158220/tkinter-understanding-mainloop
        self.img.update_drawing()
        self.root.update_idletasks()
        self.root.update()

    def create_new_pendulum(self, name, x0, theta0):
        self.img.add_pendulum(name, x0, theta0)

    def quit(self):
        if askokcancel("Verify Exit", "Really Quit?"):
            self.quit = True

    def exit_requested(self):
        return self.quit
