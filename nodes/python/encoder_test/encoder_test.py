#!/user/bin/env python3

"""
File:  encoder_test.py
Author: pdsherman
Date:   March 2022

Description: Run simple ROS node with encoder test GUI
"""

import sys
sys.path.insert(1, "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/py_libs")

import rospy
import time

import tkinter as tk # 3rd party GUI library

from EncoderTestGui import *


# ----------------------------- #
# --      START OF SCRIPT    -- #
# ----------------------------- #
if __name__ == "__main__":
    try:
        rospy.init_node("encoder_test_node", anonymous=False)
        gui = EncoderTestGui()

        while not gui.quit_requested() and not rospy.is_shutdown():
            gui.cycle()
            time.sleep(0.01)

    except tk._tkinter.TclError, e:
        print("Tk error: " + str(e))
    except rospy.ROSInterruptException:
        pass
