#!/user/bin/env python3

"""
File:  motor_test.py
Author: pdsherman
Date:   Feb. 2020

Description: Run simple ROS node with motor test GUI
"""

import rospy
import time

from MotorTestGui import *

# ----------------------------- #
# --      START OF SCRIPT    -- #
# ----------------------------- #

try:
    rospy.init_node("mtr_test", anonymous=False)
    gui = MotorTestGui()

    while not gui.quit_requested() and not rospy.is_shutdown():
        gui.update()
        time.sleep(0.01)

except rospy.ROSInterruptException:
    pass
