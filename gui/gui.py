#!/usr/bin/env python3

import rospy
import math
import time

from MainWindow import MainWindow

try:
    rospy.init_node("gui", anonymous=True)
    gui = MainWindow()
    gui.create_new_pendulum("TestPendulum", 0.0, 0.0)

    while not rospy.is_shutdown() and not gui.exit_requested():
        gui.update()

except rospy.ROSInterruptException:
    pass
