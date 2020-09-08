#!/usr/bin/env python3

import rospy
import math
import time

from MainWindow import MainWindow

try:
    rospy.init_node("gui", anonymous=False)
    gui = MainWindow()

    while not rospy.is_shutdown() and not gui.exit_requested():
        gui.update()
        time.sleep(0.01)

except rospy.ROSInterruptException:
    pass
