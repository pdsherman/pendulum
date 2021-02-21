#!/user/bin/env python

"""
File:   RosMotor.py
Author: pdsherman
Date:   Feb. 2020

Description: Simple object for ROS interface to motor object
"""

from pendulum.srv import MotorControl, MotorControlRequest, MotorControlResponse
from pendulum.msg import Current

import rospy

class RosMotor:
    def __init__(self):
        self.target_current   = 0.0;
        self.measured_current = 0.0;

        rospy.Subscriber("current_measured", Current, self.callback)
        self.pub = rospy.Publisher("current_cmd", Current, queue_size=10)

    def enable_motor(self):
        rospy.wait_for_service('control')
        rospy.ServiceProxy('control', MotorControl)(True)

    def disable_motor(self):
        rospy.wait_for_service('control')
        rospy.ServiceProxy('control', MotorControl)(False)

    def read_current(self):
        return self.measured_current

    def drive_current(self, current):
        if(self.target_current != current):
            msg = Current()
            msg.header.stamp = rospy.Time.now()
            msg.header.seq += 1
            msg.current_A = current
            self.pub.publish(msg)

            self.target_current = current

    def callback(self, msg):
        self.measured_current = msg.current_A
