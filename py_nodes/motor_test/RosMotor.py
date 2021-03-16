#!/user/bin/env python

"""
File:   RosMotor.py
Author: pdsherman
Date:   Feb. 2020

Description: Simple object for ROS interface to motor object
"""

from pendulum.srv import MotorDriveMode, MotorDriveModeRequest, MotorDriveModeResponse
from pendulum.srv import MotorControlGains, MotorControlGainsRequest, MotorControlGainsResponse
from pendulum.msg import Current

import rospy

class RosMotor:
    def __init__(self, motor_node='benchtop'):
        self.target_current   = 0.0;
        self.measured_current = 0.0;

        rospy.Subscriber(motor_node+"/current_measured", Current, self.callback)
        self.pub = rospy.Publisher(motor_node+"/current_cmd", Current, queue_size=10)

        self.drive_srv_name = motor_node + '/drive_mode'
        self.current_srv_name = motor_node + '/control_gains'

    def enable_motor(self):
        try:
            rospy.wait_for_service(self.drive_srv_name, timeout=5)
            rospy.ServiceProxy(self.drive_srv_name, MotorDriveMode)(True)
        except rospy.ROSException:
            rospy.logwarn("Unable to call service.")

    def disable_motor(self):
        try:
            rospy.wait_for_service(self.drive_srv_name, timeout=5)
            rospy.ServiceProxy(self.drive_srv_name, MotorDriveMode)(False)
        except rospy.ROSException:
            rospy.logwarn("Unable to call service.")

    def read_current(self):
        return self.measured_current

    def drive_current(self, current):
        # if(self.target_current != current):
        msg = Current()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        msg.current_A = current
        self.pub.publish(msg)

        self.target_current = current

    def set_current_gains(self, Cp, Ci):
        try:
            req = MotorControlGainsRequest(Cp=Cp, Ci=Ci);
            rospy.wait_for_service(self.current_srv_name)
            resp = rospy.ServiceProxy(self.current_srv_name, MotorControlGains)(req)
            if resp.success:
                rospy.loginfo("Current Gains Set: Cp={}, Ci={}".format(Cp, Ci))
                return True
            else:
                rospy.logwanr("Failed to set current gains.")
                return False
        except rospy.ROSException:
            rospy.logwarn("Unable to call service.")


    def callback(self, msg):
        self.measured_current = msg.current_A
