#!/usr/bin/env python3

import rospy
import time
import math

from pendulum.msg import State

try:
    rospy.init_node('pendulum_state_pub')

    pub = rospy.Publisher('TestPendulum', State, queue_size=10)
    rate = rospy.Rate(200)

    x = 50.0
    theta = 90.0
    dt = 0.02
    x_dot = 40.0
    w = 0.2
    t = 0.0

    state = State()
    while not rospy.is_shutdown():
        x += dt*x_dot
        theta = (math.sin(w*t) + 1.0)*-90.0
        t += dt

        if x >= 950.0:
            x = 950.0
            x_dot = -1.0*abs(x_dot)
        elif x <= 50.0:
            x = 50.0
            x_dot = abs(x_dot)

        state.x = x
        state.theta = theta

        pub.publish(state)
        rate.sleep()


except rospy.ROSInterruptException:
    pass
