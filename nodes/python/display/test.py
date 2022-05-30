#!/usr/bin/env python3

import sys
sys.path.insert(1, "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/py_libs")

import rospy
from pendulum.srv import DrawSystem, DrawSystemResponse, DrawSystemRequest
from pendulum.srv import DeleteSystem, DeleteSystemResponse, DeleteSystemRequest
from pendulum.msg import State


# ----------------------------- #
# --      START OF SCRIPT    -- #
# ----------------------------- #
X1_STEP     = 0.05
THETA1_STEP = 0.0872665
X2_STEP     = 0.02
THETA2_STEP = 1.5*0.0872665
X3_STEP     = 0.033
PI         = 3.14159265359

if __name__ == "__main__":
    rospy.init_node('system', anonymous=True)
    rospy.wait_for_service('/gui/draw_system')

    count = 0
    try:
        pub_name1 = 'plant1'
        pub_name2 = 'plant2'
        pub_name3 = 'line2'

        draw = rospy.ServiceProxy('/gui/draw_system', DrawSystem)
        resp = draw(pub_name1, 1, [0.1, 0.785398], ['yellow', 'blue'])
        resp = draw(pub_name2, 2, [0.5], ['red'])
        resp = draw(pub_name3, 3, [0.4, 0.6], ['white'])

        rate = rospy.Rate(10)
        pub1 = rospy.Publisher(pub_name1, State, queue_size = 2)
        pub2 = rospy.Publisher(pub_name2, State, queue_size = 2)
        pub3 = rospy.Publisher(pub_name3, State, queue_size = 2)

        s1 = State()
        s2 = State()
        s3 = State()
        x1 = 0.1
        x2 = 0.5
        x3 = [0.4, 0.6]
        theta1 = 0.785398
        theta2 = 3*0.785398
        while not rospy.is_shutdown() and count < 100:
            count += 1
            if x1 > 0.7 or x1 < 0.1:
                X1_STEP = -X1_STEP
            if theta1 > 2*PI or theta1 < 0.0:
                THETA1_STEP = -THETA1_STEP
            if x2 > 0.7 or x2 < 0.1:
                X2_STEP = -X2_STEP
            if theta2 > 2*PI or theta2 < 0.0:
                THETA2_STEP = -THETA2_STEP
            if x3[0] > 0.7 or x3[0] < 0.1:
                X3_STEP = -X3_STEP

            x1 += X1_STEP
            theta1 += THETA1_STEP
            x2 += X2_STEP
            theta2 += THETA2_STEP
            x3[0] = x3[0] + X3_STEP
            x3[1] = 0.6 # x3[1] + X3_STEP

            s1.x = [x1, theta1]
            s2.x = [x2] #, theta2]
            s3.x = x3

            pub1.publish(s1)
            pub2.publish(s2)
            pub3.publish(s3)
            rate.sleep()

        delete = rospy.ServiceProxy('/gui/delete_system', DeleteSystem)
        resp = delete(pub_name1)
        resp = delete(pub_name2)
        resp = delete(pub_name3)

    except rospy.ServiceException as e:
        print("Service call failed")
