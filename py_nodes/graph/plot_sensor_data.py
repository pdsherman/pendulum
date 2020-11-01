#!/usr/bin/env python3

"""
File:   plot_sensor_data.py
Author: pdsherman
Date:   Sep. 2020

Description: ROS node to read data from table and plot
"""

from pendulum.srv import GraphData, GraphDataResponse

import math
import time
import rospy
import threading
import sqlite3

import matplotlib.pyplot as plt

# Queue and lock used to avoid the service
# call needing to actually do the plotting which
# will block until the plot is closed.
req_lock = threading.Lock()
req_queue = []

def plot_data(req):
    rospy.loginfo("Queueing plot request")

    req_lock.acquire()
    req_queue.append(req)
    req_lock.release()

    return GraphDataResponse(True)

def get_sensor_data():
    data = {"t": [], "x": [], "theta": []}

    database = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/EncoderData.db"
    conn = sqlite3.connect(database)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()

    command = "SELECT * FROM {}".format("TestTable")
    command += " where test_time_s > {} and test_time_s < {} and theta < 10.0".format(
            5.26, 65.5)
    command += " ORDER BY timestamp"

    c.execute(command)
    for row in c:
        data["t"].append(row['test_time_s'])
        data["x"].append(row['x'])
        data["theta"].append(row['theta'])
    data["t"] = [x - data["t"][0] for x in data["t"]]

    conn.close()
    return data

def get_data(req):
    data = {"t": [], "x": [], "theta": []}

    database = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/EncoderData.db"
    conn = sqlite3.connect(database)
    conn.row_factory = sqlite3.Row

    c = conn.cursor()


    command = "SELECT * FROM {}".format(req.table_name)
    if(req.use_time_bound):
        command += " where test_time_s >= {} and test_time_s <= {}".format(
                req.lower_time_bound, req.upper_time_bound)
    command += " ORDER BY test_time_s"
    c.execute(command)

    for row in c:
        data["t"].append(row['test_time_s'])
        data["x"].append(row['x'])
        data["theta"].append(row['theta'])

    conn.close()

    return data

def graph_request(req):
    plt.figure()

    # Get Data
    rospy.loginfo("Request to Graph {} vs {}".format(req.y_axis_data, req.x_axis_data))
    data = get_data(req)
    if(req.overlay):
        sensor_data = get_sensor_data()
        plt.plot(sensor_data["t"], sensor_data["theta"],'r')

    plt.subplots_adjust(left=0.04, right=0.97)
    plt.plot(data["t"], data["theta"], 'b')
    plt.show()

# ********************** #
# **  Start of Script ** #
# ********************** #

if __name__ == "__main__":

    try:
        rospy.init_node("grapher", anonymous=False, log_level=rospy.INFO)
        rospy.Service('/grapher/plot_data', GraphData, plot_data)

        while not rospy.is_shutdown():

            req_lock.acquire()
            try:
                req = req_queue.pop(0)
            except IndexError:
                req = None
            req_lock.release()

            if req:
                graph_request(req)
            time.sleep(0.1)

    except rospy.ROSInterruptException:
        ROS_LOG("Exiting graphing node")
