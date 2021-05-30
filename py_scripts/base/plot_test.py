#!/usr/bin/env python3

"""
File:   plot_test.py
Author: pdsherman
Date:   April 2021
"""

import math
import numpy as np
import sqlite3
import sys

import matplotlib
import matplotlib.pyplot as plt

def get_data(table_name, test_name, filter=None):
    data = {"time": [], "input": [], "position": [], "target": []}
    database = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/PendulumDatabase.db"
    command  = "SELECT * FROM {} where test = \"{}\" ORDER BY test_time".format(table_name, test_name)
    if filter:
        command += filter

    conn = sqlite3.connect(database)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    c.execute(command)

    for row in c:
        data["time"].append(row["test_time"])
        data["input"].append(row["input"])
        data["position"].append(row["x"])
        data["target"].append(row["target"])

    return data


if __name__ == "__main__":

    table_names = ["BaseModelTest", "BaseControl"]
    if(len(sys.argv) > 1 and sys.argv[1] == "real"):
        test_name = sys.argv[1]

    t          = []
    x          = []
    u          = []

    for test in test_names:
        data = get_data(table_name, test)
        t.append(data["time"])
        x.append(data["position"])
        u.append(data["input"])
    goal = data["target"] # Same for each

    plt.figure()
    plt.plot(t[0], x[0], "b-")
    plt.plot(t[1], x[1], "k-")
    plt.plot(t[2], x[2], "g-")
    plt.plot(t[2], goal, "r-")
    plt.legend(["PID", "dPID", "Lag"])
    plt.title("Position vs. Target")

    plt.figure()
    plt.plot(t[0], u[0], "b-")
    plt.plot(t[1], u[1], "k-")
    plt.plot(t[2], u[2], "g-")
    plt.legend(["PID", "dPID", "Lag"])
    plt.title("Command Input (N)")

    plt.show()
