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

TEST_NAME  = "BaseControlTest"

def get_data(table_name, filter=None):
    data = {"time": [], "input": [], "position": [], "target": []}
    database = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/PendulumDatabase.db"
    command  = "SELECT * FROM {} where test = \"{}\"".format(table_name, TEST_NAME)
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

    if(len(sys.argv) > 1 and sys.argv[1] == "real"):
        table_name = "BaseControl"
    else:
        table_name = "BaseModelTest"

    data = get_data(table_name)
    t    = data["time"]
    x    = data["position"]
    u    = data["input"]
    goal = data["target"]

    plt.figure()
    plt.plot(t, x, "b-")
    plt.plot(t, goal, "r-")
    plt.title("Position vs. Target")

    plt.figure()
    plt.plot(t, u)
    plt.title("Command Input (N)")

    plt.show()
