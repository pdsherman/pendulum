#!/usr/bin/env python3

"""
Author: pdsherman
Date:   July 2021
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
        data["time"].append(1000.0* row["test_time"])
        data["input"].append(row["input"])
        data["position"].append(row["x"])
        data["target"].append(row["target"])

    return data


if __name__ == "__main__":

    table_name = "DigitalExampleTest"
    test_name  = ["Continuous", "Digital-1", "Digital-2"]

    t = []
    x = []
    u = []
    for name in test_name:
        data = get_data(table_name, name)
        t.append(data["time"])
        x.append(data["position"])
        u.append(data["input"])
    goal = data["target"] # Same for each

    # Create digital data traces
    x_d = []
    t_d = []
    u_d = []
    for k in range(len(test_name)-1):
        x_d.append([])
        t_d.append([])
        u_d.append([])
        x_d[k].append(x[k+1][0])
        t_d[k].append(t[k+1][0])
        for i in range(1, len(u[k])):
            if(u[k+1][i] != u[k+1][i-1]):
                t_d[k].append(t[k+1][i])
                x_d[k].append(x[k+1][i])

    # Plot State
    clr = ["g", "b", "r"]
    plt.figure(num=None, figsize=(13, 5))
    for i in range(len(test_name)):
        if(i != 0):
            plt.plot(t_d[i-1], x_d[i-1], clr[i]+"-o")
        else:
            plt.plot(t[i], x[i], clr[i])
    plt.plot(t[-1], goal, "m--")
    plt.legend(test_name)
    plt.title("Position vs. Target")
    plt.grid(True)
    plt.axis([0.0, 10.0, 0.0, 1.5])
    plt.tight_layout()

    # Plot control input
    plt.figure(num=None, figsize=(13, 5))
    for i in range(len(t)):
        plt.plot(t[i], u[i], clr[i])
    plt.legend(test_name)
    plt.title("Command Input (N)")
    plt.grid(True)
    plt.tight_layout()

    plt.show()
