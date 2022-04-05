#!/usr/bin/env python

"""
File:   encoder_one_test.py
Author: pdsherman
Date:   March 2022

Description: Script to graph data from encoder test
"""

import math
import time
import argparse
import sqlite3
import matplotlib.pyplot as plt

DB_FILE = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/PendulumDatabase.db"

def get_sensor_data(table, test):
    conn = sqlite3.connect(DB_FILE)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    command = "SELECT * FROM {} WHERE test_name = '{}' ORDER BY test_time".format(table, test)

    c.execute(command)
    data = {'t': [], 'theta': []}
    for row in c:
        data['t'].append(row['test_time'])
        data['theta'].append(row['theta'])
    data['t'] = [x - data['t'][0] for x in data['t']]

    conn.close()
    return data

def plot_test(data, test_name):
    plt.figure()
    plt.plot(data['t'], data['theta'])
    plt.title(test_name)
    plt.xlabel("time (s)")
    plt.ylabel("theta (rad)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Graph Data from encoder test")
    parser.add_argument('table', help="Table in SQLite database")
    parser.add_argument('test',  help="Test name to graph")
    args = parser.parse_args()

    data = get_sensor_data(args.table, args.test)
    plot_test(data, args.test)
