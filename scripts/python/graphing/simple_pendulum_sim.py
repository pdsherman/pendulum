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

def get_sensor_data(table, col_name, test):
    conn = sqlite3.connect(DB_FILE)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    command = "SELECT * FROM {} WHERE {} = '{}' ORDER BY test_time".format(table, col_name, test)

    c.execute(command)
    data = {'t': [], 'theta': [], 'theta_dot': []}
    for row in c:
        data['t'].append(row['test_time'])
        data['theta'].append(row['theta'])
    data['t'] = [x - data['t'][0] for x in data['t']]

    conn.close()
    return data

def adjust_data(data):
    idx = 0
    for i in range(len(data['theta'])):
        if(data['theta'][i+1] < data['theta'][i]):
            idx = i
            break

    data['t'] = data['t'][idx:]
    data['theta'] = data['theta'][idx:]
    data['t'] = [x-data['t'][0] for x in data['t']]

    return data

def plot_test(data, test_name, line_style = "k-"):
    plt.figure(figsize=(10.5, 4.8))
    plt.plot(data['t'], data['theta'], line_style)
    plt.title("Pendulum Angle")
    plt.xlabel("time (s)")
    plt.ylabel("theta (rad)")
    plt.axis([0.0, 62.2, -3.2, 0.1])
    plt.grid(True)
    plt.tight_layout()

def plot_both(real, drag, friction):
    plt.figure(figsize=(10.5, 4.8))
    plt.plot(real['t'], real['theta'], 'b')
    plt.plot(friction['t'], friction['theta'], 'r')
    plt.title("Pendulum Friction")
    plt.xlabel("time (s)")
    plt.ylabel("theta (rad)")
    plt.legend(["Real", "Model"], loc='upper right')
    plt.axis([0.0, 62.2, -3.2, 0.1])
    plt.grid(True)
    plt.tight_layout()

    plt.figure(figsize=(10.5, 4.8))
    plt.plot(real['t'], real['theta'], 'b')
    plt.plot(drag['t'], drag['theta'], 'r')
    plt.title("Pendulum Drag")
    plt.xlabel("time (s)")
    plt.ylabel("theta (rad)")
    plt.legend(["Real", "Model"], loc='upper right')
    plt.axis([0.0, 62.2, -3.2, 0.1])
    plt.grid(True)
    plt.tight_layout()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pendulum Model Fitting")
    parser.add_argument('--suffix', required=False, default="")
    args = parser.parse_args()

    sim_table = "SimplePendulumSim"
    sim_col   = "model"

    suffix = "" if (not args.suffix) else "_" + args.suffix
    drag_data      = get_sensor_data(sim_table, sim_col, "drag"+suffix)
    friction_data  = get_sensor_data(sim_table, sim_col, "friction"+suffix)

    real_table = "EncoderOneTest"
    real_test  = "real_03"
    real_col   = "test_name"
    real_data = get_sensor_data(real_table, real_col, real_test)
    real_data = adjust_data(real_data)

    plot_both(real_data, drag_data, friction_data)
    plt.show()
