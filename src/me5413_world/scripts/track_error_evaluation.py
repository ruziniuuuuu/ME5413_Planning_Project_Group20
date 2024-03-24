#!/usr/bin/env python

import csv
import math
import matplotlib.pyplot as plt

def evaluate_track_errors(errors_file):
    position_errors = []
    heading_errors = []
    speed_errors = []
    times = []

    with open(errors_file, 'r') as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            position_errors.append(float(row[0]))
            heading_errors.append(float(row[1]))
            speed_errors.append(float(row[2]))
            times.append(i)

    def rmse(errors):
        return math.sqrt(sum(err**2 for err in errors) / len(errors))

    rmse_position = rmse(position_errors)
    rmse_heading = rmse(heading_errors)
    rmse_speed = rmse(speed_errors)

    print(f"RMSE Position: {rmse_position:.3f}")
    print(f"RMSE Heading: {rmse_heading:.3f}")
    print(f"RMSE Speed: {rmse_speed:.3f}")


    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

    ax1.plot(times, position_errors)
    ax1.set_title(f"Position Error (RMSE: {rmse_position:.3f})")
    ax1.set_ylabel("Error (m)")

    ax2.plot(times, heading_errors)
    ax2.set_title(f"Heading Error (RMSE: {rmse_heading:.3f})")
    ax2.set_ylabel("Error (rad)")

    ax3.plot(times, speed_errors)
    ax3.set_title(f"Speed Error (RMSE: {rmse_speed:.3f})")
    ax3.set_xlabel("Time")
    ax3.set_ylabel("Error (m/s)")
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    errors_file = '../result/errors.csv'
    # errors_file = '../result/pid_stanley/errors_pid_Stanley.csv'
    # errors_file = '../result/pid_purePursuit/errors_pid_purePursuit.csv'

    evaluate_track_errors(errors_file)
