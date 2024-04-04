#!/usr/bin/env python
import csv
import math
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.gridspec import GridSpec
import argparse

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

    fig = plt.figure(figsize=(10, 8))
    gs = GridSpec(3, 1, height_ratios=[1, 1, 1])

    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])
    ax3 = fig.add_subplot(gs[2])

    # Customize the appearance of each subplot
    for ax in [ax1, ax2, ax3]:
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.tick_params(labelsize=10)

    # Plot position error
    ax1.plot(times, position_errors, linewidth=1.5, color='#1f77b4')
    ax1.set_title(f"Position Error (RMSE: {rmse_position:.3f})", fontsize=12)
    ax1.set_ylabel("Error (m)", fontsize=10)
    ax1.xaxis.set_tick_params(labelbottom=False)

    # Plot heading error
    ax2.plot(times, heading_errors, linewidth=1.5, color='#ff7f0e')
    ax2.set_title(f"Heading Error (RMSE: {rmse_heading:.3f})", fontsize=12)
    ax2.set_ylabel("Error (rad)", fontsize=10)
    ax2.xaxis.set_tick_params(labelbottom=False)

    # Plot speed error
    ax3.plot(times, speed_errors, linewidth=1.5, color='#2ca02c')
    ax3.set_title(f"Speed Error (RMSE: {rmse_speed:.3f})", fontsize=12)
    ax3.set_xlabel("Time", fontsize=10)
    ax3.set_ylabel("Error (m/s)", fontsize=10)

    # Set y-axis tick format
    ax1.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
    ax2.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
    ax3.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Evaluate track errors')
    parser.add_argument('errors_file', type=str, help='Path to the errors CSV file')
    args = parser.parse_args()

    evaluate_track_errors(args.errors_file)