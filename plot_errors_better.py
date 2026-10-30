"""Plot Lab 2 CSVs: python3 plot_errors_better.py --files robot_pose.csv linear.csv angular.csv"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_data(filename):
    # Read every measurement, including the first row after the header.
    with open(filename, newline="") as file:
        reader = csv.reader(file)
        headers = [value.strip() for value in next(reader, []) if value.strip()]
        rows = []
        for row in reader:
            if not any(value.strip() for value in row):
                continue
            # Keep nanosecond timestamps as integers until we subtract the start.
            rows.append([float(row[0]), float(row[1]), float(row[2]), int(row[3])])
    return headers, rows


def gain_labels(folder):
    # Trial names store each gain pair as linear first, angular second.
    # Example: P_0.6_0.75 D_0.5_0.1 I_0.2_0.2
    names = {"P": ("klp", "kap"), "D": ("klv", "kav"), "I": ("kli", "kai")}
    labels = []
    for word in folder.name.split():
        parts = word.split("_")
        if len(parts) == 3 and parts[0] in names:
            linear, angular = names[parts[0]]
            labels.append(f"{linear}={parts[1]}, {angular}={parts[2]}")
    if labels:
        return "Gains from folder name: " + "; ".join(labels)
    return "Gains not recorded in the folder name"


def plot_errors(filename, headers, rows, start_stamp):
    time = [(row[3] - start_stamp) / 1e9 for row in rows]
    gains = gain_labels(filename.parent)

    if headers[0] == "x":
        x = [row[0] for row in rows]
        y = [row[1] for row in rows]
        theta = [row[2] for row in rows]
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))

        axes[0].plot(x, y, color="blue", label="Path")
        # Compare the recorded path with the default sigmoid from planner.py.
        sigmoid_x = np.linspace(0.0, 2.5, 101)
        sigmoid_y = 2.0 / (1 + np.exp(-2.0 * sigmoid_x)) - 1.0
        axes[0].plot(sigmoid_x, sigmoid_y, color="orange", linestyle="--",
                     label="Planned sigmoid")
        axes[0].scatter(x[0], y[0], color="green", label="Start")
        axes[0].scatter(x[-1], y[-1], color="red", marker="x", label="End")
        axes[0].set(title="Robot trajectory", xlabel="x (m)", ylabel="y (m)")
        axes[0].axis("equal")

        axes[1].plot(time, x, color="blue", label="x")
        axes[1].plot(time, y, color="orange", linestyle="--", label="y")
        axes[1].set(title="Position versus time", xlabel="Time (s)", ylabel="Position (m)")

        axes[2].plot(time, theta, color="purple", label="theta")
        axes[2].set(title="Theta versus time", xlabel="Time (s)", ylabel="Theta (rad)")
        title = "Robot pose"
    else:
        # The standard filenames distinguish angular and linear error logs.
        angular = "angular" in filename.stem.lower()
        unit = "rad" if angular else "m"
        title = "Angular error" if angular else "Linear error"
        error = [row[0] for row in rows]
        derivative = [row[1] for row in rows]
        integral = [row[2] for row in rows]
        fig, grid = plt.subplots(2, 2, figsize=(12, 8))
        axes = grid.flatten()

        axes[0].plot(error, derivative, color="blue", label="Recorded response")
        axes[0].set(title="Derivative versus error", xlabel=f"Error ({unit})",
                    ylabel=f"Error derivative ({unit}/s)")
        axes[1].plot(time, error, color="blue", label="Error")
        axes[1].set(title="Error versus time", xlabel="Time (s)", ylabel=f"Error ({unit})")
        axes[1].axhline(0, color="gray", linestyle="--")
        axes[2].plot(time, derivative, color="orange", label="Logged derivative")
        axes[2].set(title="Error derivative versus time", xlabel="Time (s)",
                    ylabel=f"Error derivative ({unit}/s)")
        axes[3].plot(time, integral, color="green", label="Logged integral")
        axes[3].set(title="Error integral versus time", xlabel="Time (s)",
                    ylabel=f"Error integral ({unit} s)")

    for ax in axes:
        ax.grid()
        ax.legend()
    fig.suptitle(f"{title} | {filename.parent.name}\n{gains}")
    fig.tight_layout()
    print(f"{filename}: {gains}")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot Lab 2 pose and error CSVs with units.")
    parser.add_argument("--files", nargs="+", required=True, help="CSV files to plot")
    args = parser.parse_args()

    datasets = []
    start_times = {}
    for name in args.files:
        filename = Path(name).resolve()
        headers, rows = read_data(filename)
        if not rows:
            print(f"Skipping {filename}: no recorded measurements in this file.")
            continue
        datasets.append((filename, headers, rows))
        folder = filename.parent
        # Files from the same trial share a time origin when plotted together.
        start_times[folder] = min(start_times.get(folder, rows[0][3]), rows[0][3])

    for filename, headers, rows in datasets:
        plot_errors(filename, headers, rows, start_times[filename.parent])
