import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_data(filename):
    # Ignore the empty column left by the logger's trailing comma.
    with filename.open() as file:
        rows = list(csv.reader(file))
    headers = [name.strip() for name in rows[0] if name.strip()]
    values = np.array([
        [float(value) for value in row if value.strip()]
        for row in rows[1:] if any(value.strip() for value in row)
    ])
    return {name: values[:, i] for i, name in enumerate(headers)}


def find_settings(filename):
    settings_file = filename.parent / "settings.json"
    if settings_file.exists():
        return json.loads(settings_file.read_text()), filename.parent.name

    # For the main CSV, find the archived copy with identical contents.
    # Never label old data with the settings currently written in the code.
    candidates = sorted(
        (filename.parent / "results").glob("*/settings.json"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    contents = filename.read_bytes()
    for candidate in candidates:
        archived = candidate.parent / filename.name
        if archived.exists() and archived.read_bytes() == contents:
            return json.loads(candidate.read_text()), candidate.parent.name
    return None, filename.stem


def covariance_label(name, matrix):
    matrix = np.array(matrix)
    size = len(matrix)
    if np.array_equal(matrix, np.eye(size) * matrix[0, 0]):
        return rf"$ {name} = {matrix[0, 0]} I_{{{size}}} $"
    if np.array_equal(matrix, np.diag(np.diag(matrix))):
        entries = ", ".join(str(value) for value in np.diag(matrix))
        return rf"$ {name} = \mathrm{{diag}}({entries}) $"
    # Preserve off-diagonal entries if a full covariance was used.
    return name + " =\n" + np.array2string(matrix, floatmode="unique", max_line_width=100)


def format_axis(ax, title, xlabel, ylabel):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend()


def plot_file(filename, show):
    data = read_data(filename)
    settings, trial = find_settings(filename)
    time = data["stamp"] - data["stamp"][0]

    if "imu_ax" in data:
        # A separate companion log preserves the original robotPose.csv format.
        raw_file = filename.with_name(filename.stem + "_raw.csv")
        if settings and (filename.parent / "results" / trial).is_dir():
            raw_file = filename.parent / "results" / trial / raw_file.name
        raw = read_data(raw_file) if raw_file.exists() else None
        if raw is not None and (
            len(raw["stamp"]) != len(data["stamp"])
            or not np.array_equal(raw["stamp"], data["stamp"])
        ):
            print("Companion timestamps do not match; plotting without raw odometry.")
            raw = None

        fig, axes = plt.subplots(2, 3, figsize=(15, 9))
        ax = axes[0, 0]
        if raw is not None:
            ax.plot(raw["odom_x"], raw["odom_y"], color="blue",
                    label="Odometry (/odom)")
        ax.plot(data["kf_x"], data["kf_y"], color="orange",
                linestyle="--", label="EKF")
        ax.plot(data["kf_x"][0], data["kf_y"][0], "go", label="EKF start")
        ax.plot(data["kf_x"][-1], data["kf_y"][-1], "rx", label="EKF end")
        if settings and settings["motion"] == "point":
            goal = settings.get("goal")
            if goal is not None:
                ax.plot(goal[0], goal[1], "k*", markersize=10, label="Goal")
        ax.axis("equal")
        format_axis(ax, "Position estimates (not ground truth)", "x (m)", "y (m)")

        for ax, field, title, unit in [
            (axes[0, 1], "v", "Forward velocity", "v (m/s)"),
            (axes[0, 2], "w", "Angular velocity", r"$\omega$ (rad/s)"),
        ]:
            if raw is not None:
                ax.plot(time, raw["odom_" + field], color="blue",
                        label="Odometry measurement")
            key = "kf_vx" if field == "v" else "kf_w"
            ax.plot(time, data[key], color="orange", linestyle="--", label="EKF")
            format_axis(ax, title, "Time (s)", unit)

        for ax, direction in zip(axes[1, :2], ["x", "y"]):
            ax.plot(time, data["imu_a" + direction], color="blue",
                    linewidth=1, label="IMU measurement")
            ax.plot(time, data["kf_a" + direction], color="orange",
                    linestyle="--", linewidth=1.5, label="EKF")
            format_axis(ax, f"{direction} acceleration", "Time (s)",
                        rf"$a_{direction}$ (m/s²)")

        ax = axes[1, 2]
        if raw is not None:
            # Wrap both headings to the same range; do not shift their origins.
            for key, label, color, style in [
                ("odom_theta", "Odometry (/odom)", "blue", "-"),
                ("kf_theta", "EKF", "orange", "--"),
            ]:
                theta = (raw[key] + np.pi) % (2 * np.pi) - np.pi
                ax.plot(time, theta, color=color, linestyle=style, label=label)
            format_axis(ax, "Heading (wrapped to ±π)", "Time (s)", r"$\theta$ (rad)")
        else:
            ax.plot(time, data["kf_x"], color="blue", label="EKF x")
            ax.plot(time, data["kf_y"], color="orange", linestyle="--", label="EKF y")
            format_axis(ax, "Estimated position", "Time (s)", "Position (m)")
            print("Older recording: raw odometry and heading were not logged.")
        event_folder = filename.parent
        if settings and (filename.parent / "results" / trial).is_dir():
            event_folder = filename.parent / "results" / trial
        timing_file = event_folder / "pick_timing.json"
        if timing_file.exists():
            event = json.loads(timing_file.read_text())
            start = event["start_stamp_seconds"] - data["stamp"][0]
            end = event.get("end_stamp_seconds", data["stamp"][-1]) - data["stamp"][0]
            for ax in list(axes.flat)[1:]:
                ax.axvspan(start, end, color="gray", alpha=0.2, label="Scripted pickup")
                ax.legend()
            index = np.argmin(np.abs(time - start))
            axes[0, 0].plot(data["kf_x"][index], data["kf_y"][index],
                            "ks", label="Pickup start")
            axes[0, 0].legend()
        plot_name = "ekf"
    else:
        # The supplied PID logger uses nanoseconds.
        time = time / 1e9
        angular = "angular" in filename.stem.lower()
        unit = "rad" if angular else "m"
        kind = "Angular" if angular else "Linear"
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))

        ax = axes[0, 0]
        ax.plot(time, data["e"], color="blue", label="Error")
        ax.axhline(0, color="black", linewidth=0.5)
        format_axis(ax, f"{kind} error", "Time (s)", f"Error ({unit})")

        ax = axes[0, 1]
        ax.plot(time, data["e_dot"], color="orange", label="Logged derivative")
        format_axis(ax, "Error derivative", "Time (s)", f"Error derivative ({unit}/s)")

        ax = axes[1, 0]
        ax.plot(time, data["e_int"], color="green", label="Logged integral")
        format_axis(ax, "Error integral", "Time (s)", f"Error integral ({unit} s)")

        ax = axes[1, 1]
        ax.plot(data["e"], data["e_dot"], color="blue", label="Recorded response")
        format_axis(ax, "Derivative versus error", f"Error ({unit})",
                    f"Error derivative ({unit}/s)")
        plot_name = kind.lower() + "_errors"

    if settings:
        title = settings["motion"].capitalize() + " — " + plot_name.replace("_", " ")
        covariance = (covariance_label("Q", settings["Q"]) + "     " +
                      covariance_label("R", settings["R"]))
    else:
        title = filename.stem
        covariance = "Q and R unavailable: this recording has no matching saved settings."

    # Allow room for full matrices when off-diagonal entries are used.
    title_lines = covariance.count("\n") + 2
    width, height = fig.get_size_inches()
    height += max(0, title_lines - 2) * 0.25
    fig.set_size_inches(width, height)
    title_space = (0.5 + title_lines * 0.23) / height
    fig.suptitle(title + "\n" + covariance, fontsize=13)
    fig.text(0.5, 0.01, trial, ha="center", fontsize=8)
    fig.tight_layout(rect=(0, 0.04, 1, 1 - title_space))

    output = filename.parent / "plots"
    output.mkdir(exist_ok=True)
    image = output / (filename.stem + "_" + plot_name + ".png")
    fig.savefig(image, dpi=200)
    print(f"Saved: {image}")
    if show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--files", nargs="+", required=True)
    parser.add_argument("--no-show", action="store_true",
                        help="Save PNGs without opening windows")
    args = parser.parse_args()

    for name in args.files:
        plot_file(Path(name).resolve(), show=not args.no_show)
