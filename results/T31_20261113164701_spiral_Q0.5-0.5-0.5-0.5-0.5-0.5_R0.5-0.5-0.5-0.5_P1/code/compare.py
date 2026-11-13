"""Compare saved EKF trials: python3 compare.py --T1T2 or --trials T1 T2 T3."""

import argparse
import json
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from better_plot_errors import read_data, covariance_label


def load_trial(root, name):
    name = name.upper()
    if not re.fullmatch(r"T\d+", name):
        raise ValueError(f"Expected a trial number such as T1, got {name}.")
    folders = sorted(path for path in root.glob(name + "_*") if path.is_dir())
    if len(folders) != 1:
        raise ValueError(f"Expected one folder for {name} in {root}; found {len(folders)}.")
    folder = folders[0]
    settings = json.loads((folder / "settings.json").read_text())
    data = read_data(folder / "robotPose.csv")
    raw_path = folder / "robotPose_raw.csv"
    raw = read_data(raw_path) if raw_path.exists() else None
    if raw is not None and not np.array_equal(raw["stamp"], data["stamp"]):
        raise ValueError(f"{name}: companion timestamps do not match robotPose.csv.")
    if raw is None:
        print(f"{name}: older recording; odometry and heading comparison unavailable.")
    return name, settings, data, raw


def heading(values):
    # Break the line at angle wraps so it does not draw a vertical jump.
    values = (values + np.pi) % (2 * np.pi) - np.pi
    values = values.copy()
    values[np.flatnonzero(np.abs(np.diff(values)) > np.pi) + 1] = np.nan
    return values


def compare(trials, output, show):
    motions = {settings["motion"] for _, settings, _, _ in trials}
    if len(motions) > 1:
        print("NOTE: different motions selected; this is not a controlled Q/R comparison.")
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    for i, (name, settings, data, raw) in enumerate(trials):
        color = plt.get_cmap("tab10")(i % 10)
        time = data["stamp"] - data["stamp"][0]
        axes[0, 0].plot(data["kf_x"], data["kf_y"], color=color, label=name + " EKF")
        axes[0, 0].plot(data["kf_x"][0], data["kf_y"][0], "o", color=color)
        if raw is not None:
            axes[0, 0].plot(raw["odom_x"], raw["odom_y"], ":", color=color,
                            alpha=0.65, label=name + " /odom")
        for ax, estimate, measurement, source in [
            (axes[0, 1], "kf_vx", "odom_v", raw),
            (axes[0, 2], "kf_w", "odom_w", raw),
            (axes[1, 0], "kf_ax", "imu_ax", data),
            (axes[1, 1], "kf_ay", "imu_ay", data),
        ]:
            ax.plot(time, data[estimate], color=color, label=name + " EKF")
            if source is not None:
                ax.plot(time, source[measurement], ":", color=color, alpha=0.65,
                        label=name + " measured")
        if raw is not None:
            axes[1, 2].plot(time, heading(raw["kf_theta"]), color=color, label=name + " EKF")
            axes[1, 2].plot(time, heading(raw["odom_theta"]), ":", color=color,
                            alpha=0.65, label=name + " /odom")

    for ax, title, ylabel in zip(axes.flat,
            ["Position estimates (not ground truth)", "Forward velocity", "Angular velocity",
             "Forward acceleration", "Sideways acceleration (EKF: vω)", "Heading (wrapped to ±π)"],
            ["y (m)", "v (m/s)", "ω (rad/s)", "aₓ (m/s²)", "aᵧ (m/s²)", "θ (rad)"]):
        ax.set(title=title, xlabel="Time since first sample (s)", ylabel=ylabel)
        ax.grid(alpha=0.3)
        if ax.lines:
            ax.legend(fontsize=8)
        else:
            ax.text(0.5, 0.5, "Not logged in these trials", ha="center", transform=ax.transAxes)
    axes[0, 0].set_xlabel("x (m)")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    title = " vs ".join(trial[0] for trial in trials)
    fig.suptitle(title + " — solid: EKF; dotted: measurements; dots: EKF starts")
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    # Keep saved matrices on a separate page so full Q/R matrices stay readable.
    blocks = []
    for name, settings, _, _ in trials:
        blocks.append(name + " — " + settings["motion"] + "\n" +
                      covariance_label("Q", settings["Q"]) + "\n" +
                      covariance_label("R", settings["R"]) + "\n" +
                      covariance_label("P₀", settings["P_initial"]) + "\n" +
                      f"Prediction dt = {settings['prediction_dt_seconds']} s")
    heights = [max(2.0, (block.count('\n') + 2) * 0.28) for block in blocks]
    params, rows = plt.subplots(len(blocks), 1, figsize=(14, sum(heights)),
                               gridspec_kw={"height_ratios": heights}, squeeze=False)
    for ax, block in zip(rows.flat, blocks):
        ax.axis("off")
        ax.text(0.02, 0.95, block, va="top", transform=ax.transAxes, fontsize=11)
    params.tight_layout()
    output.mkdir(parents=True, exist_ok=True)
    stem = "_vs_".join(trial[0] for trial in trials)
    for figure, suffix in [(fig, "comparison"), (params, "settings")]:
        path = output / f"{stem}_{suffix}.png"
        figure.savefig(path, dpi=180)
        print(f"Saved: {path}")
    if show:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", nargs="+", required=True)
    parser.add_argument("--results", type=Path, default=Path("results"))
    parser.add_argument("--output", type=Path, default=Path("plots/comparisons"))
    parser.add_argument("--no-show", action="store_true")
    # Accept the short form requested for tab-friendly trial numbers.
    arguments = []
    for arg in sys.argv[1:]:
        if re.fullmatch(r"--(?:T\d+){2,}", arg, re.IGNORECASE):
            arguments.extend(["--trials", *re.findall(r"T\d+", arg.upper())])
        else:
            arguments.append(arg)
    args = parser.parse_args(arguments)
    if len(set(name.upper() for name in args.trials)) < 2:
        parser.error("Select at least two different trials.")
    try:
        trials = [load_trial(args.results, name) for name in dict.fromkeys(args.trials)]
        compare(trials, args.output, not args.no_show)
    except (ValueError, OSError, KeyError) as error:
        parser.exit(1, f"Cannot compare trials: {error}\n")
