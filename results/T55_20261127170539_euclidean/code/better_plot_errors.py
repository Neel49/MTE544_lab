import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_data(filename):
    filename = Path(filename)
    if not filename.exists():
        return {}
    with filename.open(newline="") as file:
        rows = csv.reader(file)
        headers = [h.strip() for h in next(rows, []) if h.strip()]
        values = []
        for row in rows:
            if not any(v.strip() for v in row):
                continue
            row = row[:len(headers)]
            if len(row) != len(headers):
                raise ValueError(f"Incomplete row in {filename}")
            values.append([float(v) if v.strip() else np.nan for v in row])
    array = np.asarray(values, dtype=float).reshape((-1, len(headers))) if headers else np.empty((0, 0))
    return {name: array[:, i] for i, name in enumerate(headers)}


def find_trial(filename):
    filename = Path(filename).resolve()
    if filename.is_dir():
        folder = filename
    elif (filename.parent / "settings.json").exists():
        folder = filename.parent
    else:
        # use the settings belonging to this CSV, never the current code values
        pointer = filename.parent / "latest_trial.json"
        candidates = []
        if pointer.exists():
            candidates.append(filename.parent / json.loads(pointer.read_text())["trial"])
        candidates.extend(p.parent for p in sorted(
            (filename.parent / "results").glob("T*/settings.json"),
            key=lambda p: p.stat().st_mtime, reverse=True))
        folder = None
        for candidate in candidates:
            saved = candidate / filename.name
            if filename.exists() and saved.exists() and filename.read_bytes() == saved.read_bytes():
                folder = candidate
                break
        if folder is None:
            raise ValueError(f"No saved trial matches {filename}. Use a new trial folder or its CSV.")
    settings_file = folder / "settings.json"
    if not settings_file.exists():
        raise ValueError(f"Missing {settings_file}")
    return folder, json.loads(settings_file.read_text())


def matrix_text(name, values):
    matrix = np.asarray(values)
    if np.array_equal(matrix, np.eye(len(matrix)) * matrix[0, 0]):
        return f"{name} = {matrix[0, 0]:g} I{len(matrix)}"
    if np.array_equal(matrix, np.diag(np.diag(matrix))):
        return f"{name} = diag({', '.join(f'{v:g}' for v in np.diag(matrix))})"
    return name + " = " + np.array2string(matrix, precision=6, max_line_width=100)


def parameters(settings):
    sigma = settings.get("map", {}).get("laser_sig", "unavailable")
    first = f"{settings['heuristic'].capitalize()} | laser_sig = {sigma} | {settings['localization']} | {settings['status']}"
    gains = settings["controller"]
    second = "   ".join(f"{axis.capitalize()} (P, D, I) = " +
                        str(tuple(gains[axis][k] for k in ("kp", "kd", "ki")))
                        for axis in ("linear", "angular"))
    lines = [first, second]
    if settings["localization"] == "ekf":
        lines += [matrix_text(key, settings[key]) for key in ("Q", "R") if key in settings]
    return "\n".join(lines)


def format_axis(ax, title, xlabel, ylabel):
    ax.set(title=title, xlabel=xlabel, ylabel=ylabel)
    ax.grid(alpha=0.25)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(fontsize=8)


def save_figure(fig, output, settings, show=False):
    text = settings["trial_name"] + "\n" + parameters(settings)
    extra = max(0, text.count("\n") - 2) * 0.22
    fig.set_size_inches(fig.get_figwidth(), fig.get_figheight() + extra)
    fig.suptitle(text, fontsize=10)
    top = 1 - (0.85 + extra) / fig.get_figheight()
    fig.tight_layout(rect=(0, 0, 1, top))
    output.parent.mkdir(exist_ok=True, parents=True)
    fig.savefig(output, dpi=180)
    print(f"Saved: {output}")
    if show:
        plt.show()
    plt.close(fig)


def draw_map(ax, folder, settings):
    description = settings.get("map")
    image = folder / settings.get("map_pgm", "map/room.pgm")
    if description and image.exists():
        data = plt.imread(image)
        ox, oy, _ = description["origin"]
        res = description["resolution"]
        ax.imshow(data, cmap="gray", vmin=0, vmax=255,
                  extent=[ox, ox + data.shape[1]*res, oy, oy + data.shape[0]*res])
    else:
        ax.text(.02, .98, "Map unavailable for this attempt", transform=ax.transAxes, va="top")
    ax.set_aspect("equal", adjustable="box")


def elapsed(data, settings):
    return (data["stamp"] - settings["start_stamp_ns"]) / 1e9


def plot_trial(folder, settings, show):
    pose = read_data(folder / "robotPose.csv")
    path = read_data(folder / "plannedPath.csv")
    linear = read_data(folder / "linear.csv")
    angular = read_data(folder / "angular.csv")
    if not len(pose.get("x", [])):
        raise ValueError(f"No pose samples in {folder}")
    output = folder / "plots"
    fig, ax = plt.subplots(figsize=(9, 8))
    draw_map(ax, folder, settings)
    if len(path.get("x", [])):
        ax.plot(path["x"], path["y"], "--", color="tab:blue", label="Planned path", linewidth=1.6)
        ax.plot(path["x"][-1], path["y"][-1], "rx", ms=9, label="Final waypoint")
    source = "odometry" if settings["localization"] == "odom" else "EKF"
    ax.plot(pose["x"], pose["y"], color="tab:orange", label=f"Recorded {source} path", linewidth=1.6)
    ax.plot(pose["x"][0], pose["y"][0], "go", label="Start")
    ax.plot(pose["x"][-1], pose["y"][-1], "s", color="tab:orange", label="Recorded end")
    ax.plot(*settings["requested_goal"], "k*", ms=11, label="Requested goal")
    format_axis(ax, "Planned path and recorded position estimate", "x (m)", "y (m)")
    save_figure(fig, output / "path.png", settings, show)

    time = elapsed(pose, settings)
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    for ax, data, title, unit in [(axes[0,0], linear, "Distance error to final waypoint", "Error (m)"),
                                 (axes[0,1], angular, "Heading error to controller target", "Error (rad)")]:
        if len(data.get("e", [])):
            ax.plot(elapsed(data, settings), data["e"], label="Logged controller error")
        else:
            ax.text(.5,.5,"No controller error samples",ha="center",transform=ax.transAxes)
        format_axis(ax, title, "Time from trial start (s)", unit)
    for ax, field, title, unit in [(axes[1,0], "v", "Forward velocity", "v (m/s)"),
                                  (axes[1,1], "w", "Angular velocity", "ω (rad/s)")]:
        ax.plot(time, pose[field], label=f"Recorded {source}")
        ax.step(time, pose["cmd_"+field], where="post", linestyle="--", label="Command")
        format_axis(ax, title, "Time from trial start (s)", unit)
    axes[2,0].plot(time, pose["x"], label="x")
    axes[2,0].plot(time, pose["y"], label="y")
    format_axis(axes[2,0], "Recorded position", "Time from trial start (s)", "Position (m)")
    theta = (pose["theta"] + np.pi) % (2*np.pi) - np.pi
    theta[1:][np.abs(np.diff(theta)) > np.pi] = np.nan
    axes[2,1].plot(time, theta, label="Heading")
    format_axis(axes[2,1], "Recorded heading (wrapped to ±π)", "Time from trial start (s)", "θ (rad)")
    save_figure(fig, output / "tracking.png", settings, show)
    final = np.array([pose["x"][-1], pose["y"][-1]])
    summary = {
        "trial": settings["trial_name"], "status": settings["status"],
        "elapsed_recorded_seconds": float(time[-1] - time[0]),
        "pose_samples": len(time),
        "recorded_estimated_path_length_m": float(np.hypot(np.diff(pose["x"]), np.diff(pose["y"])).sum()),
        "final_estimated_distance_to_requested_goal_m": float(np.linalg.norm(final-settings["requested_goal"])),
    }
    if len(path.get("x", [])):
        summary["planned_path_length_m"] = float(np.hypot(np.diff(path["x"]), np.diff(path["y"])).sum())
        summary["final_estimated_distance_to_final_waypoint_m"] = float(np.linalg.norm(final-[path["x"][-1],path["y"][-1]]))
    (output / "summary.json").write_text(json.dumps(summary, indent=2))


def plot_pid(filename, folder, settings, show):
    data = read_data(filename)
    if not len(data.get("e", [])):
        raise ValueError(f"No error samples in {filename}")
    unit = "rad" if filename.stem == "angular" else "m"
    time = elapsed(data, settings)
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for ax, key, title, ylabel in [
        (axes[0,0], "e", "Error", f"Error ({unit})"),
        (axes[0,1], "e_dot", "Logged error derivative", f"Derivative ({unit}/s)"),
        (axes[1,0], "e_int", "Logged error integral", f"Integral ({unit} s)"),
    ]:
        ax.plot(time, data[key], label=filename.stem.capitalize())
        format_axis(ax, title, "Time from trial start (s)", ylabel)
    axes[1,1].plot(data["e"],data["e_dot"],label="Recorded response")
    format_axis(axes[1,1], "Error versus derivative", f"Error ({unit})", f"Derivative ({unit}/s)")
    save_figure(fig, folder/"plots"/(filename.stem+"_errors.png"), settings, show)


def plot_comparison(trials, show):
    if len(trials) < 2:
        raise ValueError("Use at least two trial folders with --compare")
    signatures = []
    for folder, settings in trials:
        image = folder/settings.get("map_pgm", "map/room.pgm")
        signatures.append((hashlib.sha256(image.read_bytes()).hexdigest(),
                           settings.get("map", {}).get("origin"),
                           settings.get("map", {}).get("resolution")))
    if any(signature != signatures[0] for signature in signatures):
        raise ValueError("These trials use different maps or coordinate origins and cannot be overlaid directly")
    fig = plt.figure(figsize=(14, 8))
    grid = fig.add_gridspec(2,2,width_ratios=[1.3,1])
    map_ax=fig.add_subplot(grid[:,0]); linear_ax=fig.add_subplot(grid[0,1]); angular_ax=fig.add_subplot(grid[1,1])
    draw_map(map_ax,*trials[0])
    labels=[]
    for i,(folder,settings) in enumerate(trials):
        color=f"C{i%10}";label=f"T{settings['trial_number']} {settings['heuristic']}"
        labels.append(f"{label}: "+parameters(settings).replace("\n"," | "))
        pose=read_data(folder/"robotPose.csv");path=read_data(folder/"plannedPath.csv")
        if len(path.get("x",[])):
            map_ax.plot(path["x"],path["y"],"--",color=color,alpha=.65,label=label+" planned")
        if len(pose.get("x",[])):
            map_ax.plot(pose["x"],pose["y"],color=color,label=label+" recorded")
            map_ax.plot(pose["x"][0],pose["y"][0],"o",color=color)
        map_ax.plot(*settings["requested_goal"],"*",color=color,ms=10)
        for ax,filename in [(linear_ax,"linear.csv"),(angular_ax,"angular.csv")]:
            data=read_data(folder/filename)
            if len(data.get("e",[])):
                ax.plot(elapsed(data,settings),data["e"],color=color,label=label)
    format_axis(map_ax,"Paths: circles = starts, stars = requested goals","x (m)","y (m)")
    format_axis(linear_ax,"Distance error to final waypoint","Time from each trial start (s)","Error (m)")
    format_axis(angular_ax,"Heading error to controller target","Time from each trial start (s)","Error (rad)")
    fig.suptitle("Lab 4 trial comparison",fontsize=13)
    fig.text(.02,.015,"\n".join(labels),fontsize=7,va="bottom")
    fig.tight_layout(rect=(0,.07+.025*len(trials),1,.95))
    output=Path.cwd()/"plots";output.mkdir(exist_ok=True)
    filename=output/("compare_"+"_".join(f"T{s['trial_number']}" for _,s in trials)+".png")
    fig.savefig(filename,dpi=180);print(f"Saved: {filename}")
    if show:plt.show()
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description="Plot saved Lab 4 trials")
    parser.add_argument("--files",nargs="+",default=["robotPose.csv"])
    parser.add_argument("--no-show",action="store_true")
    parser.add_argument("--compare",action="store_true")
    args=parser.parse_args()
    try:
        trials=[find_trial(name) for name in args.files]
        if args.compare:
            plot_comparison(trials,not args.no_show)
        else:
            for name,(folder,settings) in zip(args.files,trials):
                requested=Path(name).resolve()
                if requested.name in {"linear.csv","angular.csv"}:
                    plot_pid(folder/requested.name,folder,settings,not args.no_show)
                else:
                    plot_trial(folder,settings,not args.no_show)
                if requested.is_file() and requested.parent != folder:
                    output=requested.parent/"plots";output.mkdir(exist_ok=True)
                    for image in (folder/"plots").iterdir():
                        shutil.copy2(image,output/image.name)
    except (ValueError,OSError,KeyError) as error:
        parser.exit(1,f"Could not plot: {error}\n")


if __name__ == "__main__":
    main()
