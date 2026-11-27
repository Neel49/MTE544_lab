import csv
import io
import json
import shutil
import subprocess
from datetime import datetime
from pathlib import Path

import numpy as np


def write_json(filename, values):
    def convert(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(f"Cannot save {type(value).__name__}")

    filename = Path(filename)
    temporary = filename.with_suffix(filename.suffix + ".tmp")
    temporary.write_text(json.dumps(values, indent=2, default=convert), encoding="utf-8")
    temporary.replace(filename)


class Logger:
    def __init__(self, filename, headers=None):
        self.filename = str(filename)
        self.headers = list(headers or ["e", "e_dot", "e_int", "stamp"])
        self.archive_filename = None
        self.enabled = True
        self.started = False

    def reset(self, archive_filename=None):
        self.archive_filename = archive_filename
        self.enabled = True
        self.started = True
        for filename in self.destinations():
            with Path(filename).open("w", newline="") as file:
                csv.writer(file, lineterminator="\n").writerow(self.headers)

    def destinations(self):
        return [self.filename] + ([self.archive_filename] if self.archive_filename else [])

    def log_values(self, values_list):
        if not self.enabled:
            return
        if len(values_list) != len(self.headers):
            raise ValueError(f"{self.filename}: CSV values do not match the headers")
        if not self.started:
            self.reset()
        buffer = io.StringIO()
        csv.writer(buffer, lineterminator="\n").writerow(values_list)
        # write both copies immediately so an interrupted run keeps its data
        for filename in self.destinations():
            with Path(filename).open("a", newline="") as file:
                file.write(buffer.getvalue())

    def save_log(self):
        # each row is already written and the file is closed
        pass


class TrialRecorder:
    def __init__(self, metadata, loggers, map_name="room", source_dir=None):
        self.loggers = loggers
        self.root = Path.cwd()
        source_dir = Path(source_dir).resolve() if source_dir else Path(__file__).resolve().parent
        results = self.root / "results"
        results.mkdir(exist_ok=True)
        numbers = [int(p.name.split("_")[0][1:]) for p in results.iterdir()
                   if p.is_dir() and p.name.split("_")[0][1:].isdigit()
                   and p.name.startswith("T")]
        number = max(numbers, default=0) + 1
        started = datetime.now().astimezone()
        while True:
            name = f"T{number}_{started:%Y%m%d%H%M%S}_{metadata['heuristic']}"
            self.folder = results / name
            try:
                self.folder.mkdir()
                break
            except FileExistsError:
                number += 1
        self.settings = dict(metadata, trial_number=number, trial_name=name,
                             started_at=started.isoformat(), status="running",
                             timestamp_unit="nanoseconds", pose_frame="odom",
                             path_frame="map", map_and_odom_assumed_aligned=True)
        self.settings["csv_files"] = [Path(log.filename).name for log in loggers] + ["plannedPath.csv"]
        code = self.folder / "code"
        code.mkdir()
        for source in source_dir.iterdir():
            if source.is_file() and source.suffix in {".py", ".rviz"}:
                shutil.copy2(source, code / source.name)
        try:
            git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=source_dir,
                                 capture_output=True, text=True, timeout=2)
            self.settings["git_commit"] = git.stdout.strip() if git.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired):
            self.settings["git_commit"] = None
        # keep the exact map used by this attempt alongside the code
        map_folder = self.folder / "map"
        map_folder.mkdir()
        base = Path(map_name)
        if base.suffix in {".yaml", ".pgm"}:
            base = base.with_suffix("")
        for extension in (".pgm", ".yaml"):
            source = base.with_suffix(extension)
            if source.exists():
                target = map_folder / ("room" + extension)
                shutil.copy2(source, target)
                self.settings["map" + extension.replace(".", "_")] = str(target.relative_to(self.folder))
        for logger in self.loggers:
            logger.reset(self.folder / Path(logger.filename).name)
        self.path_logger = Logger("plannedPath.csv", ["index", "x", "y"])
        self.path_logger.reset(self.folder / "plannedPath.csv")
        self.save_settings()
        write_json(self.root / "latest_trial.json", {"trial": str(self.folder.relative_to(self.root))})
        print(f"Saving trial to: {self.folder}", flush=True)

    def save_settings(self):
        write_json(self.folder / "settings.json", self.settings)

    def save_plan(self, path, map_utilities=None):
        if map_utilities is not None:
            self.settings["map"] = {
                "resolution": float(map_utilities.res),
                "origin": [float(map_utilities.o_x), float(map_utilities.o_y), 0.0],
                "width": map_utilities.width, "height": map_utilities.height,
                "laser_sig": float(map_utilities.laser_sig),
                "blocked_above": 0.8,
            }
            np.save(self.folder / "map" / "costMap.npy", map_utilities.likelihood_field)
        if path:
            for i, point in enumerate(path):
                self.path_logger.log_values([i, float(point[0]), float(point[1])])
            self.settings["final_waypoint"] = [float(v) for v in path[-1]]
        self.settings["planned_points"] = len(path) if path else 0
        self.save_settings()

    def finish(self, status, pose=None, detail=None):
        if self.settings["status"] != "running":
            return
        self.settings["status"] = status
        self.settings["finished_at"] = datetime.now().astimezone().isoformat()
        if pose is not None:
            self.settings["final_pose"] = [float(v) for v in pose[:3]]
            self.settings["end_stamp_ns"] = pose[3].sec * 1000000000 + pose[3].nanosec
        if detail:
            self.settings["detail"] = detail
        for logger in self.loggers:
            logger.enabled = False
            logger.archive_filename = None
        self.save_settings()
        print(f"Trial {self.settings['trial_number']}: {status}", flush=True)
