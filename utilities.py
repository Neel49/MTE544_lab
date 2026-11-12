from math import atan2, asin, sqrt

M_PI=3.1415926535

import numpy as np

class Logger:
    instances = []

    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename
        self.archive_filename = None
        Logger.instances.append(self)

        with open(self.filename, 'w') as file:
            file.write("".join(f"{header}, " for header in headers) + "\n")

    def log_values(self, values_list):
        row = "".join(f"{value}, " for value in values_list) + "\n"

        # Keep the usual CSV and the trial copy identical.
        with open(self.filename, 'a') as file:
            file.write(row)
        if self.archive_filename is not None:
            with open(self.archive_filename, 'a') as file:
                file.write(row)

    def save_log(self):
        pass


def start_trial(motion, kf, controller, goal, reach_threshold):
    from datetime import datetime
    from pathlib import Path
    import json
    import shutil

    # Short labels for scalar identity matrices; full matrices are saved below.
    def matrix_label(matrix):
        diagonal = np.diag(matrix)
        if np.array_equal(matrix, np.eye(len(diagonal)) * diagonal[0]):
            return f"{diagonal[0]:g}"
        return "matrix"

    # Continue numbering from the existing T1_, T2_, ... folders.
    results = Path("results")
    results.mkdir(exist_ok=True)
    trial_number = 1
    for previous in results.iterdir():
        prefix = previous.name.split("_")[0]
        if previous.is_dir() and prefix.startswith("T") and prefix[1:].isdigit():
            trial_number = max(trial_number, int(prefix[1:]) + 1)

    started = datetime.now().astimezone()
    name = (
        f"T{trial_number}_{started:%Y%m%d%H%M%S}_{motion}"
        f"_Q{matrix_label(kf.Q)}_R{matrix_label(kf.R)}"
        f"_P{matrix_label(kf.P)}"
    )
    folder = results / name
    folder.mkdir(parents=True, exist_ok=False)

    settings = {
        "trial_number": trial_number,
        "started_at": started.isoformat(),
        "motion": motion,
        "state_order": ["x", "y", "theta", "omega", "v", "v_dot"],
        "measurement_order": ["v", "omega", "ax", "ay"],
        "initial_state": kf.x.tolist(),
        "P_initial": kf.P.tolist(),
        "Q": kf.Q.tolist(),
        "R": kf.R.tolist(),
        "prediction_dt_seconds": kf.dt,
        "goal": goal,
        "reach_threshold_metres": reach_threshold,
        "csv_files": [Path(logger.filename).name for logger in Logger.instances],
    }
    if controller is not None:
        settings["controller"] = {}
        for axis, pid in [("linear", controller.PID_linear),
                          ("angular", controller.PID_angular)]:
            settings["controller"][axis] = {
                "type": pid.type,
                "kp": pid.kp, "kd": pid.kv, "ki": pid.ki,
                "history_length": pid.history_length,
            }

    (folder / "settings.json").write_text(
        json.dumps(settings, indent=2), encoding="utf-8"
    )

    # Save the code too, including motion settings and any later tuning edits.
    code_folder = folder / "code"
    code_folder.mkdir()
    for source in Path(__file__).resolve().parent.glob("*.py"):
        shutil.copy2(source, code_folder / source.name)

    # Only archive logs created by this run, not CSVs left from earlier runs.
    for logger in Logger.instances:
        logger.archive_filename = folder / Path(logger.filename).name
        shutil.copyfile(logger.filename, logger.archive_filename)

    print(f"Saving trial to: {folder.resolve()}")
    return folder


class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line
            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    
# Conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    # just unpack yaw for tb
    return yaw


# Calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    return sqrt( (current_pose[0] - goal_pose[0])**2 +
                (current_pose[1] - goal_pose[1])**2 )

# Calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    error_angular= atan2(goal_pose[1]-current_pose[1],
                        goal_pose[0]-current_pose[0]) - current_pose[2]
    
    if error_angular <= -M_PI:
        error_angular += 2*M_PI
    
    
    elif error_angular >= M_PI:
        error_angular -= 2*M_PI
    
    return error_angular