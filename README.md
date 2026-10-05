# MTE544 Lab 1 - Neel

Branch: `lab1_neel`. Snapshot of the saved Lab 1 code from the Ubuntu ROS VM on October 4, 2026.

The `main` branch contains the repository overview and shared configuration. This branch contains the Lab 1 implementation and supplied plotting helper.

## Files

| File | Purpose |
| --- | --- |
| `motions.py` | Publishes motion commands and subscribes to IMU, odometry, and laser scans. |
| `utilities.py` | Provides CSV logging, file reading, and quaternion-to-yaw conversion. |
| `filePlotter.py` | Supplied helper for plotting scalar sensor recordings. |
| `LICENSE` | Original MIT license for the course starter. |

Keep the three Python files together. Both `motions.py` and `filePlotter.py` import `utilities.py`.

## Environment

- Ubuntu 22.04 with ROS 2 Humble and `rclpy`.
- TurtleBot3 Burger with the Gazebo simulation installed.
- ROS message packages: `geometry_msgs`, `sensor_msgs`, and `nav_msgs`.
- Python 3 and Matplotlib for the plotting helper.

Initialize ROS in a terminal if your shell does not already do so:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Run

Use one running Gazebo simulation. If it is not already running, launch it in a terminal with GUI access:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Stop keyboard teleop before running the motion script, so it does not send competing velocity commands. In another ROS terminal, change to this repository's directory and run one motion:

```bash
python3 motions.py --motion circle
```

The other accepted motions are `spiral` and `line`. The script starts publishing motion after it has received IMU, odometry, and laser messages. A colcon build is not required for these standalone scripts.

The current exit handler only prints a message on Ctrl+C. After exiting the controller, explicitly send zero velocity:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Recordings and plotting

The script writes `imu_content_<motion>.csv`, `odom_content_<motion>.csv`, and `laser_content_<motion>.csv` into the current working directory. Rerunning a motion overwrites those three files. Preserve recordings separately for the lab submission; generated CSV files are ignored by Git.

The supplied plotting helper can be adapted for the recordings:

```bash
python3 filePlotter.py --files imu_content_circle.csv odom_content_circle.csv
```

Read the limitations below before interpreting its results.

## Current limitations

This branch preserves the current saved implementation, including the following follow-ups:

- `FileReader.read_file()` has an extra `next(file)` that skips the first measurement.
- Laser ranges are written as array text containing commas. The current numeric reader cannot parse these rows; laser serialization and parsing need to be adapted.
- An unsupported motion name reaches `arg.motion` instead of `args.motion`. Use `circle`, `spiral`, or `line` until this is corrected.
- Ctrl+C does not automatically publish a stop command.
- Motion speeds and radius are current simulation experiments. Inspect and tune them for the physical robot; straight-line and spiral speeds increase during execution.
- The plotting helper currently uses raw nanosecond timestamp differences and generic labels; adapt time units and labels for the report.

## Course source

The code is based on [UW-MTE544/MTE544_student, LabOne](https://github.com/UW-MTE544/MTE544_student/tree/LabOne). See the [lab instructions](https://github.com/UW-MTE544/MTE544_student/blob/LabOne/README.md) and [rubric](https://github.com/UW-MTE544/MTE544_student/blob/LabOne/rubrics.md).

The original MIT license and copyright notice are preserved in `LICENSE`.
