"""Save room.pgm and room.yaml in the current folder, including map alignment."""
import math
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
import yaml


def main():
    rclpy.init()
    node = Node("save_alignment")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        # record how map coordinates relate to odometry while SLAM is running
        deadline = time.monotonic() + 10
        transform = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform("map", "odom", rclpy.time.Time())
                break
            except TransformException:
                continue
        if transform is None:
            raise RuntimeError("No map-to-odom transform. Keep SLAM running and check ROS_DOMAIN_ID.")

        subprocess.run([
            "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "room",
            "--ros-args", "-p", "save_map_timeout:=15.0",
        ], check=True)
        filename = Path("room.yaml")
        data = yaml.safe_load(filename.read_text())
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        data["odom_pose_in_map"] = [transform.transform.translation.x,
                                    transform.transform.translation.y, yaw]
        # keep origin on one line with decimals for the supplied map reader
        data["origin"] = [float(value) for value in data["origin"]]
        filename.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=None, width=1000))
        print(f"Saved map to {filename.resolve().parent}")
        print("Saved alignment", data["odom_pose_in_map"])
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
