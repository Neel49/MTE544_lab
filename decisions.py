

import sys
import math
import time
import a_star
from trial_logging import TrialRecorder


from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once, shutdown
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensors, kalmanFilter

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController


from geometry_msgs.msg import PoseStamped


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class decision_maker(Node):
    
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER, localization_type=rawSensors):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)


        self.create_subscription(PoseStamped, "/goal_pose", self.designPathFor, 10)
        
        self.pathPublisher = self.create_publisher(Path, '/designedPath', 10)
        
        publishing_period=1/rate

        self.reachThreshold= 0.05 # 5 cm tolerance was used for lab 2 and lab 3

        self.localizer=localization(localization_type)


        
        if motion_type==POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
            self.planner=planner(POINT_PLANNER)

        
        elif motion_type==TRAJECTORY_PLANNER:
            # Gains from Lab 2
            # klp, klv and kli are the linear P, D and I gains
            # kap, kav and kai are the angular P, D and I gains
            self.controller=trajectoryController(klp=0.3, klv=0.2, kli=0.4,kap=0.5, kav=0.1, kai=0.3)      
            self.planner=planner(TRAJECTORY_PLANNER)
        
        else:
            print("Error! you don't have this type of planner", file=sys.stderr)


        self.goal = None
        self.motion_type = motion_type
        self.trial = None
        self.loggers = [self.localizer.loc_logger, self.controller.PID_linear.logger,
                        self.controller.PID_angular.logger]
        for logger in self.loggers:
            logger.enabled = False
        self.recordedPathPublisher = self.create_publisher(Path, '/recordedPath', 10)
        self.recordedPosePublisher = self.create_publisher(PoseStamped, '/recordedPose', 10)
        self.recordedPath = Path()
        self.last_path_stamp = None

        self.create_timer(publishing_period, self.timerCallback)


        print("waiting for your input position, use 2D nav goal in rviz2")




    # each RViz goal starts a separate trial
    def designPathFor(self, msg: PoseStamped):
        spin_once(self.localizer, timeout_sec=0.1)
        pose = self.localizer.getPose()
        if pose is None:
            print("waiting for odom msgs ....")
            return
        self.stop_robot()
        if self.trial is not None:
            self.trial.finish("replaced", pose, "A new goal was selected")
        self.goal = None
        self.recordedPath = Path()
        self.recordedPath.header.frame_id = "map"
        self.recordedPathPublisher.publish(self.recordedPath)
        self.last_path_stamp = None
        requested = [msg.pose.position.x, msg.pose.position.y]
        metadata = {
            "motion": "trajectory" if self.motion_type == TRAJECTORY_PLANNER else "point",
            "heuristic": "manhattan" if a_star.USE_MANHATTAN else "euclidean",
            "localization": "odom" if self.localizer.type == rawSensors else "ekf",
            "start_pose": [float(v) for v in pose[:3]],
            "start_stamp_ns": pose[3].sec * 1000000000 + pose[3].nanosec,
            "requested_goal": requested,
            "reach_threshold_metres": self.reachThreshold,
            "controller": {},
            "command_line": list(sys.argv),
        }
        for axis, pid in [("linear", self.controller.PID_linear),
                          ("angular", self.controller.PID_angular)]:
            metadata["controller"][axis] = {"kp": pid.kp, "kd": pid.kv, "ki": pid.ki,
                                           "history_length": pid.history_length}
        if hasattr(self.localizer, "kf"):
            kf = self.localizer.kf
            metadata.update(Q=kf.Q.copy(), R=kf.R.copy(), P_at_start=kf.P.copy(),
                            state_at_start=kf.x.copy())
        self.trial = TrialRecorder(metadata, self.loggers, self.planner.mapName)
        self.localizer.last_logged_stamp = None
        self.localizer.log_current_pose()
        try:
            self.goal = self.planner.plan(pose[:2], requested)
            path = self.goal if self.motion_type == TRAJECTORY_PLANNER else [self.goal]
            self.trial.save_plan(path, getattr(self.planner, "m_utilites", None))
        except Exception as error:
            self.goal = None
            self.trial.finish("planning_failed", pose, str(error))
            self.publishPathOnRviz2([])
            self.get_logger().error(f"Planning failed: {error}")
            return
        if self.goal is None or len(self.goal) == 0:
            self.goal = None
            self.trial.finish("no_path", pose, "No usable path to this goal")
            self.publishPathOnRviz2([])
            print("No usable path to this goal. Choose another goal or check the padding.")
            return
        self.publishPathOnRviz2(path)

    def stop_robot(self):
        self.localizer.command = [0.0, 0.0]
        self.publisher.publish(Twist())

    def publishRecordedPose(self):
        pose = self.localizer.getPose()
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = pose[3]
        message.pose.position.x = float(pose[0])
        message.pose.position.y = float(pose[1])
        message.pose.orientation.z = math.sin(float(pose[2]) / 2)
        message.pose.orientation.w = math.cos(float(pose[2]) / 2)
        self.recordedPosePublisher.publish(message)
        stamp = pose[3].sec * 1000000000 + pose[3].nanosec
        if self.trial is not None and self.trial.settings["status"] == "running":
            if stamp != self.last_path_stamp:
                self.recordedPath.poses.append(message)
                self.last_path_stamp = stamp
        self.recordedPath.header = message.header
        self.recordedPathPublisher.publish(self.recordedPath)

    def timerCallback(self):
        previous_pose = self.localizer.getPose()
        previous_stamp = previous_pose[3] if previous_pose is not None else None
        spin_once(self.localizer, timeout_sec=0.1)

        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        
        # wait for a new sensor sample before updating the controller
        if self.localizer.getPose()[3] == previous_stamp:
            return
        self.publishRecordedPose()
        vel_msg=Twist()
        
        if self.goal is None:
            return
        
        if self.motion_type == TRAJECTORY_PLANNER:
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal[-1]) <self.reachThreshold else False
        else: 
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal) <self.reachThreshold else False




        if reached_goal:
            print("reached goal")
            self.stop_robot()
            self.trial.finish("reached", self.localizer.getPose())
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()


            
            self.goal = None
            print("waiting for the new position input, use 2D nav goal on map")

            return
        
        velocity, yaw_rate = self.controller.\
            vel_request(self.localizer.getPose(), self.goal, True)

        
        vel_msg.linear.x=velocity
        vel_msg.angular.z=yaw_rate
        
        self.localizer.command = [float(velocity), float(yaw_rate)]
        self.publisher.publish(vel_msg)
        self.publishPathOnRviz2(self.goal if self.motion_type == TRAJECTORY_PLANNER else [self.goal])



    def publishPathOnRviz2(self, path):

        Path_ =  Path()

        Path_.header.frame_id ="map"
        Path_.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            # Set the orientation of the pose. Here, it's set to a default orientation.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            Path_.poses.append(pose)

        self.pathPublisher.publish(Path_)

import argparse
def main(args=None):
    # keep ROS active long enough to send a stop command on Ctrl+C
    init(signal_handler_options=SignalHandlerOptions.NO)
    if getattr(args, "heuristic", None):
        a_star.USE_MANHATTAN = args.heuristic == "manhattan"
    motion = TRAJECTORY_PLANNER if args.motion == "trajectory" else POINT_PLANNER
    source = kalmanFilter if getattr(args, "localization", "odom") == "ekf" else rawSensors
    DM = decision_maker(Twist, "/cmd_vel", 10, motion_type=motion, localization_type=source)
    status = "interrupted"
    try:
        spin(DM)
    except KeyboardInterrupt:
        print("Stopped")
    except Exception:
        status = "error"
        raise
    finally:
        for _ in range(3):
            DM.stop_robot()
            time.sleep(0.05)
        if DM.trial is not None:
            DM.trial.finish(status, DM.localizer.getPose())
        DM.localizer.destroy_node()
        DM.destroy_node()
        shutdown()


if __name__ == "__main__":
    argParser = argparse.ArgumentParser(description="point or trajectory")
    argParser.add_argument("--motion", choices=["point", "trajectory"], default="trajectory")
    argParser.add_argument("--heuristic", choices=["manhattan", "euclidean"], default=None)
    argParser.add_argument("--localization", choices=["odom", "ekf"], default="odom")
    main(argParser.parse_args())
