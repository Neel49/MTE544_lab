import sys
from utilities import Logger

from rclpy.time import Time

from utilities import *
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters



rawSensors=0; kalmanFilter=1

odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)


class localization(Node):
    
    def __init__(self, type, loggerName="robotPose.csv", loggerHeaders=None):

        super().__init__("localizer")
        
        
        headers = loggerHeaders or ["stamp", "x", "y", "theta", "v", "w",
                                    "odom_x", "odom_y", "odom_theta", "odom_v", "odom_w",
                                    "cmd_v", "cmd_w", "imu_ax", "imu_ay"]
        self.loc_logger = Logger(loggerName, headers)
        self.loc_logger.enabled = False
        self.type = type
        self.command = [0.0, 0.0]
        self.latest_sample = None
        self.last_logged_stamp = None
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors();
        elif type==kalmanFilter:
            self.initKalmanfilter()
            self.kalmanInitialized = False
        else:
            print("We don't have this type for localization", sys.stderr)
            return            
    
        self.last_fusion_stamp_ns = None
    
    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initKalmanfilter(self):
        
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        
        time_syncher.registerCallback(self.fusion_callback)
        
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):

        if not self.kalmanInitialized:
            x=np.array([odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        euler_from_quaternion(odom_msg.pose.pose.orientation),
                        0,
                        0,
                        0])        
            
            # Q and R matrices from lab Three
            Q = 1.0 * np.eye(6)
            R = np.diag([0.25, 0.25, 0.25, 0.25])
            P = 0.01*np.eye(6)
                        
            self.kf=kalman_filter(P,Q,R, x)
            
            self.kalmanInitialized = True

        
        # integrate sensor time, not the time delayed messages arrive at the computer
        stamp_ns = Time.from_msg(odom_msg.header.stamp).nanoseconds
        if self.last_fusion_stamp_ns is None:
            dt = 0.0
        else:
            dt = (stamp_ns - self.last_fusion_stamp_ns) / 1e9
            if dt <= 0:
                return
        self.last_fusion_stamp_ns = stamp_ns


        z=np.array([odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.angular.z,
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y])
        
        self.kf.predict(dt)
        self.kf.update(z)
        
        xhat=self.kf.get_states()
        
        self.pose=np.array([xhat[0],
                            xhat[1],
                            normalize_angle(xhat[2]),
                            odom_msg.header.stamp])
        
        self.record_sample(odom_msg, xhat[4], xhat[3],
                           imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y)

    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]
        
        self.record_sample(pose_msg, pose_msg.twist.twist.linear.x,
                           pose_msg.twist.twist.angular.z)

    def record_sample(self, odom_msg, velocity, yaw_rate, imu_ax="", imu_ay=""):
        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        raw = odom_msg.pose.pose
        # save the selected pose, raw odometry and the most recent velocity command
        self.latest_sample = [stamp, *[float(v) for v in self.pose[:3]],
                              float(velocity), float(yaw_rate), raw.position.x, raw.position.y,
                              euler_from_quaternion(raw.orientation),
                              odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z,
                              *self.command, imu_ax, imu_ay]
        self.log_current_pose()

    def log_current_pose(self):
        if (self.loc_logger.enabled and self.latest_sample is not None
                and self.latest_sample[0] != self.last_logged_stamp):
            self.latest_sample[11:13] = self.command
            self.loc_logger.log_values(self.latest_sample)
            self.last_logged_stamp = self.latest_sample[0]

        
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    
    spin(LOCALIZER)
