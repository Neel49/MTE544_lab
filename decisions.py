# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        self.publisher= self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # Run the localization node
        spin_once(self.localizer, timeout_sec = 0.0)

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # Check if we reached the goal
        # Add a tolerance of 0.5 cm to goal position because it is unlikely we will reach exactly 0 cm of error.
        # Only check linear displacement because orientation at goal position doesn't matter. 
        # Angular error is used to guide robot towards goal position.
        if type(self.goal) == list:
            reached_goal= calculate_linear_error(self.localizer.getPose(), self.goal[-1]) < 0.005
        else: 
            reached_goal= calculate_linear_error(self.localizer.getPose(), self.goal) < 0.005
        

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            # exit the spin
            raise SystemExit
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        # Publish the velocity to move the robot
        vel_msg.linear.x = float(velocity)
        vel_msg.angular.z = float(yaw_rate)
        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    
    odom_qos=QoSProfile(reliability=1, durability=2, history=1, depth=10)
    

    # instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [-1.0, -1.0])
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [-1.0, -1.0], motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
