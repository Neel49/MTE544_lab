# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        # Turning radius in meters. Increased after each spiral command
        self.radius_=0.2

        # Straight line speed starts at 0.05 m/s and is capped at 0.22 m/s
        self.acc_line_x_speed_=0.05
        self.max_line_x_speed_=0.22 # Documentation online says 0.22 m/s for the burger
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, 'cmd_vel', 10) # (message type, topic name, queue size)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        # The Burger simulation topics publishers are set to RELIABLE, VOLATILE, and UNKNOWN respectively. Instead of UNKNOWN I changed suscriber TO KEEP_LAST policy with depth of 10 msgs.
        qos=QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        self.imu_subscription=self.create_subscription(Imu, '/imu', self.imu_callback, qos) # (message type, topic name, callback function, qos)
        self.odom_subscription=self.create_subscription(Odometry, '/odom', self.odom_callback, qos) # (message type, topic name, callback function, qos)
        self.laser_subscription=self.create_subscription(LaserScan, '/scan', self.laser_callback, qos) # (message type, topic name, callback function, qos)
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        """Log ax, ay, yaw rate, and timestamp"""
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
        message = [imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.angular_velocity.z, timestamp]
        self.imu_logger.log_values(message)

        if not self.imu_initialized:
            self.imu_initialized = True
    def odom_callback(self, odom_msg: Odometry):
        "Log x,y, yaw, timestamp"
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(orientation)
        message = [position.x, position.y, yaw, timestamp]
        self.odom_logger.log_values(message)

        if not self.odom_initialized:
            self.odom_initialized = True
                
    def laser_callback(self, laser_msg: LaserScan):
        "Log range list, angular increment, timestamp"
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        message = [list(laser_msg.ranges), laser_msg.angle_increment, timestamp]
        self.laser_logger.log_values(message)

        if not self.laser_initialized:
            self.laser_initialized = True
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        """Command a continuous circle using constant v and omega = v / R."""
        msg=Twist()
        msg.linear.x = 0.2
        msg.angular.z = msg.linear.x / self.radius_
        return msg

    def make_spiral_twist(self):
        """Expand the turning radius with every callback whiel keeping linear velocity constant"""
        msg=Twist()
        msg.linear.x = 0.2
        msg.angular.z = msg.linear.x / self.radius_
        self.radius_ += 0.01
        return msg
    
    def make_acc_line_twist(self):
        """Increase linear forward speed up to its limit while keeping omega at 0. Acceleration is set to 0.1 m/s^2."""
        msg=Twist()
        msg.linear.x = min(self.acc_line_x_speed_, self.max_line_x_speed_)
        msg.angular.z = 0.0
        self.acc_line_x_speed_ = min(self.acc_line_x_speed_ + 0.01, self.max_line_x_speed_)
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
