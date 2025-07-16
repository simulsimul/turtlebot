#!/usr/bin/env python3

"""
Base class for TurtleBot3 Wall Following Algorithms
Converts Webots simulation code to real hardware ROS2 nodes
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from abc import ABC, abstractmethod

class WallFollowerBase(Node, ABC):
    """Base class for all wall following algorithms"""
    
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        # QoS Profile for sensor data (compatible with TurtleBot3)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # State variables
        self.scan_data = None
        self.current_pose = None
        self.last_scan_time = time.time()
        
        # Robot parameters (TurtleBot3 Burger specifications)
        self.wheel_separation = 0.160  # meters
        self.wheel_radius = 0.033  # meters
        self.max_linear_speed = 0.22  # m/s
        self.max_angular_speed = 2.84  # rad/s
        
        # Control parameters
        self.base_speed = 0.15  # m/s
        self.max_speed = 0.2  # m/s
        self.safety_distance = 0.3  # meters
        
        # Auto-start behavior
        self.start_delay = 3.0  # seconds
        self.start_time = time.time()
        self.started = False
        
        self.get_logger().info(f'{node_name} initialized. Starting in {self.start_delay} seconds...')
    
    def scan_callback(self, msg: LaserScan):
        """Process laser scan data"""
        self.scan_data = msg
        self.last_scan_time = time.time()
    
    def odom_callback(self, msg: Odometry):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
    
    def control_loop(self):
        """Main control loop - runs every 0.1 seconds"""
        # Wait for startup delay
        if not self.started:
            if time.time() - self.start_time > self.start_delay:
                self.started = True
                self.get_logger().info(f'{self.get_name()} started!')
            return
        
        # Check if we have recent sensor data
        if self.scan_data is None:
            self.get_logger().warn('No laser scan data received')
            return
        
        # Check if sensor data is too old (timeout)
        if time.time() - self.last_scan_time > 1.0:
            self.get_logger().warn('Laser scan data is too old')
            self.stop_robot()
            return
        
        # Run algorithm-specific control logic
        try:
            twist = self.compute_control_command()
            if twist is not None:
                # Safety limits
                twist = self.apply_safety_limits(twist)
                self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
            self.stop_robot()
    
    @abstractmethod
    def compute_control_command(self) -> Twist:
        """
        Abstract method to be implemented by each algorithm
        Returns: Twist message for robot control
        """
        pass
    
    def convert_webots_to_ros2_ranges(self, webots_ranges):
        """
        Convert Webots lidar range format to ROS2 LaserScan format
        Webots: starts from back (180°), clockwise
        ROS2: starts from front (0°), counter-clockwise
        """
        if len(webots_ranges) != 360:
            return webots_ranges
        
        # Rotate array to start from front instead of back
        ros2_ranges = webots_ranges[180:] + webots_ranges[:180]
        # Reverse to make it counter-clockwise
        ros2_ranges.reverse()
        return ros2_ranges
    
    def get_range_at_angle(self, angle_deg: float) -> float:
        """
        Get range measurement at specific angle
        Args:
            angle_deg: Angle in degrees (0=front, 90=left, 270=right)
        Returns:
            Range in meters
        """
        if self.scan_data is None:
            return float('inf')
        
        ranges = self.scan_data.ranges
        if len(ranges) == 0:
            return float('inf')
        
        # Convert angle to array index
        angle_rad = math.radians(angle_deg)
        angle_normalized = (angle_rad - self.scan_data.angle_min) / self.scan_data.angle_increment
        index = int(round(angle_normalized)) % len(ranges)
        
        range_value = ranges[index]
        
        # Filter invalid readings
        if math.isnan(range_value) or math.isinf(range_value):
            return self.scan_data.range_max
        
        return max(self.scan_data.range_min, min(range_value, self.scan_data.range_max))
    
    def get_min_range_in_arc(self, start_angle: float, end_angle: float) -> float:
        """
        Get minimum range in an angular arc
        Args:
            start_angle: Start angle in degrees
            end_angle: End angle in degrees
        Returns:
            Minimum range in meters
        """
        if self.scan_data is None:
            return float('inf')
        
        ranges = self.scan_data.ranges
        if len(ranges) == 0:
            return float('inf')
        
        # Convert angles to indices
        start_rad = math.radians(start_angle)
        end_rad = math.radians(end_angle)
        
        start_idx = int((start_rad - self.scan_data.angle_min) / self.scan_data.angle_increment)
        end_idx = int((end_rad - self.scan_data.angle_min) / self.scan_data.angle_increment)
        
        # Handle wrap-around
        if start_idx > end_idx:
            selected_ranges = ranges[start_idx:] + ranges[:end_idx]
        else:
            selected_ranges = ranges[start_idx:end_idx]
        
        # Filter valid ranges
        valid_ranges = [r for r in selected_ranges 
                       if not (math.isnan(r) or math.isinf(r)) 
                       and self.scan_data.range_min <= r <= self.scan_data.range_max]
        
        if not valid_ranges:
            return self.scan_data.range_max
        
        return min(valid_ranges)
    
    def wheel_speeds_to_twist(self, left_speed: float, right_speed: float) -> Twist:
        """
        Convert wheel speeds to Twist message
        Args:
            left_speed: Left wheel speed (rad/s)
            right_speed: Right wheel speed (rad/s)
        Returns:
            Twist message
        """
        twist = Twist()
        
        # Convert wheel speeds to linear and angular velocities
        linear_vel = (left_speed + right_speed) * self.wheel_radius / 2.0
        angular_vel = (right_speed - left_speed) * self.wheel_radius / self.wheel_separation
        
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        return twist
    
    def apply_safety_limits(self, twist: Twist) -> Twist:
        """
        Apply safety limits to twist command
        Args:
            twist: Input twist command
        Returns:
            Safety-limited twist command
        """
        # Limit linear velocity
        if abs(twist.linear.x) > self.max_linear_speed:
            twist.linear.x = math.copysign(self.max_linear_speed, twist.linear.x)
        
        # Limit angular velocity
        if abs(twist.angular.z) > self.max_angular_speed:
            twist.angular.z = math.copysign(self.max_angular_speed, twist.angular.z)
        
        return twist
    
    def stop_robot(self):
        """Stop the robot immediately"""
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
    
    def constrain(self, value: float, min_val: float, max_val: float) -> float:
        """Constrain value between min and max"""
        return max(min_val, min(value, max_val))
    
    def log_sensor_info(self):
        """Log current sensor information for debugging"""
        if self.scan_data is None:
            return
        
        front_dist = self.get_range_at_angle(0)    # Front
        left_dist = self.get_range_at_angle(90)    # Left
        right_dist = self.get_range_at_angle(270)  # Right
        
        self.get_logger().info(
            f'Sensor readings - Front: {front_dist:.2f}m, '
            f'Left: {left_dist:.2f}m, Right: {right_dist:.2f}m'
        )

def main(args=None):
    """Main function - should be overridden by specific algorithms"""
    print("This is a base class. Use specific algorithm implementations.")
    return

if __name__ == '__main__':
    main() 