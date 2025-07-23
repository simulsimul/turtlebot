#!/usr/bin/env python3

"""
Auto Navigator Node for TurtleBot3
Automatically starts moving when powered on
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

class AutoNavigator(Node):
    def __init__(self):
        super().__init__('auto_navigator')
        
        # QoS Profile for sensor data (compatible with TurtleBot3)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers with proper QoS
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.scan_data = None
        self.current_pose = None
        self.state = 'FORWARD'  # FORWARD, TURN_LEFT, TURN_RIGHT, STOP
        self.obstacle_distance = 0.05  # meters
        self.linear_speed = 0.10  # m/s
        self.angular_speed = 0.25  # rad/s
        
        # Auto-start behavior
        self.start_delay = 5.0  # seconds
        self.start_time = time.time()
        self.started = False
        
        self.get_logger().info('Auto Navigator initialized. Starting in 5 seconds...')
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = msg
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
    
    def control_loop(self):
        """Main control loop - runs every 0.1 seconds"""
        # Wait for startup delay
        if not self.started:
            if time.time() - self.start_time > self.start_delay:
                self.started = True
                self.get_logger().info('Auto Navigator started!')
            return
        
        # Check if we have sensor data
        if self.scan_data is None:
            return
        
        # Simple obstacle avoidance behavior
        twist = Twist()
        
        # Get front, left, and right distances
        ranges = self.scan_data.ranges
        if len(ranges) == 0:
            return
        
        # Front: 0 degrees (index 0)
        # Left: 90 degrees (index len/4)
        # Right: 270 degrees (index 3*len/4)
        front_dist = self.get_min_distance(ranges, 0, 30)  # ±30 degrees
        left_dist = self.get_min_distance(ranges, 60, 120)  # 60-120 degrees
        right_dist = self.get_min_distance(ranges, 240, 300)  # 240-300 degrees
        
        # Decision making
        if front_dist > self.obstacle_distance:
            # Path is clear, move forward
            self.state = 'FORWARD'
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        elif left_dist > right_dist:
            # Turn left
            self.state = 'TURN_LEFT'
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        else:
            # Turn right
            self.state = 'TURN_RIGHT'
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        
        # Log current state
        self.get_logger().info(
            f'State: {self.state}, Front: {front_dist:.2f}m, '
            f'Left: {left_dist:.2f}m, Right: {right_dist:.2f}m'
        )
    
    def get_min_distance(self, ranges, start_angle, end_angle):
        """Get minimum distance in a range of angles"""
        total_points = len(ranges)
        start_idx = int((start_angle / 360.0) * total_points)
        end_idx = int((end_angle / 360.0) * total_points)
        
        if start_idx > end_idx:
            # Handle wrap-around
            selected_ranges = ranges[start_idx:] + ranges[:end_idx]
        else:
            selected_ranges = ranges[start_idx:end_idx]
        
        # Filter out invalid readings
        valid_ranges = [r for r in selected_ranges if r > 0.1 and r < 10.0]
        
        if not valid_ranges:
            return 10.0  # Default to far distance if no valid readings
        
        return min(valid_ranges)

def main(args=None):
    rclpy.init(args=args)
    
    auto_navigator = AutoNavigator()
    
    try:
        rclpy.spin(auto_navigator)
    except KeyboardInterrupt:
        pass
    
    auto_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 