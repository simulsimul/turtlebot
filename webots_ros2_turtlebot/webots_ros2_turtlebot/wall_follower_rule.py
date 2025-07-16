#!/usr/bin/env python3

"""
Rule-based Wall Following Algorithm for TurtleBot3
Ported from Webots simulation to real hardware ROS2 node
"""

import rclpy
from geometry_msgs.msg import Twist
from .wall_follower_base import WallFollowerBase
import math

class WallFollowerRule(WallFollowerBase):
    """Rule-based wall following algorithm using truth table logic"""
    
    def __init__(self):
        super().__init__('wall_follower_rule')
        
        # Rule-based algorithm parameters
        self.front_threshold = 0.30  # meters
        self.side_threshold = 0.30   # meters
        self.corner_threshold = 0.30 # meters
        
        # Speed parameters (converted from Webots values)
        self.base_speed = 0.15      # m/s (was BASE_SPEED = 5 in Webots)
        self.turn_speed_ratio = 0.1  # for turning (was BASE_SPEED/9 in Webots)
        
        self.get_logger().info('Rule-based wall follower initialized')
        self.get_logger().info(f'Thresholds - Front: {self.front_threshold}m, '
                              f'Side: {self.side_threshold}m, Corner: {self.corner_threshold}m')
    
    def compute_control_command(self) -> Twist:
        """
        Implement rule-based wall following logic
        Based on truth table from original Webots code
        """
        # Get sensor readings at key angles
        front_dist = self.get_range_at_angle(0)    # Front (0°)
        left_dist = self.get_range_at_angle(90)    # Left (90°)
        corner_dist = self.get_range_at_angle(45)  # Left corner (45°)
        
        # Wall detection based on thresholds
        front_wall = front_dist < self.front_threshold
        left_wall = left_dist < self.side_threshold
        left_corner = corner_dist < self.corner_threshold
        
        # Log detection status
        self.get_logger().info(
            f'Wall detection - Front: {front_wall} ({front_dist:.2f}m), '
            f'Left: {left_wall} ({left_dist:.2f}m), '
            f'Corner: {left_corner} ({corner_dist:.2f}m)'
        )
        
        # Apply rule-based logic (truth table)
        if front_wall:
            # Priority 1: Avoid front wall
            return self.rotate_right()
        elif left_corner:
            # Priority 2: Handle corner case
            return self.rotate_right()
        elif left_wall:
            # Priority 3: Wall on left, move forward
            return self.move_forward()
        else:
            # Priority 4: No wall on left, turn left to find wall
            return self.turn_left()
    
    def move_forward(self) -> Twist:
        """Move forward at base speed"""
        twist = Twist()
        twist.linear.x = self.base_speed
        twist.angular.z = 0.0
        
        self.get_logger().debug('Action: Moving forward')
        return twist
    
    def turn_left(self) -> Twist:
        """Turn left while moving forward"""
        twist = Twist()
        twist.linear.x = self.base_speed * self.turn_speed_ratio
        twist.angular.z = self.base_speed / 0.160  # Convert to angular velocity
        
        self.get_logger().debug('Action: Turning left')
        return twist
    
    def turn_right(self) -> Twist:
        """Turn right while moving forward"""
        twist = Twist()
        twist.linear.x = self.base_speed * self.turn_speed_ratio
        twist.angular.z = -self.base_speed / 0.160  # Convert to angular velocity
        
        self.get_logger().debug('Action: Turning right')
        return twist
    
    def rotate_left(self) -> Twist:
        """Rotate left in place"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.base_speed / 0.080  # Faster rotation
        
        self.get_logger().debug('Action: Rotating left')
        return twist
    
    def rotate_right(self) -> Twist:
        """Rotate right in place"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.base_speed / 0.080  # Faster rotation
        
        self.get_logger().debug('Action: Rotating right')
        return twist
    
    def move_backward(self) -> Twist:
        """Move backward"""
        twist = Twist()
        twist.linear.x = -self.base_speed
        twist.angular.z = 0.0
        
        self.get_logger().debug('Action: Moving backward')
        return twist

def main(args=None):
    """Main function to run the rule-based wall follower"""
    rclpy.init(args=args)
    
    try:
        wall_follower = WallFollowerRule()
        
        # Log startup information
        wall_follower.get_logger().info('Rule-based wall follower started')
        wall_follower.get_logger().info('Algorithm: Truth table based decision making')
        wall_follower.get_logger().info('Behavior: Follow left wall, avoid obstacles')
        
        rclpy.spin(wall_follower)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'wall_follower' in locals():
            wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 