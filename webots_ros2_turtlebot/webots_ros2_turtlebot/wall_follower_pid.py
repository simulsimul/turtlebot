#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from .wall_follower_base import WallFollowerBase
import math

class WallFollowerPID(WallFollowerBase):
    """PID-based wall following algorithm for maintaining lateral distance"""
    
    def __init__(self):
        super().__init__('wall_follower_pid')
        
        # PID Controller parameters
        self.kp = 30.0  # Proportional gain
        self.kd = 12.0  # Derivative gain  
        self.ki = 0.0   # Integral gain (disabled for stability)
        
        # Alternative Tustin discretized PID parameters
        self.use_tustin = False  # Set to True to use Tustin method
        self.kp_tustin = 50.0
        self.kd_tustin = 25.0
        self.ki_tustin = 0.0
        
        # Control parameters
        self.desired_distance = 0.20  # Desired lateral distance to wall (meters)
        self.front_threshold = 0.25   # Front obstacle threshold (meters)
        self.base_speed = 0.15        # Base forward speed (m/s)
        self.max_correction = 0.1     # Maximum correction speed (m/s)
        
        # PID state variables
        self.last_error = 0.0
        self.last_last_error = 0.0  # For Tustin method
        self.cumulative_error = 0.0
        self.last_correction = 0.0  # For Tustin method
        
        # Anti-windup parameters
        self.anti_windup_limit = 20.0
        
        # Control loop timing
        self.dt = 0.1  # Control loop period (seconds)
        
        self.get_logger().info('PID-based wall follower initialized')
        self.get_logger().info(f'PID gains - Kp: {self.kp}, Kd: {self.kd}, Ki: {self.ki}')
        self.get_logger().info(f'Desired distance: {self.desired_distance}m')
        self.get_logger().info(f'Using Tustin method: {self.use_tustin}')
    
    def compute_control_command(self) -> Twist:
        """
        Implement PID-based wall following logic
        """
        # Get sensor readings
        front_dist = self.get_min_range_in_arc(-10, 10)  # Front arc (±10°)
        left_dist = self.get_min_range_in_arc(80, 100)   # Left side arc (90° ±10°)
        
        # Check for front obstacle first (safety priority)
        if front_dist < self.front_threshold:
            self.get_logger().info(f'Front obstacle detected at {front_dist:.2f}m - rotating right')
            return self.rotate_right()
        
        # PID control for wall following
        return self.pid_control(left_dist)
    
    def pid_control(self, current_distance: float) -> Twist:
        """
        Apply PID control to maintain desired distance from wall
        """
        # Calculate error
        error = current_distance - self.desired_distance
        
        # Calculate derivative
        rate_error = error - self.last_error
        
        # Calculate integral with anti-windup
        self.cumulative_error += error
        if abs(self.cumulative_error) > self.anti_windup_limit:
            self.cumulative_error = math.copysign(self.anti_windup_limit, self.cumulative_error)
        
        # Choose PID method
        if self.use_tustin:
            correction = self.tustin_pid_control(error)
        else:
            correction = self.simple_pid_control(error, rate_error)
        
        # Convert correction to wheel speeds and then to Twist
        left_speed = self.base_speed - correction
        right_speed = self.base_speed + correction
        
        # Limit speeds
        left_speed = self.constrain(left_speed, -self.max_speed, self.max_speed)
        right_speed = self.constrain(right_speed, -self.max_speed, self.max_speed)
        
        # Convert to Twist message
        twist = self.wheel_speeds_to_twist(
            left_speed / self.wheel_radius,   # Convert to rad/s
            right_speed / self.wheel_radius
        )
        
        # Update state for next iteration
        self.last_last_error = self.last_error
        self.last_error = error
        
        # Log control information
        self.get_logger().info(
            f'PID Control - Distance: {current_distance:.3f}m, '
            f'Error: {error:.3f}m, Correction: {correction:.3f}, '
            f'Speeds: L={left_speed:.3f}, R={right_speed:.3f}'
        )
        
        return twist
    
    def simple_pid_control(self, error: float, rate_error: float) -> float:
        """
        Simple discrete PID implementation
        """
        correction = (self.kp * error + 
                     self.kd * rate_error + 
                     self.ki * self.cumulative_error)
        
        return self.constrain(correction, -self.max_correction, self.max_correction)
    
    def tustin_pid_control(self, error: float) -> float:
        """
        Tustin discretized PID implementation
        Based on the difference equation from the original code
        """
        # Tustin discretized PID equation:
        # u_k = u_(k-1) + Kp(e_k - e_(k-1)) + (Kp*T)/Ti * e_k + (Kp*Td)/T * (e_k - 2*e_(k-1) + e_(k-2))
        
        # Calculate integral term with anti-windup
        integral_value = self.ki_tustin * error
        integral_value = self.constrain(integral_value, -self.anti_windup_limit, self.anti_windup_limit)
        
        # Calculate correction using Tustin method
        correction = (self.last_correction + 
                     self.kp_tustin * (error - self.last_error) + 
                     integral_value + 
                     self.kd_tustin * (error - 2*self.last_error + self.last_last_error))
        
        # Update for next iteration
        self.last_correction = correction
        
        return self.constrain(correction, -self.max_correction, self.max_correction)
    
    def rotate_right(self) -> Twist:
        """Rotate right to avoid front obstacle"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.base_speed / 0.080  # Convert to angular velocity
        
        # Reset PID state when rotating
        self.reset_pid_state()
        
        return twist
    
    def reset_pid_state(self):
        """Reset PID controller state"""
        self.last_error = 0.0
        self.last_last_error = 0.0
        self.cumulative_error = 0.0
        self.last_correction = 0.0
        
        self.get_logger().debug('PID state reset')
    
    def set_pid_gains(self, kp: float, kd: float, ki: float):
        """Dynamically adjust PID gains"""
        self.kp = kp
        self.kd = kd
        self.ki = ki
        
        self.get_logger().info(f'PID gains updated - Kp: {kp}, Kd: {kd}, Ki: {ki}')
    
    def set_desired_distance(self, distance: float):
        """Dynamically adjust desired wall distance"""
        self.desired_distance = distance
        self.get_logger().info(f'Desired distance updated to {distance}m')

def main(args=None):
    """Main function to run the PID-based wall follower"""
    rclpy.init(args=args)
    
    try:
        wall_follower = WallFollowerPID()
        
        # Log startup information
        wall_follower.get_logger().info('PID-based wall follower started')
        wall_follower.get_logger().info('Algorithm: PID control for lateral distance')
        wall_follower.get_logger().info('Behavior: Maintain constant distance from left wall')
        
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