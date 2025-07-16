#!/usr/bin/env python3

"""
Machine Learning-based Wall Following Algorithm for TurtleBot3
Ported from Webots simulation to real hardware ROS2 node
"""

import rclpy
from geometry_msgs.msg import Twist
from .wall_follower_base import WallFollowerBase
import math
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class WallFollowerML(WallFollowerBase):
    """Machine Learning-based wall following algorithm using trained models"""
    
    def __init__(self):
        super().__init__('wall_follower_ml')
        
        # ML parameters
        self.model_type = 'neural_network'  # 'decision_tree' or 'neural_network'
        self.model = None
        self.model_trained = False
        
        # Control parameters
        self.front_threshold = 0.25  # Front obstacle threshold (meters)
        self.max_speed = 0.2        # Maximum speed (m/s)
        
        # Data processing parameters
        self.n_inputs = 3   # front, left, corner measurements
        self.n_outputs = 2  # left_speed, right_speed
        
        # Initialize and train model
        self.initialize_model()
        
        self.get_logger().info('Machine Learning-based wall follower initialized')
        self.get_logger().info(f'Model type: {self.model_type}')
        self.get_logger().info(f'Model trained: {self.model_trained}')
    
    def initialize_model(self):
        """Initialize and train the machine learning model"""
        try:
            # Import ML libraries
            from sklearn.tree import DecisionTreeRegressor
            from sklearn.neural_network import MLPRegressor
            
            # Load training data
            data = self.load_training_data()
            if data is None:
                self.get_logger().error('Failed to load training data')
                return
            
            # Prepare data
            X, y = self.prepare_data(data)
            
            # Create and train model
            if self.model_type == 'decision_tree':
                self.model = DecisionTreeRegressor(random_state=42)
            else:  # neural_network
                self.model = MLPRegressor(
                    hidden_layer_sizes=(100, 50),
                    max_iter=1000,
                    random_state=42,
                    early_stopping=True,
                    validation_fraction=0.1
                )
            
            # Train model
            self.get_logger().info('Training machine learning model...')
            self.model.fit(X, y)
            self.model_trained = True
            
            self.get_logger().info('Model training completed successfully')
            
            # Log training statistics
            if hasattr(self.model, 'score'):
                score = self.model.score(X, y)
                self.get_logger().info(f'Training score: {score:.4f}')
                
        except ImportError as e:
            self.get_logger().error(f'Failed to import ML libraries: {e}')
            self.get_logger().error('Please install scikit-learn: pip install scikit-learn')
        except Exception as e:
            self.get_logger().error(f'Error initializing model: {e}')
    
    def load_training_data(self):
        """Load training data from CSV file"""
        try:
            package_share_directory = get_package_share_directory('webots_ros2_turtlebot')
            data_path = os.path.join(package_share_directory, 'resource', 'data.csv')
            
            # Alternative path for development
            if not os.path.exists(data_path):
                current_dir = os.path.dirname(os.path.abspath(__file__))
                data_path = os.path.join(current_dir, '..', 'resource', 'data.csv')
            
            if not os.path.exists(data_path):
                self.get_logger().error(f'Training data file not found: {data_path}')
                return None
            
            # Load data using numpy
            data = np.genfromtxt(data_path, delimiter=',')
            
            self.get_logger().info(f'Loaded training data: {data.shape[0]} samples, {data.shape[1]} features')
            return data
            
        except Exception as e:
            self.get_logger().error(f'Error loading training data: {e}')
            return None
    
    def prepare_data(self, data):
        """Prepare and clean training data"""
        try:
            # Filter out NaN and infinite values
            n_rows, n_cols = data.shape
            counter_nan = 0
            counter_inf = 0
            
            for j in range(n_cols):
                for i in range(n_rows):
                    if math.isnan(data[i, j]):
                        data[i, j] = 0
                        counter_nan += 1
                    if math.isinf(data[i, j]):
                        data[i, j] = 0
                        counter_inf += 1
            
            self.get_logger().info(f'Filtered {counter_nan} NaN values and {counter_inf} infinite values')
            
            # Split inputs and outputs
            X = data[:, :self.n_inputs]  # First 3 columns: front, left, corner
            y = data[:, self.n_inputs:]  # Last 2 columns: left_speed, right_speed
            
            # Normalize inputs (optional, can improve training)
            X_max = np.max(X, axis=0)
            X_max[X_max == 0] = 1  # Avoid division by zero
            X = X / X_max
            self.input_normalization = X_max
            
            return X, y
            
        except Exception as e:
            self.get_logger().error(f'Error preparing data: {e}')
            return None, None
    
    def compute_control_command(self) -> Twist:
        """
        Implement ML-based wall following logic
        """
        if not self.model_trained:
            self.get_logger().warn('Model not trained, stopping robot')
            return Twist()  # Stop robot
        
        # Get sensor readings (same as in training data)
        front_dist = self.get_range_at_angle(0)    # Front (0°)
        left_dist = self.get_range_at_angle(90)    # Left (90°)
        corner_dist = self.get_range_at_angle(45)  # Corner (45°)
        
        # Check for front obstacle first (safety priority)
        if front_dist < self.front_threshold:
            self.get_logger().info(f'Front obstacle detected at {front_dist:.2f}m - rotating right')
            return self.rotate_right()
        
        # Prepare input for ML model
        return self.ml_control(front_dist, left_dist, corner_dist)
    
    def ml_control(self, front_dist: float, left_dist: float, corner_dist: float) -> Twist:
        """
        Apply ML model to predict control commands
        """
        try:
            # Constrain input values to reasonable range
            front_dist = self.constrain(front_dist, 0.0, 10.0)
            left_dist = self.constrain(left_dist, 0.0, 10.0)
            corner_dist = self.constrain(corner_dist, 0.0, 10.0)
            
            # Normalize inputs (if normalization was used during training)
            if hasattr(self, 'input_normalization'):
                input_data = np.array([[front_dist, left_dist, corner_dist]]) / self.input_normalization
            else:
                input_data = np.array([[front_dist, left_dist, corner_dist]])
            
            # Get prediction from model
            prediction = self.model.predict(input_data)
            
            # Extract predicted speeds
            left_speed = prediction[0][0]
            right_speed = prediction[0][1]
            
            # Apply speed limits
            left_speed = self.constrain(left_speed, -self.max_speed, self.max_speed)
            right_speed = self.constrain(right_speed, -self.max_speed, self.max_speed)
            
            # Convert to Twist message
            twist = self.wheel_speeds_to_twist(
                left_speed / self.wheel_radius,   # Convert to rad/s
                right_speed / self.wheel_radius
            )
            
            # Log control information
            self.get_logger().info(
                f'ML Control - Front: {front_dist:.3f}m, Left: {left_dist:.3f}m, '
                f'Corner: {corner_dist:.3f}m, Speeds: L={left_speed:.3f}, R={right_speed:.3f}'
            )
            
            return twist
            
        except Exception as e:
            self.get_logger().error(f'Error in ML control: {e}')
            return Twist()  # Stop robot on error
    
    def rotate_right(self) -> Twist:
        """Rotate right to avoid front obstacle"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5  # Rotate right
        
        return twist
    
    def set_model_type(self, model_type: str):
        """Change model type and retrain"""
        if model_type in ['decision_tree', 'neural_network']:
            self.model_type = model_type
            self.get_logger().info(f'Model type changed to: {model_type}')
            self.initialize_model()
        else:
            self.get_logger().error(f'Invalid model type: {model_type}')

def main(args=None):
    """Main function to run the ML-based wall follower"""
    rclpy.init(args=args)
    
    try:
        wall_follower = WallFollowerML()
        
        # Log startup information
        wall_follower.get_logger().info('ML-based wall follower started')
        wall_follower.get_logger().info('Algorithm: Machine Learning (Decision Tree / Neural Network)')
        wall_follower.get_logger().info('Behavior: Learned from PID controller data')
        
        if not wall_follower.model_trained:
            wall_follower.get_logger().error('Model training failed - robot will not move')
        
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