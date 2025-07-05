#!/usr/bin/env python3

"""Launch TurtleBot3 on real hardware with auto-start capability."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_nav = LaunchConfiguration('nav', default='false')
    use_slam = LaunchConfiguration('slam', default='false')
    auto_mode = LaunchConfiguration('auto_mode', default='true')
    
    # TurtleBot3 bringup (real hardware)
    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turtlebot3_bringup'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Auto navigation node (custom behavior)
    auto_navigator = Node(
        package='webots_ros2_turtlebot',
        executable='auto_navigator',
        name='auto_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(auto_mode)
    )
    
    # Navigation stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_nav)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'nav',
            default_value='false',
            description='Enable navigation'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Enable SLAM'
        ),
        DeclareLaunchArgument(
            'auto_mode',
            default_value='true',
            description='Enable autonomous behavior'
        ),
        
        turtlebot3_bringup,
        auto_navigator,
        nav2_bringup,
    ]) 