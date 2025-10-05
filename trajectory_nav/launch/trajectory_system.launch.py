#!/usr/bin/env python3
"""
Trajectory Navigation System Launch File

Launches the three core trajectory nodes and optional RViz visualization.
NOTE: Launch Gazebo separately before running this launch file.

Usage:
    # First, launch Gazebo in separate terminal:
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo empty_world.launch.py
    
    # Then launch trajectory system:
    ros2 launch trajectory_nav trajectory_system.launch.py config:=straight
    ros2 launch trajectory_nav trajectory_system.launch.py config:=circle
    ros2 launch trajectory_nav trajectory_system.launch.py config:=curve
    ros2 launch trajectory_nav trajectory_system.launch.py config:=straight use_rviz:=false

Architecture:
    1. Trajectory Generator - Publishes smooth path from waypoints (starts immediately)
    2. Trajectory Controller - Pure pursuit control (starts after 3s delay)
    3. Trajectory Monitor - Tracks performance metrics (starts after 3s delay)
    4. RViz2 - Visualization (optional, starts immediately)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for trajectory navigation nodes."""
    
    # Get package directory
    trajectory_nav_dir = get_package_share_directory('trajectory_nav')
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='straight',
        description='Trajectory configuration: straight, circle, or curve'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for visualization'
    )
    
    # Get launch configurations
    config_name = LaunchConfiguration('config')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Construct path to config file
    config_file = PathJoinSubstitution([
        trajectory_nav_dir,
        'config',
        ['trajectory_', config_name, '.yaml']
    ])
    
    # Path to RViz config
    rviz_config_file = os.path.join(trajectory_nav_dir, 'rviz', 'trajectory_config.rviz')
    
    # 1. Trajectory Generator Node (starts immediately)
    generator_node = Node(
        package='trajectory_nav',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': True}
        ]
    )
    
    # 2. Trajectory Controller Node (3s delay - ensure trajectory is published)
    controller_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='trajectory_nav',
                executable='trajectory_controller',
                name='trajectory_controller',
                output='screen',
                parameters=[
                    config_file,
                    {'use_sim_time': True}
                ]
            )
        ]
    )
    
    # 3. Trajectory Monitor Node (3s delay - simultaneous with controller)
    monitor_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='trajectory_nav',
                executable='trajectory_monitor',
                name='trajectory_monitor',
                output='screen',
                parameters=[
                    config_file,
                    {'use_sim_time': True}
                ]
            )
        ]
    )
    
    # 4. RViz2 (optional, starts immediately)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )
    
    # Build and return launch description
    return LaunchDescription([
        # Launch arguments
        config_arg,
        use_rviz_arg,
        
        # Nodes (in order)
        generator_node,      # Starts immediately
        controller_node,     # Starts at 3s
        monitor_node,        # Starts at 3s
        rviz_node           # Starts immediately (if enabled)
    ])



