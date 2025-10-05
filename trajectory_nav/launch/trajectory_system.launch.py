#!/usr/bin/env python3
"""
Trajectory System Launch File

Orchestrates the complete trajectory navigation system:
1. Gazebo simulation (0s delay)
2. Trajectory generator (2s delay - wait for Gazebo initialization)
3. Controller and monitor (3s delay - ensure trajectory is published)
4. Optional RViz visualization

Usage:
    ros2 launch trajectory_nav trajectory_system.launch.py config:=straight
    ros2 launch trajectory_nav trajectory_system.launch.py config:=circle
    ros2 launch trajectory_nav trajectory_system.launch.py config:=curve
    ros2 launch trajectory_nav trajectory_system.launch.py config:=straight use_rviz:=false

Author: Student
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with trajectory navigation system."""
    
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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )
    
    # Get launch configurations
    config_name = LaunchConfiguration('config')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    trajectory_nav_dir = get_package_share_directory('trajectory_nav')
    
    # Construct path to config file
    config_file = PathJoinSubstitution([
        trajectory_nav_dir,
        'config',
        ['trajectory_', config_name, '.yaml']
    ])
    
    # 1. Launch Gazebo with TurtleBot3 (0s delay)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 2. Launch trajectory generator (2s delay - wait for Gazebo)
    generator_node = Node(
        package='trajectory_nav',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True
    )
    
    generator_delayed = TimerAction(
        period=2.0,
        actions=[generator_node]
    )
    
    # 3. Launch trajectory controller (3s delay - ensure trajectory published)
    controller_node = Node(
        package='trajectory_nav',
        executable='trajectory_controller',
        name='trajectory_controller',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True
    )
    
    controller_delayed = TimerAction(
        period=3.0,
        actions=[controller_node]
    )
    
    # 4. Launch trajectory monitor (3s delay - simultaneous with controller)
    monitor_node = Node(
        package='trajectory_nav',
        executable='trajectory_monitor',
        name='trajectory_monitor',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True
    )
    
    monitor_delayed = TimerAction(
        period=3.0,
        actions=[monitor_node]
    )
    
    # 5. Optional RViz launch (3s delay)
    rviz_config_file = os.path.join(trajectory_nav_dir, 'rviz', 'trajectory_config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=launch.conditions.IfCondition(use_rviz)
    )
    
    rviz_delayed = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(config_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_sim_time_arg)
    
    # Add nodes in sequence
    ld.add_action(gazebo_launch)           # 0s
    ld.add_action(generator_delayed)       # 2s
    ld.add_action(controller_delayed)      # 3s
    ld.add_action(monitor_delayed)         # 3s
    ld.add_action(rviz_delayed)            # 3s (if enabled)
    
    return ld


# Note: This launch file requires proper imports
# The condition import needs to be added at the top



