#!/usr/bin/env python3
"""
Trajectory Generator Node for ROS2 Navigation System

Generates smooth trajectories from discrete waypoints using cubic spline
interpolation with arc-length parameterization. Publishes to /trajectory
topic with TRANSIENT_LOCAL QoS for late-joiner support.

Author: Student
License: MIT
"""

import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import cumulative_trapezoid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class TrajectoryGenerator(Node):
    """
    Generates smooth trajectories from waypoints using cubic spline interpolation.
    
    Publishes:
        /trajectory (nav_msgs/Path): Smooth interpolated path
        /trajectory_markers (visualization_msgs/MarkerArray): Visualization markers
    """
    
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # 1. Declare parameters with explicit types
        # Note: Empty list [] defaults to BYTE_ARRAY, so we use a double array default
        self.declare_parameter(
            'waypoints', 
            [0.0],  # Typed default - will be overridden by YAML
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='Waypoints as flat list [x0,y0,x1,y1,...]'
            )
        )
        self.declare_parameter('num_samples', 200)
        self.declare_parameter('desired_velocity', 0.20)
        self.declare_parameter('frame_id', 'odom')
        
        # 2. Retrieve and validate parameters
        waypoints_flat = self.get_parameter('waypoints').value
        self.num_samples = self.get_parameter('num_samples').value
        self.desired_velocity = self.get_parameter('desired_velocity').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Validate waypoints
        if not waypoints_flat or len(waypoints_flat) < 4:
            self.get_logger().error(
                f'Insufficient waypoints: need at least 4 values (2 points), got {len(waypoints_flat)}'
            )
            raise ValueError('Insufficient waypoints for trajectory generation')
        
        if len(waypoints_flat) % 2 != 0:
            self.get_logger().error(f'Waypoints must be pairs of (x,y): got {len(waypoints_flat)} values')
            raise ValueError('Waypoints must have even number of values')
        
        # Convert flat list to numpy array
        self.waypoints = np.array(waypoints_flat).reshape(-1, 2)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
        # Validate other parameters
        if self.num_samples < 50:
            self.get_logger().warn(f'num_samples={self.num_samples} is low, increasing to 50')
            self.num_samples = 50
        
        if self.desired_velocity <= 0 or self.desired_velocity > 0.22:
            self.get_logger().error(
                f'Invalid desired_velocity={self.desired_velocity} (must be 0 < v <= 0.22 m/s)'
            )
            raise ValueError('Invalid desired_velocity parameter')
        
        # 3. Create QoS profile - TRANSIENT_LOCAL for late-joiner support
        trajectory_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # 4. Create publishers
        self.trajectory_pub = self.create_publisher(
            Path, 
            '/trajectory', 
            trajectory_qos
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/trajectory_markers',
            10
        )
        
        # 5. Generate trajectory
        self.trajectory = self._generate_smooth_trajectory()
        
        # 6. Create timer to republish (for late joiners)
        self.republish_timer = self.create_timer(1.0, self._republish_trajectory)
        
        # 7. Log initialization success
        self.get_logger().info(
            f'Trajectory generator initialized: {len(self.trajectory.poses)} poses, '
            f'velocity={self.desired_velocity} m/s'
        )
        
        # Publish immediately
        self._republish_trajectory()
    
    def _generate_smooth_trajectory(self) -> Path:
        """
        Generate smooth trajectory using cubic spline with arc-length parameterization.
        
        Returns:
            Path: ROS2 Path message with interpolated poses
        """
        try:
            # Extract x and y coordinates
            x_waypoints = self.waypoints[:, 0]
            y_waypoints = self.waypoints[:, 1]
            
            # Create parameter t for waypoints (0 to 1)
            t_waypoints = np.linspace(0, 1, len(self.waypoints))
            
            # Create cubic splines for x(t) and y(t)
            spline_x = CubicSpline(t_waypoints, x_waypoints)
            spline_y = CubicSpline(t_waypoints, y_waypoints)
            
            # Sample spline at high resolution for arc-length computation
            t_fine = np.linspace(0, 1, 1000)
            x_fine = spline_x(t_fine)
            y_fine = spline_y(t_fine)
            
            # Compute derivatives for arc-length
            dx_dt = spline_x.derivative()(t_fine)
            dy_dt = spline_y.derivative()(t_fine)
            
            # Arc-length element: ds = sqrt(dx^2 + dy^2) dt
            ds_dt = np.sqrt(dx_dt**2 + dy_dt**2)
            
            # Cumulative arc-length
            s_fine = np.concatenate(([0], cumulative_trapezoid(ds_dt, t_fine)))
            total_length = s_fine[-1]
            
            # Create uniform arc-length samples
            s_uniform = np.linspace(0, total_length, self.num_samples)
            
            # Interpolate t as function of s, then compute x(s) and y(s)
            t_uniform = np.interp(s_uniform, s_fine, t_fine)
            x_uniform = spline_x(t_uniform)
            y_uniform = spline_y(t_uniform)
            
            # Compute heading angle at each point (tangent direction)
            dx = np.gradient(x_uniform)
            dy = np.gradient(y_uniform)
            headings = np.arctan2(dy, dx)
            
            # Create Path message
            path = Path()
            path.header.frame_id = self.frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            
            for i in range(len(x_uniform)):
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.header.stamp = path.header.stamp
                
                pose.pose.position.x = float(x_uniform[i])
                pose.pose.position.y = float(y_uniform[i])
                pose.pose.position.z = 0.0
                
                # Convert yaw to quaternion (2D rotation around z-axis)
                yaw = headings[i]
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                path.poses.append(pose)
            
            self.get_logger().info(
                f'Generated trajectory: {len(path.poses)} poses, '
                f'length={total_length:.2f}m, '
                f'estimated time={total_length/self.desired_velocity:.1f}s'
            )
            
            return path
            
        except Exception as e:
            self.get_logger().error(f'Trajectory generation failed: {e}')
            raise
    
    def _republish_trajectory(self):
        """Republish trajectory and markers (for late joiners)."""
        # Update timestamp
        self.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Publish trajectory
        self.trajectory_pub.publish(self.trajectory)
        
        # Publish visualization markers
        markers = self._create_visualization_markers()
        self.marker_pub.publish(markers)
    
    def _create_visualization_markers(self) -> MarkerArray:
        """
        Create RViz visualization markers for waypoints and trajectory.
        
        Returns:
            MarkerArray: Markers for waypoints (red spheres) and path (green line)
        """
        marker_array = MarkerArray()
        
        # Marker 1: Waypoints as red spheres
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = self.frame_id
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = 'waypoints'
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.1
        waypoint_marker.scale.y = 0.1
        waypoint_marker.scale.z = 0.1
        waypoint_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        for wp in self.waypoints:
            point = PoseStamped().pose.position
            point.x = float(wp[0])
            point.y = float(wp[1])
            point.z = 0.0
            waypoint_marker.points.append(point)
        
        marker_array.markers.append(waypoint_marker)
        
        # Marker 2: Smooth path as green line strip
        path_marker = Marker()
        path_marker.header.frame_id = self.frame_id
        path_marker.header.stamp = waypoint_marker.header.stamp
        path_marker.ns = 'smooth_path'
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.02  # Line width
        path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        for pose in self.trajectory.poses:
            path_marker.points.append(pose.pose.position)
        
        marker_array.markers.append(path_marker)
        
        return marker_array


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TrajectoryGenerator()
        rclpy.spin(node)
    except Exception as e:
        print(f'Trajectory generator error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


