#!/usr/bin/env python3
"""
Trajectory Monitor Node for ROS2 Navigation System

Monitors trajectory tracking performance by computing cross-track error
(minimum distance from robot to reference trajectory). Logs statistics
and exports time-series data to CSV on shutdown.

Author: Student
License: MIT
"""

import math
import csv
from datetime import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64


class TrajectoryMonitor(Node):
    """
    Monitors trajectory tracking performance and exports metrics.
    
    Subscribes:
        /trajectory (nav_msgs/Path): Reference trajectory
        /odom (nav_msgs/Odometry): Robot odometry
    
    Publishes:
        /tracking_error (std_msgs/Float64): Real-time tracking error
    """
    
    def __init__(self):
        super().__init__('trajectory_monitor')
        
        # 1. Declare parameters
        self.declare_parameter('csv_output_path', 'trajectory_metrics')
        self.declare_parameter('use_sim_time', True)
        
        # 2. Retrieve parameters
        csv_base = self.get_parameter('csv_output_path').value
        
        # Append timestamp to CSV filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'{csv_base}_{timestamp}.csv'
        
        # State variables
        self.trajectory = None
        self.trajectory_points = None  # Numpy array for efficient computation
        self.current_pose = None
        self.start_time = None
        
        # Metrics storage
        self.error_history = []
        self.time_history = []
        self.position_history = []
        
        # Statistics
        self.max_error = 0.0
        self.error_sum_squared = 0.0
        self.error_count = 0
        
        # 3. Create QoS profiles
        trajectory_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # 4. Create subscribers
        self.trajectory_sub = self.create_subscription(
            Path,
            '/trajectory',
            self._trajectory_callback,
            trajectory_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        
        # 5. Create publisher
        self.error_pub = self.create_publisher(Float64, '/tracking_error', 10)
        
        # 6. Create timer for monitoring at 10Hz
        self.monitor_timer = self.create_timer(0.1, self._monitor_loop)
        
        # Timer for periodic statistics logging (every 5 seconds)
        self.stats_timer = self.create_timer(5.0, self._log_statistics)
        
        # 7. Log initialization
        self.get_logger().info(
            f'Monitor initialized: CSV output will be saved to {self.csv_filename}'
        )
    
    def _trajectory_callback(self, msg: Path):
        """Store trajectory and convert to numpy array for efficient computation."""
        self.trajectory = msg
        
        # Extract positions as numpy array [N x 2]
        self.trajectory_points = np.array([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ])
        
        self.get_logger().info(
            f'Received trajectory with {len(self.trajectory_points)} points'
        )
        
        # Reset metrics for new trajectory
        self.error_history.clear()
        self.time_history.clear()
        self.position_history.clear()
        self.max_error = 0.0
        self.error_sum_squared = 0.0
        self.error_count = 0
        self.start_time = None
    
    def _odom_callback(self, msg: Odometry):
        """Store current robot pose."""
        self.current_pose = msg.pose.pose.position
        
        # Set start time on first odometry message
        if self.start_time is None:
            self.start_time = self.get_clock().now()
    
    def _monitor_loop(self):
        """Main monitoring loop running at 10Hz."""
        # Check preconditions
        if self.trajectory is None or self.trajectory_points is None:
            return
        
        if self.current_pose is None or self.start_time is None:
            return
        
        if len(self.trajectory_points) == 0:
            return
        
        try:
            # Compute cross-track error (minimum distance to trajectory)
            error = self._compute_cross_track_error()
            
            # Record metrics
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            
            self.error_history.append(error)
            self.time_history.append(elapsed_time)
            self.position_history.append([self.current_pose.x, self.current_pose.y])
            
            # Update statistics
            self.error_sum_squared += error * error
            self.error_count += 1
            if error > self.max_error:
                self.max_error = error
            
            # Publish tracking error
            error_msg = Float64()
            error_msg.data = error
            self.error_pub.publish(error_msg)
            
        except Exception as e:
            self.get_logger().error(f'Monitor loop error: {e}')
    
    def _compute_cross_track_error(self):
        """
        Compute cross-track error: minimum distance from robot to trajectory.
        
        Returns:
            float: Cross-track error in meters
        """
        # Current robot position
        robot_pos = np.array([self.current_pose.x, self.current_pose.y])
        
        # Compute distances to all trajectory points
        # Using vectorized numpy operations for efficiency
        deltas = self.trajectory_points - robot_pos
        distances = np.sqrt(np.sum(deltas**2, axis=1))
        
        # Minimum distance is cross-track error
        min_distance = np.min(distances)
        
        return float(min_distance)
    
    def _log_statistics(self):
        """Log current statistics every 5 seconds."""
        if self.error_count == 0:
            return
        
        rms_error = math.sqrt(self.error_sum_squared / self.error_count)
        mean_error = sum(self.error_history) / len(self.error_history)
        
        self.get_logger().info(
            f'Tracking stats: RMS={rms_error:.4f}m, '
            f'Max={self.max_error:.4f}m, '
            f'Mean={mean_error:.4f}m, '
            f'Samples={self.error_count}'
        )
    
    def export_csv(self):
        """Export metrics to CSV file on shutdown."""
        if len(self.error_history) == 0:
            self.get_logger().warn('No data to export')
            return
        
        try:
            with open(self.csv_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write header
                writer.writerow(['time', 'x', 'y', 'error'])
                
                # Write data
                for i in range(len(self.error_history)):
                    writer.writerow([
                        f'{self.time_history[i]:.3f}',
                        f'{self.position_history[i][0]:.6f}',
                        f'{self.position_history[i][1]:.6f}',
                        f'{self.error_history[i]:.6f}'
                    ])
            
            # Compute final statistics
            rms_error = math.sqrt(self.error_sum_squared / self.error_count)
            mean_error = sum(self.error_history) / len(self.error_history)
            total_time = self.time_history[-1] if self.time_history else 0.0
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('FINAL TRACKING METRICS')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'RMS Error:        {rms_error:.6f} m')
            self.get_logger().info(f'Max Error:        {self.max_error:.6f} m')
            self.get_logger().info(f'Mean Error:       {mean_error:.6f} m')
            self.get_logger().info(f'Total Time:       {total_time:.2f} s')
            self.get_logger().info(f'Total Samples:    {self.error_count}')
            self.get_logger().info(f'CSV File:         {self.csv_filename}')
            self.get_logger().info('=' * 60)
            
        except Exception as e:
            self.get_logger().error(f'Failed to export CSV: {e}')
    
    def shutdown(self):
        """Clean shutdown with CSV export."""
        self.get_logger().info('Shutting down, exporting metrics...')
        self.export_csv()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = TrajectoryMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt detected')
    except Exception as e:
        print(f'Monitor error: {e}')
    finally:
        # Ensure CSV is exported on shutdown
        if node:
            node.shutdown()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


