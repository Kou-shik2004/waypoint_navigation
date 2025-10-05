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
        self.tracking_active = False
        self.tracking_completed = False  # Set to True when goal reached, prevents restart
        self.robot_stopped_count = 0
        
        # Metrics storage
        self.error_history = []
        self.time_history = []
        self.position_history = []
        
        # Statistics
        self.max_error = 0.0
        self.error_sum_squared = 0.0
        self.error_count = 0
        self.distance_traveled = 0.0
        self.last_position = None
        
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
        # If we already completed a trajectory, ignore all further messages
        if self.tracking_completed:
            # Silently ignore - generator republishes every 1s for late joiners
            return
        
        # Check if this is a new trajectory (not a republish)
        is_new_trajectory = (
            self.trajectory is None or
            len(msg.poses) != len(self.trajectory.poses)
        )
        
        if not is_new_trajectory:
            # Ignore republished trajectories (generator publishes every 1s for late joiners)
            return
        
        self.trajectory = msg
        
        # Extract positions as numpy array [N x 2]
        self.trajectory_points = np.array([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ])
        
        self.get_logger().info(
            f'Received NEW trajectory with {len(self.trajectory_points)} points - Starting tracking'
        )
        
        # Reset metrics for new trajectory
        self.error_history.clear()
        self.time_history.clear()
        self.position_history.clear()
        self.max_error = 0.0
        self.error_sum_squared = 0.0
        self.error_count = 0
        self.distance_traveled = 0.0
        self.last_position = None
        self.start_time = None
        self.tracking_active = True
        self.robot_stopped_count = 0
    
    def _odom_callback(self, msg: Odometry):
        """Store current robot pose."""
        self.current_pose = msg.pose.pose.position
        
        # Set start time on first odometry message
        if self.start_time is None:
            self.start_time = self.get_clock().now()
    
    def _monitor_loop(self):
        """Main monitoring loop running at 10Hz."""
        # Don't log if tracking is complete
        if not self.tracking_active:
            return
        
        # Check preconditions with diagnostic logging
        if self.trajectory is None or self.trajectory_points is None:
            # Log once every 50 iterations (every 5 seconds) when waiting for trajectory
            if not hasattr(self, '_waiting_traj_logged') or self._loop_count % 50 == 0:
                if self.trajectory is None:
                    self.get_logger().warn('Waiting for trajectory...', throttle_duration_sec=5.0)
                self._waiting_traj_logged = True
            self._loop_count = getattr(self, '_loop_count', 0) + 1
            return
        
        if self.current_pose is None or self.start_time is None:
            # Log once when waiting for odometry
            if not hasattr(self, '_waiting_odom_logged'):
                self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=5.0)
                self._waiting_odom_logged = True
            return
        
        if len(self.trajectory_points) == 0:
            self.get_logger().error('Trajectory has 0 points!')
            return
        
        try:
            # Compute cross-track error (minimum distance to trajectory)
            error = self._compute_cross_track_error()
            
            # Record metrics
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            
            # Update distance traveled and check if robot stopped
            current_pos = np.array([self.current_pose.x, self.current_pose.y])
            if self.last_position is not None:
                delta = current_pos - self.last_position
                distance_increment = np.linalg.norm(delta)
                self.distance_traveled += distance_increment
                
                # Detect if robot has stopped (moved less than 0.001m in 0.1s = 0.01 m/s)
                if distance_increment < 0.001:
                    self.robot_stopped_count += 1
                else:
                    self.robot_stopped_count = 0
                
                # If robot stopped for 30 consecutive iterations (3 seconds), stop tracking
                if self.robot_stopped_count >= 30:
                    self.tracking_active = False
                    self.tracking_completed = True  # Prevent restart
                    self.get_logger().info(
                        f'[GOAL REACHED] Robot stopped. Finalizing metrics...'
                    )
                    self._print_final_summary()
                    return
            
            self.last_position = current_pos.copy()
            
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
        if self.error_count == 0 or not self.tracking_active:
            return
        
        rms_error = math.sqrt(self.error_sum_squared / self.error_count)
        mean_error = sum(self.error_history) / len(self.error_history)
        
        self.get_logger().info(
            f'Tracking stats: RMS={rms_error:.4f}m, '
            f'Max={self.max_error:.4f}m, '
            f'Mean={mean_error:.4f}m, '
            f'Samples={self.error_count}'
        )
    
    def _print_final_summary(self):
        """Print formatted final statistics to terminal."""
        if self.error_count == 0:
            self.get_logger().warn('No tracking data collected')
            return
        
        rms_error = math.sqrt(self.error_sum_squared / self.error_count)
        mean_error = sum(self.error_history) / len(self.error_history)
        total_time = self.time_history[-1] if self.time_history else 0.0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('FINAL TRACKING METRICS')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'RMS Error:        {rms_error:.6f} m')
        self.get_logger().info(f'Max Error:        {self.max_error:.6f} m')
        self.get_logger().info(f'Mean Error:       {mean_error:.6f} m')
        self.get_logger().info(f'Distance:         {self.distance_traveled:.6f} m')
        self.get_logger().info(f'Total Time:       {total_time:.2f} s')
        self.get_logger().info(f'Total Samples:    {self.error_count}')
        self.get_logger().info('=' * 60)
    
    def export_csv(self):
        """Export metrics to CSV file on shutdown."""
        if len(self.error_history) == 0:
            self.get_logger().warn('No data to export - trajectory not completed')
            return
        
        try:
            # Compute final statistics
            rms_error = math.sqrt(self.error_sum_squared / self.error_count)
            mean_error = sum(self.error_history) / len(self.error_history)
            total_time = self.time_history[-1] if self.time_history else 0.0
            
            with open(self.csv_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write summary statistics at the top
                writer.writerow(['# TRAJECTORY TRACKING SUMMARY'])
                writer.writerow(['Metric', 'Value', 'Unit'])
                writer.writerow(['RMS Error', f'{rms_error:.6f}', 'm'])
                writer.writerow(['Max Error', f'{self.max_error:.6f}', 'm'])
                writer.writerow(['Mean Error', f'{mean_error:.6f}', 'm'])
                writer.writerow(['Distance Traveled', f'{self.distance_traveled:.6f}', 'm'])
                writer.writerow(['Total Time', f'{total_time:.2f}', 's'])
                writer.writerow(['Total Samples', f'{self.error_count}', 'count'])
                writer.writerow([])  # Blank line separator
                
                # Write time-series data header
                writer.writerow(['# TIME-SERIES DATA'])
                writer.writerow(['time', 'x', 'y', 'error'])
                
                # Write time-series data
                for i in range(len(self.error_history)):
                    writer.writerow([
                        f'{self.time_history[i]:.3f}',
                        f'{self.position_history[i][0]:.6f}',
                        f'{self.position_history[i][1]:.6f}',
                        f'{self.error_history[i]:.6f}'
                    ])
            
            # Print final statistics to terminal (if not already printed)
            if self.tracking_active:
                self._print_final_summary()
            
            self.get_logger().info(f'CSV exported: {self.csv_filename}')
            
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


