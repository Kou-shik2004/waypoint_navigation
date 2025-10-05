#!/usr/bin/env python3
"""
Trajectory Controller Node for ROS2 Navigation System

Implements pure pursuit control algorithm for differential drive robot.
Tracks reference trajectory using geometric control law with lookahead distance.

Control Law: ω = 2v·sin(α)/L
where α is heading error, L is lookahead distance, v is desired velocity.

Author: Student
License: MIT
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


class TrajectoryController(Node):
    """
    Pure pursuit controller for trajectory tracking.
    
    Subscribes:
        /trajectory (nav_msgs/Path): Reference trajectory to follow
        /odom (nav_msgs/Odometry): Robot odometry feedback
    
    Publishes:
        /cmd_vel (geometry_msgs/Twist): Velocity commands
    """
    
    # TurtleBot3 Burger physical limits (HARD CONSTRAINTS)
    MAX_LINEAR_VELOCITY = 0.22  # m/s
    MAX_ANGULAR_VELOCITY = 2.84  # rad/s
    
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # 1. Declare parameters with defaults
        self.declare_parameter('lookahead_distance', 0.30)
        self.declare_parameter('goal_tolerance', 0.05)
        
        # 2. Retrieve and validate parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        if self.lookahead_distance <= 0:
            self.get_logger().error(f'Invalid lookahead_distance={self.lookahead_distance}')
            raise ValueError('lookahead_distance must be positive')
        
        if self.goal_tolerance <= 0:
            self.get_logger().error(f'Invalid goal_tolerance={self.goal_tolerance}')
            raise ValueError('goal_tolerance must be positive')
        
        # State variables
        self.trajectory = None
        self.current_pose = None
        self.current_yaw = None
        self.desired_velocity = 0.20  # Will be overridden from trajectory
        self.target_index = 0
        self.goal_reached = False
        
        # Debug logging counter
        self.loop_counter = 0
        
        # 3. Create QoS profiles
        # Trajectory uses TRANSIENT_LOCAL (must match publisher)
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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 6. Create timer for control loop at EXACTLY 20Hz
        self.control_timer = self.create_timer(0.05, self._control_loop)
        
        # 7. Log initialization
        self.get_logger().info(
            f'Controller initialized: lookahead={self.lookahead_distance}m, '
            f'goal_tolerance={self.goal_tolerance}m, control_rate=20Hz'
        )
    
    def _trajectory_callback(self, msg: Path):
        """
        Store received trajectory.
        Only reset target_index for NEW trajectories (different poses).
        Generator republishes same trajectory every 1s for late-joiners (TRANSIENT_LOCAL QoS).
        """
        # Check if this is a new trajectory or just a republish
        is_new_trajectory = (
            self.trajectory is None or
            len(msg.poses) != len(self.trajectory.poses) or
            (len(msg.poses) > 0 and 
             (msg.poses[0].pose.position.x != self.trajectory.poses[0].pose.position.x or
              msg.poses[0].pose.position.y != self.trajectory.poses[0].pose.position.y))
        )
        
        self.trajectory = msg
        
        # Only reset tracking state for genuinely new trajectories
        if is_new_trajectory:
            self.target_index = 0
            self.goal_reached = False
            self.get_logger().info(f'Received NEW trajectory with {len(msg.poses)} poses')
        else:
            # Just a republish of the same trajectory - don't reset progress
            pass  # Silently ignore republishes
    
    def _odom_callback(self, msg: Odometry):
        """Store current robot pose and extract yaw from quaternion."""
        self.current_pose = msg.pose.pose.position
        
        # Extract yaw from quaternion (2D rotation around z-axis)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def _control_loop(self):
        """
        Main control loop running at 20Hz.
        Implements pure pursuit control algorithm.
        """
        # Check preconditions
        if self.trajectory is None or len(self.trajectory.poses) == 0:
            self._publish_zero_velocity()
            return
        
        if self.current_pose is None or self.current_yaw is None:
            self._publish_zero_velocity()
            return
        
        if self.goal_reached:
            self._publish_zero_velocity()
            return
        
        try:
            # Check if goal reached (only after making progress to avoid immediate detection)
            # Require at least 10% progress through trajectory before checking goal
            min_progress = max(10, int(0.1 * len(self.trajectory.poses)))
            
            if self.target_index >= min_progress:
                goal_pose = self.trajectory.poses[-1].pose.position
                distance_to_goal = self._compute_distance(
                    self.current_pose.x, self.current_pose.y,
                    goal_pose.x, goal_pose.y
                )
                
                if distance_to_goal < self.goal_tolerance:
                    self.get_logger().info(
                        f'Goal reached! Final distance: {distance_to_goal:.3f}m'
                    )
                    self.goal_reached = True
                    self._publish_zero_velocity()
                    return
            
            # Find lookahead point on trajectory
            lookahead_point, target_idx = self._find_lookahead_point()
            
            if lookahead_point is None:
                self.get_logger().warn('No lookahead point found, stopping')
                self._publish_zero_velocity()
                return
            
            self.target_index = target_idx
            
            # Compute control commands using pure pursuit
            linear_vel, angular_vel = self._compute_control(lookahead_point)
            
            # Apply hard velocity limits (TurtleBot3 physical constraints)
            linear_vel = self._clamp(
                linear_vel, 
                -self.MAX_LINEAR_VELOCITY, 
                self.MAX_LINEAR_VELOCITY
            )
            angular_vel = self._clamp(
                angular_vel,
                -self.MAX_ANGULAR_VELOCITY,
                self.MAX_ANGULAR_VELOCITY
            )
            
            # Debug logging (every 100 iterations = once per 5 seconds at 20Hz)
            self.loop_counter += 1
            if self.loop_counter % 100 == 0:
                # Compute debug info
                dx = lookahead_point.x - self.current_pose.x
                dy = lookahead_point.y - self.current_pose.y
                distance_to_target = math.sqrt(dx**2 + dy**2)
                desired_heading = math.atan2(dy, dx)
                alpha = self.normalize_angle(desired_heading - self.current_yaw)
                
                self.get_logger().info(
                    f'[CONTROL] Index: {target_idx}/{len(self.trajectory.poses)} | '
                    f'Dist: {distance_to_target:.3f}m | '
                    f'Alpha: {math.degrees(alpha):.1f}° | '
                    f'Cmd: v={linear_vel:.3f} ω={angular_vel:.3f} | '
                    f'Pos: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}) | '
                    f'Target: ({lookahead_point.x:.2f}, {lookahead_point.y:.2f})'
                )
            
            # Publish velocity command
            self._publish_velocity(linear_vel, angular_vel)
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
            self._publish_zero_velocity()
    
    def _find_lookahead_point(self):
        """
        Find the lookahead point on trajectory at lookahead_distance ahead.
        
        Returns:
            tuple: (lookahead_point, target_index) or (None, 0) if not found
        """
        if self.trajectory is None or len(self.trajectory.poses) == 0:
            return None, 0
        
        # Start search from current target index
        min_idx = self.target_index
        max_idx = len(self.trajectory.poses)
        
        # Search forward for point at approximately lookahead_distance
        for i in range(min_idx, max_idx):
            pose = self.trajectory.poses[i].pose.position
            distance = self._compute_distance(
                self.current_pose.x, self.current_pose.y,
                pose.x, pose.y
            )
            
            # Found point at or beyond lookahead distance
            if distance >= self.lookahead_distance:
                return pose, i
        
        # If no point found at lookahead distance, use last point (approaching goal)
        return self.trajectory.poses[-1].pose.position, len(self.trajectory.poses) - 1
    
    def _compute_control(self, lookahead_point):
        """
        Compute control commands using pure pursuit algorithm.
        
        Pure Pursuit Control Law:
            ω = 2v·sin(α)/L
        
        where:
            α = heading error (angle from robot to lookahead point)
            L = lookahead distance
            v = desired linear velocity
        
        Args:
            lookahead_point: Target point to track
        
        Returns:
            tuple: (linear_velocity, angular_velocity)
        """
        try:
            # Compute vector from robot to lookahead point
            dx = lookahead_point.x - self.current_pose.x
            dy = lookahead_point.y - self.current_pose.y
            
            # Desired heading (angle to lookahead point)
            desired_heading = math.atan2(dy, dx)
            
            # Heading error (normalized to [-π, π])
            alpha = self.normalize_angle(desired_heading - self.current_yaw)
            
            # Distance to lookahead point
            distance = math.sqrt(dx**2 + dy**2)
            
            # Pure pursuit control law: ω = 2v·sin(α)/L
            # Use actual distance as L (not fixed lookahead_distance)
            if distance < 1e-6:
                # Avoid division by zero
                angular_velocity = 0.0
            else:
                angular_velocity = (2.0 * self.desired_velocity * math.sin(alpha)) / distance
            
            # Linear velocity: full speed forward, reduce when turning sharply
            # Scale down velocity for large heading errors
            linear_velocity = self.desired_velocity * math.cos(alpha)
            
            # Ensure forward progress (minimum velocity)
            if linear_velocity < 0.05:
                linear_velocity = 0.05
            
            return linear_velocity, angular_velocity
            
        except Exception as e:
            self.get_logger().error(f'Control computation failed: {e}')
            return 0.0, 0.0
    
    def normalize_angle(self, angle):
        """
        Normalize angle to [-π, π] range.
        
        CRITICAL: Always normalize angles before control computation to avoid
        discontinuities at ±π boundary.
        
        Args:
            angle: Angle in radians
        
        Returns:
            float: Normalized angle in [-π, π]
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def _compute_distance(self, x1, y1, x2, y2):
        """Compute Euclidean distance between two points."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def _clamp(self, value, min_value, max_value):
        """Clamp value to [min_value, max_value] range."""
        return max(min_value, min(max_value, value))
    
    def _publish_velocity(self, linear, angular):
        """Publish velocity command to /cmd_vel."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)
    
    def _publish_zero_velocity(self):
        """Publish zero velocity (safety fallback)."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TrajectoryController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Controller error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


