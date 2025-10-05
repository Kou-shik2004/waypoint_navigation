#!/usr/bin/env python3
"""
Unit tests for trajectory navigation system.
Tests core algorithmic functions and parameter validation.
"""

import pytest
import numpy as np
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from trajectory_nav.trajectory_generator import TrajectoryGenerator
from trajectory_nav.trajectory_controller import TrajectoryController
from trajectory_nav.trajectory_monitor import TrajectoryMonitor


@pytest.fixture(scope='module')
def ros_context():
    """Initialize and cleanup ROS2 context for all tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


class TestTrajectoryController:
    """Test pure pursuit controller logic."""
    
    def test_angle_normalization(self, ros_context):
        """Test angle wrapping to [-pi, pi] range."""
        node = TrajectoryController()
        
        # Test cases: (input, expected_output)
        test_cases = [
            (0.0, 0.0),
            (math.pi, math.pi),
            (-math.pi, -math.pi),
            (2 * math.pi, 0.0),
            (-2 * math.pi, 0.0),
            (math.pi / 2, math.pi / 2),
            (-math.pi / 2, -math.pi / 2),
        ]
        
        for input_angle, expected in test_cases:
            result = node.normalize_angle(input_angle)
            assert abs(result - expected) < 1e-6, \
                f"normalize_angle({input_angle}) = {result}, expected {expected}"
        
        node.destroy_node()
    
    def test_distance_computation(self, ros_context):
        """Test Euclidean distance calculation."""
        node = TrajectoryController()
        
        # Test cases: (x1, y1, x2, y2, expected_distance)
        test_cases = [
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, 1.0, 1.0),
            (0.0, 0.0, 3.0, 4.0, 5.0),  # 3-4-5 triangle
            (1.0, 1.0, 4.0, 5.0, 5.0),
        ]
        
        for x1, y1, x2, y2, expected in test_cases:
            result = node._compute_distance(x1, y1, x2, y2)
            assert abs(result - expected) < 1e-6, \
                f"distance({x1},{y1} to {x2},{y2}) = {result}, expected {expected}"
        
        node.destroy_node()
    
    def test_velocity_clamping(self, ros_context):
        """Test that velocity limits are enforced."""
        node = TrajectoryController()
        
        # Test clamping function
        assert node._clamp(0.5, 0.0, 1.0) == 0.5
        assert node._clamp(1.5, 0.0, 1.0) == 1.0
        assert node._clamp(-0.5, 0.0, 1.0) == 0.0
        
        # Test that physical limits are defined correctly
        assert node.MAX_LINEAR_VELOCITY == 0.22
        assert node.MAX_ANGULAR_VELOCITY == 2.84
        
        node.destroy_node()
    
    def test_controller_initialization(self, ros_context):
        """Test controller can be created with default parameters."""
        node = TrajectoryController()
        
        assert node.lookahead_distance == 0.30
        assert node.goal_tolerance == 0.05
        assert node.trajectory is None
        assert node.goal_reached is False
        
        node.destroy_node()


class TestTrajectoryMonitor:
    """Test monitoring and error computation."""
    
    def test_cross_track_error_computation(self, ros_context):
        """Test cross-track error calculation with known geometry."""
        node = TrajectoryMonitor()
        
        # Create a straight line trajectory along x-axis
        trajectory_points = np.array([
            [0.0, 0.0],
            [1.0, 0.0],
            [2.0, 0.0],
            [3.0, 0.0],
        ])
        
        node.trajectory_points = trajectory_points
        
        # Robot exactly on the line at point (1.0, 0.0)
        class MockPose:
            pass
        mock_pose = MockPose()
        mock_pose.x = 1.0
        mock_pose.y = 0.0
        node.current_pose = mock_pose
        
        error = node._compute_cross_track_error()
        assert abs(error - 0.0) < 1e-6
        
        # Robot 0.5m above the line
        mock_pose.x = 1.0
        mock_pose.y = 0.5
        error = node._compute_cross_track_error()
        assert abs(error - 0.5) < 1e-6
        
        # Robot at (0.5, 0.5) - closest point is (0.0, 0.0)
        mock_pose.x = 0.5
        mock_pose.y = 0.5
        error = node._compute_cross_track_error()
        expected = math.sqrt(0.5**2 + 0.5**2)  # sqrt(0.5) = 0.707
        assert abs(error - expected) < 1e-6
        
        node.destroy_node()
    
    def test_empty_trajectory_handling(self, ros_context):
        """Test that monitor handles empty trajectory gracefully."""
        node = TrajectoryMonitor()
        
        # No trajectory set
        assert node.trajectory is None
        assert node.trajectory_points is None
        
        node.destroy_node()
    
    def test_monitor_initialization(self, ros_context):
        """Test monitor initializes with correct defaults."""
        node = TrajectoryMonitor()
        
        assert node.trajectory is None
        assert node.error_history == []
        assert node.max_error == 0.0
        assert node.tracking_active is False
        
        node.destroy_node()


class TestAlgorithmicFunctions:
    """Test standalone algorithmic correctness."""
    
    def test_angle_wrap_edge_cases(self, ros_context):
        """Test angle normalization handles edge cases."""
        node = TrajectoryController()
        
        # Large positive angles
        assert abs(node.normalize_angle(10 * math.pi)) < 1e-6
        
        # Large negative angles  
        assert abs(node.normalize_angle(-10 * math.pi)) < 1e-6
        
        # Just over pi wraps to negative
        result = node.normalize_angle(math.pi + 0.01)
        assert result < 0  # Should wrap to negative side
        
        node.destroy_node()
    
    def test_distance_zero_case(self, ros_context):
        """Test distance computation for identical points."""
        node = TrajectoryController()
        
        # Same point
        assert node._compute_distance(5.0, 5.0, 5.0, 5.0) == 0.0
        
        # Very close points
        dist = node._compute_distance(0.0, 0.0, 1e-10, 1e-10)
        assert dist < 1e-9
        
        node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])