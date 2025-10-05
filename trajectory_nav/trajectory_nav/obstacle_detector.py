#!/usr/bin/env python3
"""
Obstacle Detector Node - Prototype Implementation

This is a prototype implementation demonstrating the obstacle avoidance
extension architecture described in DESIGN.md. 

NOT INTEGRATED: This node is included as proof-of-concept code to show
how the system would be extended for obstacle detection. It is not
connected to the trajectory controller in the current submission.

For integration details and architecture, see DESIGN.md Section 7:
"Obstacle Avoidance Extension (Future Work)"

License: MIT
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleDetector(Node):
    """Detects obstacles in forward path and publishes stop signal."""
    
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Parameters
        self.declare_parameter('safety_distance', 0.5)  # meters
        self.declare_parameter('scan_angle', 60.0)      # degrees (forward cone)
        
        self.safety_distance = self.get_parameter('safety_distance').value
        self.scan_angle_rad = math.radians(self.get_parameter('scan_angle').value)
        
        # State
        self.obstacle_detected = False
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        
        # Publishers
        self.safety_stop_pub = self.create_publisher(Bool, '/safety_stop', 10)
        
        # Timer for consistent publishing (10Hz)
        self.timer = self.create_timer(0.1, self._publish_safety_status)
        
        self.get_logger().info(
            f'Obstacle detector initialized: safety_distance={self.safety_distance}m, '
            f'scan_angle={math.degrees(self.scan_angle_rad):.0f}°'
        )
    
    def _scan_callback(self, msg: LaserScan):
        """Process laser scan and detect obstacles in forward cone."""
        obstacle_detected = False
        min_distance = float('inf')
        
        # Check ranges in forward cone
        num_ranges = len(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        for i, r in enumerate(msg.ranges):
            # Skip invalid readings
            if math.isinf(r) or math.isnan(r) or r <= 0:
                continue
            
            # Compute angle of this ray
            angle = angle_min + i * angle_increment
            
            # Check if ray is in forward cone (within scan_angle/2 of 0°)
            if abs(angle) <= self.scan_angle_rad / 2:
                # Track minimum distance in cone
                if r < min_distance:
                    min_distance = r
                
                # Check if obstacle is too close
                if r < self.safety_distance:
                    obstacle_detected = True
        
        # Update state
        prev_state = self.obstacle_detected
        self.obstacle_detected = obstacle_detected
        
        # Log state changes
        if obstacle_detected and not prev_state:
            self.get_logger().warn(
                f'OBSTACLE DETECTED at {min_distance:.2f}m - STOPPING'
            )
        elif not obstacle_detected and prev_state:
            self.get_logger().info('Path clear - RESUMING')
    
    def _publish_safety_status(self):
        """Publish current safety stop status at 10Hz."""
        stop_msg = Bool()
        stop_msg.data = self.obstacle_detected
        self.safety_stop_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObstacleDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
