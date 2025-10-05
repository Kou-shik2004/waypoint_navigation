# ROS 2 Transforms (TF2)

## Overview
TF2 is the transform library for ROS 2, providing a flexible system for tracking coordinate frames over time. It manages the relationships between coordinate frames in a tree structure.

## Core Concepts

### Transform Tree
- Hierarchical structure of coordinate frames
- Each frame has one parent and potentially multiple children
- Transforms describe the pose of child frames relative to parent frames

### Three Components

1. **Broadcasters** - Publish transform data
2. **Listeners** - Query and use transform data  
3. **Buffer** - Stores and manages transform history

## Dynamic Transform Broadcaster

Publishes continuously changing transforms (e.g., moving robot parts).

### Setup

```python
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

self.tfb_ = TransformBroadcaster(self)
```

### Publishing Transforms

```python
tfs = TransformStamped()

# Header
tfs.header.stamp = self.get_clock().now().to_msg()
tfs.header.frame_id = "world"  # Parent frame
tfs.child_frame_id = "turtle1"  # Child frame

# Translation (meters)
tfs.transform.translation.x = msg.x
tfs.transform.translation.y = msg.y
tfs.transform.translation.z = 0.0

# Rotation (quaternion)
r = R.from_euler('xyz', [0, 0, msg.theta])
tfs.transform.rotation.x = r.as_quat()[0]
tfs.transform.rotation.y = r.as_quat()[1]
tfs.transform.rotation.z = r.as_quat()[2]
tfs.transform.rotation.w = r.as_quat()[3]

# Broadcast
self.tfb_.sendTransform(tfs)
```

## Transform Listener

Queries the TF tree to get transforms between frames.

### Setup

```python
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

self._tf_buffer = Buffer()
self._tf_listener = TransformListener(self._tf_buffer, self)
```

### Looking Up Transforms

```python
try:
    # Get transform from source_frame to target_frame
    trans = self._tf_buffer.lookup_transform(
        target_frame,    # Target frame
        source_frame,    # Source frame
        rclpy.time.Time()  # Time (Time() = latest available)
    )
    
    # Access transform data
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    
except LookupException as e:
    self.get_logger().error('Transform lookup failed: {}'.format(repr(e)))
```

### Transform Direction
`lookup_transform(target, source, time)` returns the pose of the **source frame origin** with respect to the **target frame**.

## Static Transform Broadcaster

For fixed, non-moving transforms (e.g., sensor mounted on robot).

### Command Line Usage

```bash
ros2 run tf2_ros static_transform_publisher \
    x y z yaw pitch roll \
    parent_frame child_frame
```

**Example:**
```bash
ros2 run tf2_ros static_transform_publisher \
    0.1 0 0 -1.57 0.0 0.0 \
    turtle1 turtle_cam1
```

This creates a frame `turtle_cam1` at:
- 0.1m in x-axis from `turtle1`
- Rotated -1.57 rad in yaw

## Euler Angle Conversion

Using scipy for quaternion conversions:

```python
from scipy.spatial.transform import Rotation as R

# Euler to Quaternion
r = R.from_euler('xyz', [roll, pitch, yaw])
quat = r.as_quat()  # Returns [x, y, z, w]

# Quaternion to Euler
r = R.from_quat([x, y, z, w])
euler = r.as_euler('xyz')  # Returns [roll, pitch, yaw]
```

## Debugging and Visualization

### View Transform Tree
```bash
# Echo specific transform
ros2 run tf2_ros tf2_echo child_frame parent_frame

# Monitor all transforms
ros2 run tf2_ros tf2_monitor

# Visualize in RViz2
# Add TF display module
rviz2
```

### View TF Tree Structure
```bash
ros2 run tf2_tools view_frames
```

This generates a PDF showing the complete transform tree with frame relationships.

## Best Practices

1. **Naming Conventions**
   - Use descriptive frame names
   - Maintain consistent naming across system

2. **Timing**
   - Use `get_clock().now()` for current transforms
   - Historical lookups require buffered data

3. **Error Handling**
   - Always wrap lookups in try-except
   - Handle `LookupException`, `ConnectivityException`, `ExtrapolationException`

4. **Performance**
   - Static transforms have minimal overhead
   - Dynamic transforms should publish at appropriate rates
   - Use `waitForTransform()` for synchronization

5. **Frame Structure**
   - Keep tree structure simple and logical
   - Avoid circular dependencies
   - Document frame relationships

## Common Use Cases

### Sensor Frame Publishing
```python
# Publish static transform for a camera
tfs = TransformStamped()
tfs.header.stamp = self.get_clock().now().to_msg()
tfs.header.frame_id = "base_link"
tfs.child_frame_id = "camera_link"

tfs.transform.translation.x = 0.2
tfs.transform.translation.y = 0.0
tfs.transform.translation.z = 0.3

# Camera pointing forward
r = R.from_euler('xyz', [0, 0, 0])
quat = r.as_quat()
tfs.transform.rotation.x = quat[0]
tfs.transform.rotation.y = quat[1]
tfs.transform.rotation.z = quat[2]
tfs.transform.rotation.w = quat[3]

self.static_broadcaster.sendTransform(tfs)
```

### Following a Target
```python
# Get transform to target
try:
    trans = self._tf_buffer.lookup_transform(
        'base_link',
        'target_object',
        rclpy.time.Time()
    )
    
    # Calculate distance
    distance = math.sqrt(
        trans.transform.translation.x ** 2 +
        trans.transform.translation.y ** 2
    )
    
    # Calculate angle to target
    angle = math.atan2(
        trans.transform.translation.y,
        trans.transform.translation.x
    )
    
except LookupException:
    self.get_logger().warn('Target not found')
```

## Common Issues and Solutions

### Issue: Transform Lookup Fails
**Problem:** `LookupException` when calling `lookup_transform()`

**Solutions:**
1. Check if broadcaster is publishing
2. Verify frame names match exactly
3. Add delay or use `wait_for_transform()`
4. Check buffer history depth

### Issue: Transforms Too Old
**Problem:** `ExtrapolationException` when querying past transforms

**Solutions:**
1. Increase buffer cache time
2. Check broadcaster frequency
3. Use `Time(0)` for latest available transform

### Issue: Circular Frame Dependencies
**Problem:** Multiple parents for same frame

**Solutions:**
1. Restructure transform tree
2. Each frame can have only one parent
3. Use `view_frames` to visualize structure

## Complete Example: Robot with Sensor

```python
import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class RobotTFPublisher(Node):
    def __init__(self):
        super().__init__('robot_tf_publisher')
        
        # Dynamic broadcaster for robot pose
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Static broadcaster for sensor
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static sensor transform once
        self.publish_static_transforms()
        
        # Timer for dynamic robot pose
        self.timer = self.create_timer(0.1, self.publish_robot_pose)
        
    def publish_static_transforms(self):
        # Sensor mounted on robot
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        
        t.transform.translation.x = 0.15
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        
        r = R.from_euler('xyz', [0, 0, 0])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.static_broadcaster.sendTransform(t)
        
    def publish_robot_pose(self):
        # Dynamic robot pose in world
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        # Get current position (from odometry, etc.)
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0
        
        # Get current orientation
        r = R.from_euler('xyz', [0, 0, 0.5])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = RobotTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

TF2 provides a powerful and flexible system for managing coordinate frames in ROS 2:
- **Broadcasters** publish transform relationships
- **Listeners** query the transform tree
- **Static transforms** for fixed relationships
- **Dynamic transforms** for moving parts
- Rich debugging and visualization tools

Always maintain a clear, logical transform tree structure and handle exceptions properly for robust robot applications.
