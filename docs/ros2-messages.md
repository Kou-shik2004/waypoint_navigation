# ROS 2 Messages

## Overview
ROS 2 messages are the primary containers for exchanging data between nodes. Publishers and subscribers use messages on specified topics to carry data throughout the ROS system.

## Message Type Structure
Each message type follows the naming convention:
```
package_name/msg/TypeName
```

For example: `sensor_msgs/LaserScan`, `geometry_msgs/Twist`

## Common Message Types

### nav_msgs/Path
Represents a path for a robot to follow.

**Fields:**
- `std_msgs/Header header` - Frame ID of the path
- `geometry_msgs/PoseStamped[] poses` - Array of poses to follow

### nav_msgs/Odometry
Represents position and velocity estimates in free space.

**Fields:**
- `std_msgs/Header header` - Pose parent frame
- `string child_frame_id` - Twist coordinate frame
- `geometry_msgs/PoseWithCovariance pose` - Estimated pose (typically relative to fixed world frame)
- `geometry_msgs/TwistWithCovariance twist` - Estimated linear and angular velocity

### geometry_msgs/Twist
Expresses velocity in free space (linear and angular components).

**Fields:**
- `Vector3 linear` - Linear velocity (x, y, z)
- `Vector3 angular` - Angular velocity (x, y, z)

## Working with Messages in Code

### Creating Messages
```python
import rclpy
from geometry_msgs.msg import Twist

# Create a new message
twist = ros2message("geometry_msgs/Twist")
```

### Accessing Message Fields
```python
# Access nested fields
x_velocity = twist.linear.x
z_rotation = twist.angular.z
```

### Setting Message Data
```python
twist.linear.y = 5.0
twist.angular.z = 1.2
```

### Copying Messages
Messages are structures and can be copied directly:
```python
twist_copy = twist  # Creates independent copy
```

### Saving and Loading Messages
```python
# Save to file
save("pose_file.mat", "pose_data")

# Load from file
message_data = load("pose_file.mat")
```

## Finding Available Message Types
```bash
# List all topics with message types
ros2 topic list -t

# List all available message types
ros2 msg list

# Show message definition
ros2 msg show geometry_msgs/Twist
```

## Best Practices
- Always initialize numeric fields (default is 0)
- Use appropriate message types for your data
- Keep blank message templates for repeated use
- Validate message structure before publishing
