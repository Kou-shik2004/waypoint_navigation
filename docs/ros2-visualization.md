# ROS 2 Visualization with RViz Markers

## Overview
The Markers display in RViz allows programmatic addition of primitive shapes to the 3D view through `visualization_msgs/msg/Marker` or `visualization_msgs/msg/MarkerArray` messages.

## Setting Up a Marker Publisher

```cpp
auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>
    ("visualization_marker", 1);
```

## Marker Message Structure

### Essential Parameters

**Header**
- `frame_id` - Coordinate frame for the marker
- `stamp` - Timestamp for the marker

**Identification**
- `ns` (namespace) - Combined with `id` to create unique identifier
- `id` - Unique ID within namespace

**Type and Action**
- `type` - Marker shape (ARROW, CUBE, SPHERE, etc.)
- `action` - Operation: ADD (0), MODIFY (0), DELETE (2), DELETEALL (3)

**Pose and Scale**
- `pose` - Position (x, y, z) and orientation (quaternion)
- `scale` - Size (x, y, z) in meters

**Appearance**
- `color` - RGBA values in range [0.0-1.0]
  - **Important:** Alpha must be > 0 or marker will be transparent!
- `lifetime` - Auto-deletion time (0 = forever)

**Advanced**
- `frame_locked` - Retransform to frame each update
- `points[]` - Used for LINE_STRIP, LINE_LIST, POINTS, etc.
- `colors[]` - Per-vertex colors (optional)
- `text` - For TEXT_VIEW_FACING markers
- `mesh_resource` - URI for MESH_RESOURCE markers

## Marker Types

### Basic Shapes

#### ARROW (0)
Two specification methods:
1. **Position/Orientation**: scale.x = length, scale.y = width, scale.z = height
2. **Start/End Points**: scale.x = shaft diameter, scale.y = head diameter, scale.z = head length

#### CUBE (1)
- Pivot at center
- scale.x, scale.y, scale.z define dimensions

#### SPHERE (2)
- Pivot at center
- Different scale values create ellipsoid

#### CYLINDER (3)
- Pivot at center
- scale.x, scale.y define diameter, scale.z = height

### Line Types

#### LINE_STRIP (4)
- Connects consecutive points: 0-1, 1-2, 2-3...
- Only scale.x used (line width)

#### LINE_LIST (5)
- Connects point pairs: 0-1, 2-3, 4-5...
- Only scale.x used (line width)

### List Types (Performance Optimized)

#### CUBE_LIST (6)
- Batch rendering for multiple cubes
- All cubes share same scale
- Uses `points[]` for positions

#### SPHERE_LIST (7)
- Batch rendering for multiple spheres
- All spheres share same scale
- Uses `points[]` for positions

#### POINTS (8)
- scale.x = point width, scale.y = point height
- Uses `points[]` array

### Special Types

#### TEXT_VIEW_FACING (9)
- Always oriented toward viewer
- Uses `text` field
- scale.z = character height

#### MESH_RESOURCE (10)
```cpp
marker.type = visualization_msgs::Marker::MESH_RESOURCE;
marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
```
- Supports .stl, .mesh, .dae formats
- Scale is relative to mesh file size
- `mesh_use_embedded_materials` flag for COLLADA materials

#### TRIANGLE_LIST (11)
- Every 3 points form a triangle: 0-1-2, 3-4-5...
- Uses `points[]` and optionally `colors[]`

## Example Usage

```cpp
visualization_msgs::msg::Marker marker;
marker.header.frame_id = "/my_frame";
marker.header.stamp = rclcpp::Clock().now();
marker.ns = "basic_shapes";
marker.id = 0;
marker.type = visualization_msgs::msg::Marker::SPHERE;
marker.action = visualization_msgs::msg::Marker::ADD;

marker.pose.position.x = 0;
marker.pose.position.y = 0;
marker.pose.position.z = 0;
marker.pose.orientation.w = 1.0;

marker.scale.x = 1.0;
marker.scale.y = 1.0;
marker.scale.z = 1.0;

marker.color.r = 0.0f;
marker.color.g = 1.0f;
marker.color.b = 0.0f;
marker.color.a = 1.0;  // Critical: set alpha!

marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

marker_pub->publish(marker);
```

## Performance Notes
- Single markers with lists (CUBE_LIST, SPHERE_LIST) render much faster than individual markers
- Use MarkerArray for multiple different markers
- Batch rendering is more efficient for thousands of objects
