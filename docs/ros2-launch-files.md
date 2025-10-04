# ROS2 Python Launch Files

## Overview

Launch files allow you to start multiple nodes with complete configuration (remapping, parameters, etc.) from a single file using one command. This is much more practical than starting each node manually in different terminals.

## Creating Launch Files

### File Location

**Option 1: Existing Package**
- Create a `launch/` folder at the root of your package
- Place all launch files in this folder

**Option 2: Dedicated Launch Package (Recommended)**
- Create a new package specifically for launch files
- Common naming: `<robot_name>_bringup`
- Example: `my_robot_bringup`

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_bringup
cd my_robot_bringup/
rm -rf include/ src/
mkdir launch
touch launch/demo.launch.py
```

## Basic Launch File Structure

### Minimal Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker"
    )
    
    listener_node = Node(
        package="demo_nodes_py",
        executable="listener"
    )
    
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    
    return ld
```

### Required Components

1. **Import statements**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
```

2. **Function definition**
```python
def generate_launch_description():
    # Must return LaunchDescription
```

3. **Create LaunchDescription**
```python
ld = LaunchDescription()
```

4. **Add nodes**
```python
ld.add_action(node_object)
```

5. **Return LaunchDescription**
```python
return ld
```

## Installing Launch Files

### Add Dependencies (package.xml)

```xml
<exec_depend>demo_nodes_cpp</exec_depend>
<exec_depend>demo_nodes_py</exec_depend>
```

### From Cpp Package (CMakeLists.txt)

Add after `find_package(ament_cmake REQUIRED)`:

```cmake
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)
```

### From Python Package (setup.py)

```python
from setuptools import setup
import os
from glob import glob

setup(
    # ... other setup parameters ...
    data_files=[
        # ... other data files ...
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
)
```

### Build Package

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash  # Or setup.zsh for zsh
```

## Running Launch Files

```bash
ros2 launch <package_name> <launch_file_name>
```

Example:
```bash
ros2 launch my_robot_bringup demo.launch.py
```

## Node Customization

### Rename Node

```python
node = Node(
    package="demo_nodes_cpp",
    executable="talker",
    name="my_talker"  # Custom name
)
```

### Topic/Service Remapping

```python
node = Node(
    package="demo_nodes_cpp",
    executable="talker",
    remappings=[
        ("chatter", "my_chatter"),
        ("service_name", "new_service_name")
    ]
)
```

### Set Parameters

**Direct Parameter Setting:**
```python
node = Node(
    package="turtlesim",
    executable="turtlesim_node",
    parameters=[
        {"background_b": 200},
        {"background_g": 200},
        {"background_r": 200}
    ]
)
```

**From YAML File:**
```python
import os
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
    get_package_share_directory('my_package'),
    'config',
    'params.yaml'
)

node = Node(
    package="my_package",
    executable="my_node",
    parameters=[config]
)
```

### Output Configuration

```python
node = Node(
    package="demo_nodes_cpp",
    executable="talker",
    output='screen',      # Print to screen
    emulate_tty=True      # Better formatting
)
```

### Namespace

```python
node = Node(
    package="demo_nodes_cpp",
    executable="talker",
    namespace='robot1'  # All topics prefixed with /robot1
)
```

## Complete Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    # Load config file
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'robot_params.yaml'
    )
    
    # First node with custom configuration
    controller_node = Node(
        package="my_robot_control",
        executable="controller",
        name="robot_controller",
        namespace="robot1",
        parameters=[
            config,
            {"use_sim_time": True}
        ],
        remappings=[
            ("cmd_vel", "robot1/cmd_vel"),
            ("odom", "robot1/odom")
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Second node
    sensor_node = Node(
        package="my_robot_sensors",
        executable="lidar_processor",
        name="lidar",
        namespace="robot1",
        parameters=[{"scan_topic": "/scan"}],
        output='screen'
    )
    
    # Add all nodes
    ld.add_action(controller_node)
    ld.add_action(sensor_node)
    
    return ld
```

## Advanced Features

### Passing Arguments to Launch Files

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Use argument
    node = Node(
        package='my_package',
        executable='my_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        node
    ])
```

Run with:
```bash
ros2 launch my_package demo.launch.py use_sim_time:=true
```

### Include Other Launch Files

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('other_package'),
                'launch',
                'other.launch.py'
            )
        ])
    )
    
    return LaunchDescription([other_launch])
```

### Conditional Nodes

```python
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='my_package',
        executable='my_node',
        condition=IfCondition(LaunchConfiguration('start_node'))
    )
    
    return LaunchDescription([node])
```

### Group Actions

```python
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    group = GroupAction([
        PushRosNamespace('robot1'),
        Node(package='pkg', executable='node1'),
        Node(package='pkg', executable='node2'),
    ])
    
    return LaunchDescription([group])
```

### Lifecycle Nodes

```python
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    lifecycle_node = LifecycleNode(
        package='my_package',
        executable='my_lifecycle_node',
        name='my_node',
        namespace='',
        output='screen'
    )
    
    return LaunchDescription([lifecycle_node])
```

### Execute Process (Non-ROS)

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    process = ExecuteProcess(
        cmd=['echo', 'Hello World'],
        output='screen'
    )
    
    return LaunchDescription([process])
```

### Event Handlers

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(package='pkg', executable='node1', name='node1')
    node2 = Node(package='pkg', executable='node2', name='node2')
    
    # Start node2 when node1 exits
    handler = RegisterEventHandler(
        OnProcessExit(
            target_action=node1,
            on_exit=[node2]
        )
    )
    
    return LaunchDescription([node1, handler])
```

## Example YAML Config File

Create `config/robot_params.yaml`:

```yaml
/**:
  ros__parameters:
    max_velocity: 1.0
    max_acceleration: 0.5
    controller_frequency: 50.0
    use_sim_time: false
    
robot_controller:
  ros__parameters:
    kp_linear: 1.0
    kp_angular: 2.0
    control_mode: "velocity"
```

Load in launch file:

```python
config = os.path.join(
    get_package_share_directory('my_robot_bringup'),
    'config',
    'robot_params.yaml'
)

node = Node(
    package="my_package",
    executable="my_node",
    parameters=[config]
)
```

## Best Practices

1. **Use dedicated bringup package** - Keep launch files separate from code
2. **Organize by functionality** - Group related nodes in same launch file
3. **Use YAML configs** - Store parameters in separate config files
4. **Add comments** - Document what each node does
5. **Handle dependencies** - Add all package dependencies to package.xml
6. **Use namespaces** - Organize multi-robot systems
7. **Set output wisely** - Use 'screen' for debugging, 'log' for production
8. **Version control configs** - Track parameter changes
9. **Use arguments** - Make launch files configurable
10. **Test incrementally** - Start with simple launches, add complexity gradually

## Debugging

### Check if nodes are running
```bash
ros2 node list
```

### View node info
```bash
ros2 node info /node_name
```

### Check topics
```bash
ros2 topic list
ros2 topic echo /topic_name
```

### View parameters
```bash
ros2 param list
ros2 param get /node_name param_name
```

### Launch file syntax check
```bash
ros2 launch --show-args my_package demo.launch.py
```

## Common Issues

### Issue: Nodes not starting
- Check package dependencies in package.xml
- Verify executable names
- Ensure package is built: `colcon build`
- Source workspace: `source install/setup.bash`

### Issue: Parameters not loading
- Check YAML file path
- Verify parameter names match node expectations
- Check namespace configuration

### Issue: Topic remapping not working
- Verify both sides of remapping tuple
- Check for namespace conflicts
- Use `ros2 topic list` to verify actual topic names

### Issue: Launch file not found
- Ensure launch file is installed (check CMakeLists.txt or setup.py)
- Rebuild package after adding install rules
- Check file permissions (should be executable)

## Multi-Robot Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = []
    
    # Launch multiple robots
    for i in range(1, 4):  # robots 1, 2, 3
        robot = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name=f'turtle{i}',
            namespace=f'robot{i}',
            parameters=[{
                'background_r': i * 70,
                'background_g': 100,
                'background_b': 200
            }]
        )
        robots.append(robot)
    
    ld = LaunchDescription()
    for robot in robots:
        ld.add_action(robot)
    
    return ld
```

## Summary

Launch files are essential for:
- Starting multiple nodes simultaneously
- Configuring nodes with parameters
- Managing complex robot systems
- Ensuring consistent startup procedures
- Simplifying deployment and testing

Always install your launch files properly and test them incrementally as you build complexity.
