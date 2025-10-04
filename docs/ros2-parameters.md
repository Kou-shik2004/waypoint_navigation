# ROS2 Parameters with rclpy

## Overview

ROS2 params allow you to provide configuration for a node at run time. All params specified for a node are specific to this node and only exist while the node is alive. You can start your node with different settings each time without changing your Python code.

## Declaring ROS2 Params

**Important**: You must declare every ROS2 param you'll use in your node, typically in the node's constructor. If you don't declare a parameter, you will get a `ParameterNotDeclaredException` when trying to get or set it.

### Basic Declaration

```python
import rclpy
from rclpy.node import Node

class TestParams(Node):
    def __init__(self):
        super().__init__('test_params_rclpy')
        
        # Declare parameters with type specification
        self.declare_parameter('my_str', rclpy.Parameter.Type.STRING)
        self.declare_parameter('my_int', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('my_double_array', rclpy.Parameter.Type.DOUBLE_ARRAY)
```

### Declare Multiple Parameters

```python
self.declare_parameters(
    namespace='',
    parameters=[
        ('my_str', rclpy.Parameter.Type.STRING),
        ('my_int', rclpy.Parameter.Type.INTEGER),
        ('my_double_array', rclpy.Parameter.Type.DOUBLE_ARRAY)
    ]
)
```

### Undeclare a Parameter

```python
self.undeclare_parameter('my_str')
```

## Running Nodes with Parameters

### Without Parameters

```bash
ros2 run ros2_tutorials test_params_rclpy
```

### With Parameters

```bash
ros2 run ros2_tutorials test_params_rclpy --ros-args -p my_str:="Hello world" -p my_int:=5 -p my_double_array:="[4.4, 5.5, 6.6]"
```

### Using Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_tutorials',
            executable='test_params_rclpy',
            parameters=[
                {'my_str': 'Hello world'},
                {'my_int': 5},
                {'my_double_array': [4.4, 5.5, 6.6]}
            ],
            output='screen',
            emulate_tty=True
        )
    ])
```

## Getting Parameters

### Get One by One

```python
param_str = self.get_parameter('my_str')
param_int = self.get_parameter('my_int')
param_double_array = self.get_parameter('my_double_array')

# Access value
self.get_logger().info("str: %s, int: %s, double[]: %s" % (
    str(param_str.value),
    str(param_int.value),
    str(param_double_array.value)
))
```

### Get Multiple at Once

```python
(param_str, param_int, param_double_array) = self.get_parameters(
    ['my_str', 'my_int', 'my_double_array']
)
```

### Set Default Values

```python
# Single parameter with default
self.declare_parameter('my_str', 'default value')
self.declare_parameter('my_int', 7)
self.declare_parameter('my_double_array', [1.1, 2.2])

# Multiple parameters with defaults
self.declare_parameters(
    namespace='',
    parameters=[
        ('my_str', "default value"),
        ('my_int', 7),
        ('my_double_array', [1.1, 2.2])
    ]
)
```

## Setting Parameters

```python
from rclpy.parameter import Parameter

# Create parameter objects
param_str = Parameter('my_str', Parameter.Type.STRING, 'Set from code')
param_int = Parameter('my_int', Parameter.Type.INTEGER, 12)
param_double_array = Parameter('my_double_array', Parameter.Type.DOUBLE_ARRAY, [1.1, 2.2])

# Set parameters
self.set_parameters([param_str, param_int, param_double_array])
```

## Allow Undeclared Parameters

```python
def __init__(self):
    super().__init__('test_params_rclpy',
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True)
    
    # Use get_parameter_or for safe retrieval
    param_str = self.get_parameter_or(
        'my_str', 
        Parameter('str', Parameter.Type.STRING, 'Is there anybody out there?'))
```

## Parameter Callbacks

```python
from rcl_interfaces.msg import SetParametersResult

class TestParams(Node):
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'my_str' and param.type_ == Parameter.Type.STRING:
                self.my_str = param.value
        return SetParametersResult(successful=True)
    
    def __init__(self):
        super().__init__('test_params_rclpy')
        
        self.declare_parameter('my_str', 'default value')
        self.my_str = self.get_parameter('my_str').value
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
```

## Command Line Tools

### List Parameters

```bash
ros2 param list
```

### Get Parameter Value

```bash
ros2 param get /test_params_rclpy my_str
```

### Set Parameter Value

```bash
ros2 param set /test_params_rclpy my_str "new value"
```

## Important Notes

- Parameters not declared within a node won't appear in the parameter list and won't be available
- Attempting to access an undeclared parameter raises `ParameterNotDeclaredException`
- Declaring a parameter doesn't set a value, it just means the parameter exists
- Each parameter is specific to its node and only exists while the node is alive
- For global parameters shared by multiple nodes, create a dedicated parameter node

## Best Practices

1. **Always declare parameters first** - Do this in the node constructor
2. **Use default values** - Prevents errors when parameters aren't set at launch
3. **Use YAML config files** - For many parameters, store them in separate config files
4. **Document parameters** - Make it clear what each parameter does and its valid range
5. **Validate parameter values** - Use callbacks to ensure parameters are within acceptable ranges
