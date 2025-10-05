# Path Interpolation with SciPy CubicSpline

## Overview

CubicSpline is used in ROS2 robotics applications for smooth path planning and trajectory generation. It provides a piecewise cubic polynomial that is twice continuously differentiable (C2 smooth), creating smooth paths between waypoints.

## Installation

```bash
pip3 install scipy
```

## Basic Usage

```python
import numpy as np
from scipy.interpolate import CubicSpline

# Waypoints (x values must be strictly increasing)
x = np.array([0, 1, 2, 3, 4])
y = np.array([0, 2, 1, 3, 2])

# Create cubic spline
cs = CubicSpline(x, y)

# Interpolate at new points
x_new = np.linspace(0, 4, 100)
y_new = cs(x_new)
```

## CubicSpline Parameters

### x: array_like, shape (n,)
1-D array of independent variable values (must be strictly increasing)
```python
x = np.array([0.0, 1.0, 2.0, 3.0])  # Valid
x = np.array([0.0, 2.0, 1.0, 3.0])  # Invalid - not increasing!
```

### y: array_like
Dependent variable values. Can be multi-dimensional.
```python
# 1D path
y = np.array([0, 1, 2, 3])

# 2D path (x, y coordinates)
y = np.array([[0, 0], [1, 2], [2, 1], [3, 3]])

# 3D path (x, y, z coordinates)
y = np.array([[0, 0, 0], [1, 2, 0.5], [2, 1, 1.0]])
```

### axis: int, optional (default=0)
Axis along which y is varying
```python
# Default: rows are waypoints
y = np.array([[x1, y1], [x2, y2], [x3, y3]])  # axis=0

# Columns are waypoints
y = np.array([[x1, x2, x3], [y1, y2, y3]])    # axis=1
```

### bc_type: string or 2-tuple, optional

Boundary condition types:

**'not-a-knot' (default)**
- First and second segments at curve ends are the same polynomial
- Good default when no information on boundary conditions
```python
cs = CubicSpline(x, y, bc_type='not-a-knot')
```

**'periodic'**
- Curve is periodic: `y[0] == y[-1]`
- Results in `y'[0] == y'[-1]` and `y''[0] == y''[-1]`
```python
# For closed loops
cs = CubicSpline(theta, y, bc_type='periodic')
```

**'clamped'**
- First derivative at ends are zero
```python
cs = CubicSpline(x, y, bc_type='clamped')
# Equivalent to:
cs = CubicSpline(x, y, bc_type=((1, 0.0), (1, 0.0)))
```

**'natural'**
- Second derivative at ends are zero
```python
cs = CubicSpline(x, y, bc_type='natural')
# Equivalent to:
cs = CubicSpline(x, y, bc_type=((2, 0.0), (2, 0.0)))
```

**Custom derivatives**
- Specify exact derivative values at endpoints
```python
# Format: (order, deriv_value)
# order: 1 (first derivative) or 2 (second derivative)

# y'(0) = 0, y'(1) = 3
cs = CubicSpline([0, 1], [0, 1], bc_type=((1, 0), (1, 3)))

# For 2D paths
y = np.array([[0, 0], [1, 1]])
deriv_start = np.array([0, 0])  # velocity at start
deriv_end = np.array([1, 1])    # velocity at end
cs = CubicSpline(x, y, bc_type=((1, deriv_start), (1, deriv_end)))
```

### extrapolate: {bool, 'periodic', None}, optional
- `True`: Extrapolate beyond data range using first/last intervals
- `False`: Return NaN for out-of-bounds points  
- `'periodic'`: Use periodic extrapolation
- `None` (default): `'periodic'` if `bc_type='periodic'`, else `True`

## Methods

### Evaluate Spline

```python
# Function value
y_new = cs(x_new)

# First derivative
dy_dx = cs(x_new, 1)

# Second derivative
d2y_dx2 = cs(x_new, 2)

# Third derivative
d3y_dx3 = cs(x_new, 3)
```

### Derivative

```python
# Get new spline representing derivative
cs_prime = cs.derivative(nu=1)
y_prime = cs_prime(x_new)
```

### Antiderivative

```python
# Get new spline representing antiderivative
cs_integral = cs.antiderivative(nu=1)
```

### Integrate

```python
# Definite integral from a to b
result = cs.integrate(a, b)
```

### Solve

```python
# Find x where cs(x) == y
roots = cs.solve(y=target_value)
```

### Roots

```python
# Find x where cs(x) == 0
roots = cs.roots()
```

## Robotics Applications

### 2D Path Planning

```python
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Waypoints
waypoints_x = np.array([0, 1, 2, 3, 4])
waypoints_y = np.array([0, 2, 1.5, 3, 2.5])

# Create spline (using distance as parameter)
distances = np.sqrt(np.diff(waypoints_x)**2 + np.diff(waypoints_y)**2)
distances = np.concatenate(([0], np.cumsum(distances)))

cs_x = CubicSpline(distances, waypoints_x)
cs_y = CubicSpline(distances, waypoints_y)

# Generate smooth path
s = np.linspace(0, distances[-1], 100)
path_x = cs_x(s)
path_y = cs_y(s)

# Velocity (first derivative)
vel_x = cs_x(s, 1)
vel_y = cs_y(s, 1)

# Acceleration (second derivative)
acc_x = cs_x(s, 2)
acc_y = cs_y(s, 2)

plt.plot(waypoints_x, waypoints_y, 'o', label='Waypoints')
plt.plot(path_x, path_y, '-', label='Smooth Path')
plt.legend()
plt.axis('equal')
plt.show()
```

### 3D Trajectory

```python
# 3D waypoints
waypoints = np.array([
    [0, 0, 0],
    [1, 1, 0.5],
    [2, 0.5, 1],
    [3, 2, 0.8],
    [4, 1, 0]
])

# Time parameter
t = np.array([0, 1, 2, 3, 4])

# Create splines for each dimension
cs_x = CubicSpline(t, waypoints[:, 0])
cs_y = CubicSpline(t, waypoints[:, 1])
cs_z = CubicSpline(t, waypoints[:, 2])

# Generate trajectory
t_new = np.linspace(0, 4, 200)
trajectory = np.column_stack([
    cs_x(t_new),
    cs_y(t_new),
    cs_z(t_new)
])

# Velocity profile
velocity = np.column_stack([
    cs_x(t_new, 1),
    cs_y(t_new, 1),
    cs_z(t_new, 1)
])

speed = np.linalg.norm(velocity, axis=1)
```

### Closed Loop Path (Periodic)

```python
# Circle waypoints
theta = np.linspace(0, 2*np.pi, 5)
x = np.cos(theta)
y = np.sin(theta)

# Ensure first and last points are identical for periodic
points = np.column_stack([x, y])

# Create periodic spline
cs = CubicSpline(theta, points, bc_type='periodic')

# Generate smooth closed path
theta_new = np.linspace(0, 2*np.pi, 100)
path = cs(theta_new)

plt.plot(points[:, 0], points[:, 1], 'o')
plt.plot(path[:, 0], path[:, 1], '-')
plt.axis('equal')
plt.show()
```

### Velocity Constrained Path

```python
# Start and end with zero velocity
x = np.array([0, 1, 2, 3])
y = np.array([0, 1, 0.5, 0])

# Zero velocity at start and end
cs = CubicSpline(x, y, bc_type='clamped')

# Or specify exact velocities
cs = CubicSpline(x, y, bc_type=((1, 0.0), (1, -0.5)))

x_new = np.linspace(0, 3, 100)
y_new = cs(x_new)
vel = cs(x_new, 1)

# Verify zero velocity at endpoints
print(f"Velocity at start: {vel[0]:.6f}")
print(f"Velocity at end: {vel[-1]:.6f}")
```

### ROS2 Integration Example

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import CubicSpline
import numpy as np

class PathInterpolator(Node):
    def __init__(self):
        super().__init__('path_interpolator')
        
        self.subscription = self.create_subscription(
            Path,
            'waypoints',
            self.waypoint_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Path,
            'smooth_path',
            10
        )
    
    def waypoint_callback(self, msg):
        # Extract waypoints
        waypoints = []
        for pose in msg.poses:
            waypoints.append([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            ])
        
        waypoints = np.array(waypoints)
        
        if len(waypoints) < 2:
            return
        
        # Create parameter (cumulative distance)
        diffs = np.diff(waypoints, axis=0)
        distances = np.sqrt(np.sum(diffs**2, axis=1))
        t = np.concatenate(([0], np.cumsum(distances)))
        
        # Create cubic splines
        cs_x = CubicSpline(t, waypoints[:, 0])
        cs_y = CubicSpline(t, waypoints[:, 1])
        cs_z = CubicSpline(t, waypoints[:, 2])
        
        # Generate smooth path
        num_points = 100
        t_new = np.linspace(0, t[-1], num_points)
        
        smooth_path = Path()
        smooth_path.header = msg.header
        
        for i in range(num_points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(cs_x(t_new[i]))
            pose.pose.position.y = float(cs_y(t_new[i]))
            pose.pose.position.z = float(cs_z(t_new[i]))
            
            # Calculate orientation from velocity direction
            if i < num_points - 1:
                dx = cs_x(t_new[i], 1)
                dy = cs_y(t_new[i], 1)
                yaw = np.arctan2(dy, dx)
                
                # Convert to quaternion (simplified, z-axis rotation only)
                pose.pose.orientation.z = np.sin(yaw / 2)
                pose.pose.orientation.w = np.cos(yaw / 2)
            
            smooth_path.poses.append(pose)
        
        self.publisher.publish(smooth_path)
```

### Time-Optimal Path Parametrization

```python
def create_time_optimal_trajectory(waypoints, max_velocity, max_acceleration):
    """
    Create a cubic spline trajectory with time parametrization
    that respects velocity and acceleration constraints.
    """
    # Calculate path length
    diffs = np.diff(waypoints, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    
    # Simple time allocation based on constraints
    times = [0]
    for length in segment_lengths:
        # Time for this segment (simplified)
        t_accel = max_velocity / max_acceleration
        d_accel = 0.5 * max_acceleration * t_accel**2
        
        if length < 2 * d_accel:
            # Triangular velocity profile
            t_segment = 2 * np.sqrt(length / max_acceleration)
        else:
            # Trapezoidal velocity profile
            t_segment = 2 * t_accel + (length - 2*d_accel) / max_velocity
        
        times.append(times[-1] + t_segment)
    
    times = np.array(times)
    
    # Create spline with time parameter
    cs_x = CubicSpline(times, waypoints[:, 0])
    cs_y = CubicSpline(times, waypoints[:, 1])
    
    return cs_x, cs_y, times[-1]

# Usage
waypoints = np.array([[0, 0], [1, 1], [2, 0.5], [3, 2]])
cs_x, cs_y, total_time = create_time_optimal_trajectory(
    waypoints,
    max_velocity=1.0,
    max_acceleration=0.5
)

# Sample at specific frequency
dt = 0.01  # 100 Hz
t = np.arange(0, total_time, dt)
trajectory = np.column_stack([cs_x(t), cs_y(t)])
```

## Comparison with Other Methods

### CubicSpline vs Linear Interpolation

```python
from scipy.interpolate import interp1d

x = np.array([0, 1, 2, 3])
y = np.array([0, 2, 1, 3])

# Linear interpolation (C0 - continuous but not smooth)
linear = interp1d(x, y, kind='linear')

# Cubic spline (C2 - smooth with continuous 2nd derivative)
cubic = CubicSpline(x, y)

x_new = np.linspace(0, 3, 100)

# Linear has discontinuous derivatives at waypoints
# Cubic has smooth derivatives everywhere
```

### CubicSpline vs Bezier Curves

- **CubicSpline**: Passes through all waypoints, easier to control
- **Bezier**: Uses control points, doesn't necessarily pass through waypoints
- **Use CubicSpline** when exact waypoint passage is required
- **Use Bezier** when shape control is more important than exact points

## Performance Optimization

### Pre-compute for Repeated Evaluation

```python
# Create spline once
cs = CubicSpline(x, y)

# Evaluate many times (fast)
for t in time_values:
    position = cs(t)
    velocity = cs(t, 1)
```

### Batch Evaluation

```python
# Vectorized evaluation (faster)
t_array = np.linspace(0, 10, 1000)
positions = cs(t_array)  # All at once

# vs loop (slower)
# positions = [cs(t) for t in t_array]
```

## Common Issues and Solutions

### Issue: "x must be strictly increasing"

```python
# Bad
x = np.array([0, 1, 1, 2])  # Duplicate value!

# Good
x = np.array([0, 1, 1.001, 2])  # Slightly offset

# Or use parametric form
t = np.arange(len(points))  # Always increasing
```

### Issue: Oscillations between waypoints

```python
# Cause: Default 'not-a-knot' boundary condition
# Solution: Use 'natural' or 'clamped' boundary conditions

cs = CubicSpline(x, y, bc_type='natural')
```

### Issue: Path doesn't close smoothly

```python
# Ensure first and last points are identical
waypoints = np.vstack([waypoints, waypoints[0]])

# Use periodic boundary condition
cs = CubicSpline(t, waypoints, bc_type='periodic')
```

### Issue: Sharp turns create high velocities

```python
# Solution: Use custom derivative constraints
# Set lower velocities at sharp turns

# Or: Adjust time parametrization to slow down at turns
curvature = calculate_curvature(waypoints)
time_allocation = adjust_for_curvature(curvature)
```

## Best Practices

1. **Use appropriate parametrization**: Time, arc-length, or custom based on application
2. **Set boundary conditions**: Choose based on physical constraints
3. **Validate smoothness**: Check velocity and acceleration profiles
4. **Handle edge cases**: Minimum 2 waypoints required
5. **Consider computational cost**: Pre-compute when possible
6. **Test extrapolation**: Be aware of behavior outside waypoint range
7. **Verify continuity**: Ensure C2 continuity for smooth robot motion
8. **Scale appropriately**: Normalize coordinates if they differ by orders of magnitude

## References

- [SciPy Documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html)
- Carl de Boor, "A Practical Guide to Splines", Springer-Verlag, 1978
- [Cubic Spline Interpolation on Wikiversity](https://en.wikiversity.org/wiki/Cubic_Spline_Interpolation)
