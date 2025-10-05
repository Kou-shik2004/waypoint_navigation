# Trajectory Navigation System - ROS2 Humble

Autonomous trajectory navigation for TurtleBot3 Burger using smooth path generation and pure pursuit control.

## System Overview

**Architecture**: Three independent ROS2 nodes
- **trajectory_generator**: Generates smooth trajectories from waypoints using cubic spline interpolation
- **trajectory_controller**: Pure pursuit controller for trajectory tracking (20Hz control loop)
- **trajectory_monitor**: Performance monitoring with real-time error tracking and CSV export

## Quick Start

### Prerequisites
```bash
# Set TurtleBot3 model (REQUIRED before every launch)
export TURTLEBOT3_MODEL=burger

# Install dependencies
sudo apt install ros-humble-turtlebot3* python3-numpy python3-scipy
```

### Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_nav
source install/setup.bash
```

### Run System

**Test 1: Straight Line Trajectory**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch trajectory_nav trajectory_system.launch.py config:=straight
```

**Test 2: Circular Trajectory**
```bash
ros2 launch trajectory_nav trajectory_system.launch.py config:=circle
```

**Test 3: S-Curve Trajectory**
```bash
ros2 launch trajectory_nav trajectory_system.launch.py config:=curve
```

**Headless Mode (no RViz)**
```bash
ros2 launch trajectory_nav trajectory_system.launch.py config:=straight use_rviz:=false
```

### Stop System
Press `Ctrl+C` - the monitor will automatically export metrics to CSV before shutting down.

## Testing & Validation

### Expected Performance Metrics

| Trajectory | RMS Error | Max Error | Time  |
|-----------|-----------|-----------|-------|
| Straight  | < 0.05m   | < 0.10m   | ~20s  |
| Circle    | < 0.15m   | < 0.25m   | ~31s  |
| S-curve   | < 0.20m   | < 0.30m   | ~34s  |

### Debugging Commands

```bash
# Verify trajectory is publishing
ros2 topic echo /trajectory --once

# Monitor tracking error live
ros2 topic echo /tracking_error

# Check controller velocity commands
ros2 topic echo /cmd_vel

# List node parameters
ros2 param list /trajectory_generator
ros2 param list /trajectory_controller
```

### Output Files

After each run, metrics are saved to:
```
trajectory_metrics_YYYYMMDD_HHMMSS.csv
```

CSV columns: `time, x, y, error`

## Configuration

Edit YAML files in `config/` to customize trajectories:

**config/trajectory_straight.yaml**
- `waypoints`: Flat list of (x,y) coordinates `[x0, y0, x1, y1, ...]`
- `num_samples`: Number of interpolated points (default: 200)
- `desired_velocity`: Target velocity in m/s (max: 0.22 m/s)
- `lookahead_distance`: Pure pursuit lookahead (default: 0.30 m)
- `goal_tolerance`: Goal reached threshold (default: 0.05 m)

## Key Features

✅ **Smooth Trajectory Generation**
- Cubic spline interpolation with arc-length parameterization
- Ensures uniform point spacing regardless of waypoint distribution

✅ **Pure Pursuit Control**
- Geometric control law: ω = 2v·sin(α)/L
- Deterministic 20Hz control loop
- Hard velocity limits for TurtleBot3 safety

✅ **Quantitative Metrics**
- Real-time cross-track error computation
- RMS, max, and mean error statistics
- Automatic CSV export on shutdown

✅ **Professional Configuration**
- YAML-based waypoint definition (no hardcoded values)
- TRANSIENT_LOCAL QoS for late-joiner support
- Modular three-node architecture

## Technical Details

### TurtleBot3 Burger Constraints
- Max linear velocity: **0.22 m/s** (hard limit)
- Max angular velocity: **2.84 rad/s** (hard limit)
- Differential drive kinematics (non-holonomic)

### QoS Configuration
- `/trajectory`: TRANSIENT_LOCAL durability (late-joiner support)
- `/odom`, `/cmd_vel`, `/tracking_error`: Default QoS

### Control Loop Timing
- Controller: Exactly **20Hz** (50ms period)
- Monitor: **10Hz** (100ms period)
- Generator: Republishes every 1s for late joiners

## Troubleshooting

**Problem**: Robot doesn't move
```bash
# Check trajectory published
ros2 topic echo /trajectory --once

# Check odometry available
ros2 topic echo /odom --once

# Verify QoS compatibility
ros2 topic info /trajectory -v
```

**Problem**: "No module named 'trajectory_nav'"
```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_nav
source install/setup.bash
```

**Problem**: TURTLEBOT3_MODEL not set
```bash
export TURTLEBOT3_MODEL=burger
# Make permanent:
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

## Package Structure
```
trajectory_nav/
├── trajectory_nav/
│   ├── trajectory_generator.py      # Spline-based path generation
│   ├── trajectory_controller.py     # Pure pursuit controller
│   └── trajectory_monitor.py        # Metrics collection
├── launch/
│   └── trajectory_system.launch.py  # System orchestration
├── config/
│   ├── trajectory_straight.yaml     # Straight line test
│   ├── trajectory_circle.yaml       # Circular path test
│   └── trajectory_curve.yaml        # S-curve test
├── package.xml                      # ROS2 dependencies
├── setup.py                         # Python entry points
└── README.md                        # This file
```

## Development

**Phase 1**: Core Functionality ✅
- Trajectory generation with cubic splines
- Pure pursuit control implementation
- Performance monitoring and CSV export

**Phase 2**: Testing & Validation (Next)
- Run all three test trajectories
- Document metrics and generate plots
- Error handling validation

**Phase 3**: Extensions (Future)
- Trapezoidal velocity profiles
- Obstacle avoidance

## License

MIT License

## Author

Student - ROS2 Trajectory Navigation Project


