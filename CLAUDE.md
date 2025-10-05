# CLAUDE.md - ROS2 Trajectory Navigation System

## Project Overview

### Purpose
Time-boxed assignment (17 hours core) implementing autonomous navigation for TurtleBot3 Burger using ROS2 Humble. The system generates smooth trajectories from discrete waypoints and tracks them using pure pursuit control with quantitative performance validation.

### Architecture (Three Independent Nodes)
```
trajectory_generator → [/trajectory] → trajectory_controller → [/cmd_vel] → TurtleBot3
                              ↓                    ↑
                    trajectory_monitor ← [/odom] ←┘
```

**Design Philosophy**: Incremental development, quantitative validation, professional configurability over complexity.

### Critical Constraints
- **Hard velocity limits**: 0.22 m/s linear, 2.84 rad/s angular (TurtleBot3 Burger physical limits)
- **Timing**: Controller runs at exactly 20Hz deterministic, not "as fast as possible"
- **QoS**: Trajectory topic uses TRANSIENT_LOCAL durability (late-joiner problem solution)
- **Coordinate frame**: All poses use 'odom' frame consistently

### Success Criteria
- Three test trajectories (straight, circle, S-curve) complete with documented metrics
- RMS tracking error < 0.05m (straight), < 0.15m (circle), < 0.20m (S-curve)
- CSV export on shutdown with time-series error data
- Error handling tests pass (empty waypoints, teleport, Ctrl+C)

---

## Important Files & Structure

### Core Implementation Files
```
trajectory_nav/                          # Main ROS2 package
├── trajectory_nav/
│   ├── trajectory_generator.py          # ~150 lines - Cubic spline smoothing
│   ├── trajectory_controller.py         # ~200 lines - Pure pursuit control
│   └── trajectory_monitor.py            # ~150 lines - Metrics collection
├── launch/
│   └── trajectory_system.launch.py      # Orchestrates all nodes with delays
├── config/
│   ├── trajectory_straight.yaml         # Test 1: Straight line
│   ├── trajectory_circle.yaml           # Test 2: Circular path
│   └── trajectory_curve.yaml            # Test 3: S-curve
├── rviz/
│   └── trajectory_config.rviz           # Visualization configuration
├── package.xml                          # ROS2 dependencies
├── setup.py                             # Python entry points
├── setup.cfg                            # Python package config
└── README.md                            # User-facing documentation
```

### Reference Files (DO NOT MODIFY)
```
project-context.md                       # Complete project specification
.cursor/rules/ros2-navigation.mdc        # Coding standards and patterns
```

### Role of Each File

**trajectory_generator.py**:
- Loads waypoints from YAML parameters (NOT hardcoded)
- Applies cubic spline interpolation with arc-length parameterization
- Publishes nav_msgs/Path with transient_local QoS
- Stays alive republishing every 1s for late joiners

**trajectory_controller.py**:
- Implements pure pursuit: ω = 2v·sin(α)/L
- Decoupled callbacks: odom stores pose, timer runs control at 20Hz
- Applies TurtleBot3 velocity limits as hard constraints
- Publishes geometry_msgs/Twist to /cmd_vel

**trajectory_monitor.py**:
- Computes cross-track error (minimum distance to reference)
- Publishes std_msgs/Float64 to /tracking_error for live plotting
- Exports CSV on shutdown: trajectory_metrics_YYYYMMDD_HHMMSS.csv
- Logs final statistics: RMS error, max error, completion time

**trajectory_system.launch.py**:
- Launches Gazebo (0s delay)
- Launches generator (2s delay - let Gazebo initialize)
- Launches controller and monitor (3s delay - ensure trajectory published)
- Optionally launches RViz with saved configuration

**YAML config files**:
- Define waypoints as flat list: [x0, y0, x1, y1, ...]
- Set num_samples, desired_velocity, frame_id
- Allow testing different trajectories without recompiling

---

## Key Commands

### Environment Setup
```bash
# Set TurtleBot3 model (REQUIRED before every launch)
export TURTLEBOT3_MODEL=burger

# Source ROS2 workspace
source ~/ros2_ws/install/setup.bash
```

### Build Commands
```bash
# Initial package creation
cd ~/ros2_ws/src
ros2 pkg create trajectory_nav --build-type ament_python \
  --dependencies rclpy nav_msgs geometry_msgs std_msgs visualization_msgs

# Build package
cd ~/ros2_ws
colcon build --packages-select trajectory_nav

# Build with verbose output (debugging)
colcon build --packages-select trajectory_nav --event-handlers console_direct+
```

### Launch Commands
```bash
# Launch full system with straight line trajectory
ros2 launch trajectory_nav trajectory_system.launch.py config:=straight

# Launch with circle trajectory
ros2 launch trajectory_nav trajectory_system.launch.py config:=circle

# Launch with S-curve trajectory
ros2 launch trajectory_nav trajectory_system.launch.py config:=curve

# Launch without RViz (headless)
ros2 launch trajectory_nav trajectory_system.launch.py config:=straight use_rviz:=false
```

### Testing & Debugging Commands
```bash
# Verify trajectory is publishing
ros2 topic echo /trajectory --once

# Monitor tracking error live
ros2 topic echo /tracking_error

# Check controller is publishing velocity commands
ros2 topic echo /cmd_vel

# List all parameters for a node
ros2 param list /trajectory_generator

# Get specific parameter value
ros2 param get /trajectory_generator waypoints

# Record complete test run
ros2 bag record -o test_straight /trajectory /odom /cmd_vel /tracking_error
```

### Analysis Commands
```bash
# Plot tracking error in real-time (requires rqt_plot)
ros2 run rqt_plot rqt_plot /tracking_error/data

# Use plotjuggler for more advanced visualization
ros2 run plotjuggler plotjuggler

# Replay recorded bag file
ros2 bag play test_straight
```

---

## Coding Guidelines

### ROS2 Python Patterns

**Node Initialization Order** (STRICT):
```python
def __init__(self):
    super().__init__('node_name')
    
    # 1. Declare parameters with defaults
    self.declare_parameter('param', default_value)
    
    # 2. Retrieve and validate immediately
    self.param = self.get_parameter('param').value
    if self.param is None or self.param <= 0:
        self.get_logger().error('Invalid param')
        raise ValueError('Validation failed')
    
    # 3. Create QoS profiles (before publishers/subscribers)
    qos = QoSProfile(...)
    
    # 4. Create publishers
    # 5. Create subscribers
    # 6. Create timers
    # 7. Log initialization success
```

**Timer-Based Control** (NOT spin loops):
```python
# CORRECT: Deterministic timing
self.timer = self.create_timer(0.05, self.control_loop)  # Exactly 20Hz

# WRONG: Variable timing
while rclpy.ok():
    rclpy.spin_once(self)
    self.control_loop()
```

**Error Handling Strategy**:
```python
# Check preconditions, return safe default
if self.trajectory is None:
    self.publish_zero_velocity()
    return

# Wrap risky operations
try:
    result = math.atan2(dy, dx)
except Exception as e:
    self.get_logger().error(f'Computation failed: {e}')
    return 0.0, 0.0  # Safe fallback
```

### Mathematical Conventions

**Angle Normalization** (CRITICAL):
```python
def normalize_angle(self, angle):
    """Always normalize to [-π, π] before control"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
```

**Quaternion to Yaw** (standard 2D formula):
```python
siny_cosp = 2.0 * (qw * qz + qx * qy)
cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
yaw = math.atan2(siny_cosp, cosy_cosp)
```

**Velocity Limiting** (hard constraints):
```python
linear_vel = max(-0.22, min(0.22, linear_vel))   # TurtleBot3 limit
angular_vel = max(-2.84, min(2.84, angular_vel))  # TurtleBot3 limit
```

### Code Style
- Use 4 spaces for indentation (not tabs)
- Import order: standard library, third-party (numpy, scipy), ROS2 packages
- Docstrings for public methods: """Brief description. Returns: type"""
- Type hints optional but encouraged for clarity
- Variable names: descriptive (e.g., `lookahead_distance` not `L`)
- Constants: UPPER_SNAKE_CASE at module level
- Private methods: prefix with underscore `_method_name`

---

## Onboarding Instructions

### For AI Agents

**Before making ANY code changes**:
1. Read `project-context.md` to understand phase priorities
2. Check `.cursor/rules/ros2-navigation.mdc` for coding standards
3. **Consult `documentation/` folder for ROS2 implementation patterns** (see below)
4. Verify which phase we're in (core → testing → extensions)
5. Confirm the change aligns with current phase goals

**ROS2 Reference Documentation (documentation/ folder)**:
The `documentation/` directory contains authoritative ROS2 guides. **Always check these before implementing**:
- `documentation/ros2-interpolation.md` - When using CubicSpline or computing derivatives
- `documentation/ros2-quality-of-service.md` - When setting up publishers/subscribers with QoS
- `documentation/ros2-parameters.md` - When declaring or loading parameters from YAML
- `documentation/ros2-launch-files.md` - When writing or modifying launch files
- `documentation/ros2-messages.md` - When working with Path, Twist, Odometry messages
- `documentation/ros2-transforms.md` - When dealing with coordinate frames or quaternions
- `documentation/ros2-visualization.md` - When adding RViz markers or visualization

**When implementing a node**:
1. Identify which documentation/ files are relevant (e.g., QoS + parameters + messages)
2. Read those sections to understand the correct patterns
3. Reference the detailed pseudocode in artifacts
4. Follow the initialization order strictly
5. Test incrementally using `ros2 topic echo` commands
6. Don't move to next component until current one works

**When debugging**:
1. Check QoS profiles match between publisher and subscribers
2. Verify all nodes use `use_sim_time: True`
3. Confirm frame_id is 'odom' for all poses
4. Check velocity limits aren't violated
5. Look for angle discontinuities (forgot to normalize?)

### For Human Developers

**Initial Setup**:
```bash
# Install ROS2 Humble (Ubuntu 22.04)
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Install Python dependencies
pip3 install numpy scipy matplotlib

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone or create trajectory_nav package here

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**Development Workflow**:
1. Make changes to Python files in `trajectory_nav/trajectory_nav/`
2. Rebuild: `colcon build --packages-select trajectory_nav`
3. Source: `source install/setup.bash`
4. Test single node: `ros2 run trajectory_nav trajectory_generator`
5. Test full system: `ros2 launch trajectory_nav trajectory_system.launch.py`

**Testing Workflow**:
1. Launch system with test config
2. Let it run to completion (robot stops moving)
3. Check terminal output for final metrics
4. Find CSV file: `trajectory_metrics_YYYYMMDD_HHMMSS.csv`
5. Generate plots from CSV using provided Python scripts
6. Record screen capture showing Gazebo + RViz

---

## Agent Behaviors and Permissions

### ALLOWED Operations

**Automatic code modifications**:
- Fix syntax errors, import issues, or typos
- Add error handling following the established patterns
- Improve code comments and docstrings
- Refactor for clarity while maintaining functionality
- Add logging statements for debugging
- Adjust parameter values within reasonable ranges

**File creation**:
- Create missing YAML config files with computed waypoints
- Generate test scripts for validation
- Create plotting scripts for CSV data visualization
- Add launch file variants for different test scenarios

**Documentation updates**:
- Update README with new instructions or findings
- Add inline comments explaining complex logic
- Document discovered bugs or issues
- Update this CLAUDE.md with new learnings

### RESTRICTED Operations (Ask First)

**Do NOT automatically**:
- Change the three-node architecture (generator, controller, monitor)
- Switch from pure pursuit to a different control algorithm
- Add dependencies to package.xml without discussion
- Modify TurtleBot3 velocity limits (0.22 m/s, 2.84 rad/s)
- Change QoS profiles (especially trajectory topic's transient_local)
- Skip phases (must complete Phase 1 before Phase 2)
- Implement obstacle avoidance before core is complete

**Require explicit approval for**:
- Adding new ROS2 packages as dependencies
- Changing the control loop frequency from 20Hz
- Modifying the core mathematical algorithms (spline, pure pursuit)
- Restructuring the package layout
- Adding new nodes beyond the three core ones
- Changing coordinate frames from 'odom'

### Safety Guardrails

**Before executing any change**:
1. Check if it affects core functionality (generator, controller, monitor)
2. Verify it doesn't violate TurtleBot3 physical constraints
3. Ensure it maintains backward compatibility with YAML configs
4. Confirm it doesn't break the phase-based implementation order

**If unsure whether a change is allowed**:
- Describe the proposed change and ask for approval
- Explain the reasoning and expected impact
- Suggest alternatives if available

---

## Known Issues or Troubleshooting

### Common Problems and Solutions

**Problem**: "No module named 'trajectory_nav'"
```bash
# Solution: Rebuild and source
cd ~/ros2_ws
colcon build --packages-select trajectory_nav
source install/setup.bash
```

**Problem**: Robot doesn't move, no errors logged
```bash
# Check if trajectory was received
ros2 topic echo /trajectory --once

# Check if odometry is publishing
ros2 topic echo /odom --once

# Check if controller is publishing commands
ros2 topic echo /cmd_vel

# Common cause: QoS mismatch - verify transient_local on both ends
```

**Problem**: "ValueError: Insufficient waypoints"
```bash
# YAML file must have at least 4 values (2 waypoints × 2 coordinates)
# Check YAML syntax: waypoints: [x0, y0, x1, y1, ...]
```

**Problem**: Robot oscillates wildly
```bash
# Likely causes:
# 1. Lookahead distance too small - increase from 0.30 to 0.50
# 2. Forgot to normalize angle - check normalize_angle() is called
# 3. Velocity limits not applied - verify clamping to 0.22 m/s
```

**Problem**: Trajectory not visible in RViz
```bash
# Check fixed frame is set to 'odom' in RViz
# Verify Path display is subscribed to /trajectory
# Check Marker display is subscribed to /trajectory_markers
# Ensure nodes launched in correct order (generator before RViz)
```

**Problem**: CSV file not saved on shutdown
```bash
# Monitor node must call on_shutdown() in finally block
# Check file permissions in current directory
# Verify CSV path parameter is valid

# Temporary fix: Run with explicit path
ros2 run trajectory_nav trajectory_monitor \
  --ros-args -p csv_output_path:=/home/user/metrics.csv
```

**Problem**: Robot doesn't reach goal
```bash
# Check goal_tolerance parameter (default 0.05m)
# Verify final waypoint is actually reachable
# Check if controller stops too early (is_at_goal logic)
# Look for "Goal reached!" message in logs
```

**Problem**: "TURTLEBOT3_MODEL is not set"
```bash
# MUST export before EVERY launch
export TURTLEBOT3_MODEL=burger

# Make permanent by adding to ~/.bashrc:
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### Performance Expectations

**If metrics are worse than expected**:
- RMS error > 0.05m on straight line: Check velocity limits, increase lookahead
- RMS error > 0.15m on circle: Normal for tight curves, acceptable
- RMS error > 0.30m on any trajectory: Check for bugs in control logic

**If robot behavior is unexpected**:
- Stops immediately: Check trajectory and odometry are both received
- Spins in place: Likely angle normalization bug
- Overshoots goal: Reduce velocity or increase goal_tolerance
- Jerky motion: Control loop not running at steady 20Hz

### Debug Logging Strategy

**Add temporary logging**:
```python
# In control_loop, every 10th iteration
if self.loop_count % 10 == 0:
    self.get_logger().info(
        f'Target: {target_index}/{total}, '
        f'Distance: {distance:.3f}m, '
        f'Alpha: {alpha:.2f}rad, '
        f'Omega: {omega:.2f}rad/s'
    )
```

**Enable debug level logging**:
```bash
ros2 run trajectory_nav trajectory_controller \
  --ros-args --log-level debug
```

### Integration with Reference Code

**Studying Shaurya's implementation**:
- Look at message construction patterns (standard ROS2)
- Check how he handles coordinate transformations
- Review his marker publishing for RViz
- DO NOT copy algorithmic implementations

**If stuck on a bug**:
- Compare your control flow to his (high-level structure)
- Check if you're missing error handling he has
- Verify your QoS settings match expected patterns
- But implement the solution yourself, don't copy code

---

## Development Status Tracking

**Phase 1: Core Functionality** (Target: Hours 1-14)
- [ ] Package setup complete
- [ ] Generator node: publishes trajectory
- [ ] Controller node: receives and tracks trajectory
- [ ] Monitor node: logs metrics and exports CSV
- [ ] Straight line test passes (RMS < 0.05m)

**Phase 2: Testing & Validation** (Target: Hours 15-19)
- [ ] Circle trajectory config created and tested
- [ ] S-curve trajectory config created and tested
- [ ] All three tests documented with metrics
- [ ] Error handling tests completed
- [ ] Videos recorded for all tests

**Phase 3: Extensions** (Target: Hours 20+ if time)
- [ ] Trapezoidal velocity profiles implemented
- [ ] Obstacle avoidance implemented
- [ ] Additional trajectories tested

**Completion Criteria**:
- All Phase 1 tasks complete before starting Phase 2
- All Phase 2 tasks complete before starting Phase 3
- Each phase validated before moving forward

---

## Quick Reference

**Most frequently needed commands**:
```bash
# Standard launch
export TURTLEBOT3_MODEL=burger
ros2 launch trajectory_nav trajectory_system.launch.py config:=straight

# Quick rebuild
cd ~/ros2_ws && colcon build --packages-select trajectory_nav && source install/setup.bash

# Check trajectory publishing
ros2 topic echo /trajectory --once

# Monitor error live
ros2 topic echo /tracking_error
```

**Most common parameters to tune**:
- `lookahead_distance`: 0.20 to 0.50 (start with 0.30)
- `desired_velocity`: 0.10 to 0.22 (start with 0.20)
- `goal_tolerance`: 0.03 to 0.10 (start with 0.05)
- `num_samples`: 150 to 300 (start with 200)

**Critical files to never modify**:
- `project-context.md` - Project specification
- `.cursor/rules/ros2-navigation.mdc` - Coding standards
- Validated offline simulation code - Reference only

**When in doubt**:
- Check project-context.md for requirements
- Check .cursor/rules/ for coding patterns
- Check pseudocode artifact for implementation details
- Ask before making architectural changes