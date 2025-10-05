# ROS2 Reference Documentation

This folder contains comprehensive ROS2 implementation guides and reference materials. These documents should be consulted **before** implementing any ROS2 patterns in the trajectory navigation system.

## 📚 Available Documentation

### Core ROS2 Concepts

| File | Topics Covered | When to Use |
|------|---------------|-------------|
| **[ros2-parameters.md](ros2-parameters.md)** | • declare_parameter()<br>• get_parameter()<br>• YAML config loading<br>• Parameter validation | Before loading any configuration or declaring node parameters |
| **[ros2-quality-of-service.md](ros2-quality-of-service.md)** | • QoS policies (reliability, durability)<br>• TRANSIENT_LOCAL for late joiners<br>• History and depth settings<br>• QoS compatibility | Before creating publishers/subscribers with specific QoS needs |
| **[ros2-launch-files.md](ros2-launch-files.md)** | • Launch file structure<br>• Node configuration<br>• TimerAction for delays<br>• Parameter passing<br>• Including other launches | Before writing or modifying launch files |

### Messages and Data Types

| File | Topics Covered | When to Use |
|------|---------------|-------------|
| **[ros2-messages.md](ros2-messages.md)** | • nav_msgs (Path, Odometry)<br>• geometry_msgs (Twist, PoseStamped)<br>• sensor_msgs<br>• std_msgs<br>• Message field meanings | Before working with ROS2 messages |

### Transforms and Frames

| File | Topics Covered | When to Use |
|------|---------------|-------------|
| **[ros2-transforms.md](ros2-transforms.md)** | • tf2_ros library<br>• Coordinate frames<br>• Quaternion conversions<br>• Transform lookups | Before working with coordinate frames or rotations |

### Visualization

| File | Topics Covered | When to Use |
|------|---------------|-------------|
| **[ros2-visualization.md](ros2-visualization.md)** | • RViz markers<br>• Path visualization<br>• Marker types and properties<br>• Color coding | Before adding RViz visualization |

### Mathematical Operations

| File | Topics Covered | When to Use |
|------|---------------|-------------|
| **[ros2-interpolation.md](ros2-interpolation.md)** | • SciPy CubicSpline<br>• Arc-length parameterization<br>• Derivative computation<br>• Boundary conditions | Before implementing trajectory generation or path smoothing |

## 🎯 Quick Navigation by Task

### Task: Implement Trajectory Generator
**Read these docs in order:**
1. `ros2-interpolation.md` - For CubicSpline usage
2. `ros2-parameters.md` - For loading waypoints from YAML
3. `ros2-quality-of-service.md` - For TRANSIENT_LOCAL QoS setup
4. `ros2-messages.md` - For nav_msgs/Path structure
5. `ros2-visualization.md` - For RViz markers

### Task: Implement Controller
**Read these docs in order:**
1. `ros2-parameters.md` - For loading control parameters
2. `ros2-quality-of-service.md` - For subscribing with correct QoS
3. `ros2-messages.md` - For Odometry and Twist messages
4. `ros2-transforms.md` - For quaternion to yaw conversion

### Task: Implement Monitor
**Read these docs in order:**
1. `ros2-quality-of-service.md` - For trajectory subscription
2. `ros2-messages.md` - For reading Path and Odometry
3. `ros2-parameters.md` - For CSV output configuration

### Task: Create Launch File
**Read these docs in order:**
1. `ros2-launch-files.md` - For launch file structure and TimerAction
2. `ros2-parameters.md` - For passing parameters to nodes

## 💡 Best Practices

### Before Writing Code
1. ✅ Identify which documentation files are relevant to your task
2. ✅ Read the specific sections you need
3. ✅ Look for example code snippets in the docs
4. ✅ Check for "common pitfalls" sections
5. ✅ Use the exact patterns shown in the documentation

### During Implementation
- Reference documentation when uncertain about syntax
- Copy import statements exactly as shown
- Follow the parameter declaration patterns
- Use the QoS profiles as documented

### After Implementation
- Verify your code matches the documented patterns
- Check that you haven't mixed different approaches
- Ensure all critical sections are covered (error handling, validation)

## 🔍 How AI Agents Should Use This Documentation

**Pattern 1: Before implementing any ROS2 feature**
```
1. Identify the feature (e.g., "publish trajectory with TRANSIENT_LOCAL QoS")
2. Find relevant doc (ros2-quality-of-service.md)
3. Read the specific section (TRANSIENT_LOCAL durability)
4. Copy the pattern into your implementation
5. Adapt to your specific use case
```

**Pattern 2: When debugging**
```
1. Identify the problem area (e.g., "subscriber not receiving messages")
2. Find relevant doc (ros2-quality-of-service.md)
3. Check compatibility rules
4. Verify QoS profiles match
5. Apply the fix
```

**Pattern 3: When extending functionality**
```
1. Review existing implementation
2. Find relevant doc for new feature
3. Follow the same patterns as existing code
4. Maintain consistency across codebase
```

## 📝 Documentation Maintenance

When adding new ROS2 features to the project:
1. Check if relevant documentation exists
2. If missing, add new documentation file following this structure
3. Update this README.md index
4. Update references in CLAUDE.md and ros2-navigation.mdc

## 🚀 Current Project Context

This documentation supports the **ROS2 Trajectory Navigation System** for TurtleBot3 Burger:
- **Phase 1**: Core functionality (trajectory generation, control, monitoring)
- **Phase 2**: Testing and validation
- **Phase 3**: Extensions (optional)

All implementations should reference these docs to ensure consistency with ROS2 best practices.

---

**Last Updated**: Phase 1 Implementation Complete
**Total Docs**: 7 files covering all essential ROS2 topics
**Status**: ✅ Indexed and ready for AI agent reference

