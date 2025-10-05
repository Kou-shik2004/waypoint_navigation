# **ROS2 Trajectory Navigation \- Project Context**

## **Assignment Overview**

**Objective: Implement autonomous trajectory navigation for TurtleBot3 Burger robot in Gazebo simulation using ROS2 Humble.**

**Deliverables:**

1. **Smooth trajectory generation from discrete waypoints**  
2. **Time-stamped trajectory with arc-length parameterization**  
3. **Pure pursuit controller for differential drive tracking**  
4. **Simulation demonstration with quantitative performance metrics**  
5. **Bonus (if time permits): Obstacle detection/avoidance, trapezoidal velocity profiles**

**Time Budget: \~17 hours core implementation \+ extras if time permits**

**Grading Breakdown (100 points total):**

* **Code Quality: 35 points (Trajectory Generation: 15, Controller: 20\)**  
* **Architecture & Comments: 10 points**  
* **Simulation: 20 points**  
* **Testability & QA: 20 points ← High priority**  
* **Documentation & Videos: 15 points**

## **What's Already Validated**

### **Offline Python Simulation (Complete)**

**You've already implemented and validated:**

* **Cubic spline path smoothing using SciPy**  
* **Arc-length parameterization for constant velocity**  
* **Pure pursuit controller with lookahead distance \= 0.30m**  
* **Cross-track error computation**

**Proven Performance (ideal conditions):**

* **RMS error: 0.016m (1.6cm)**  
* **Max error: 0.033m (3.3cm)**  
* **Test trajectory: 5-waypoint sinusoidal curve**

**Files Available:**

* **`trajectory_generation_and_tracking.py` \- Complete offline implementation**  
* **`trajectory_constant_velocity.csv` \- Generated trajectory data**  
* **Validation plots showing excellent tracking performance**

### **Expected Gazebo Performance**

**Due to sensor noise, timing jitter, and actuator delays:**

* **RMS error: 0.05 \- 0.15m (worse than offline, still acceptable)**  
* **Max error: 0.10 \- 0.30m (depending on trajectory curvature)**

## **System Architecture**

### **Three-Node Design (Modular & Testable)**

**┌─────────────────────┐**

**│ trajectory\_generator│ ──\[/trajectory\]──┐**

**│   (Runs once, stays │                  │**

**│   alive for late    │                  ▼**

**│   joiners)          │         ┌────────────────────┐**

**└─────────────────────┘         │ trajectory\_tracker │**

                                **│  (Pure Pursuit @   │**

        **┌───────────────────────│   20Hz control)    │──\[/cmd\_vel\]──▶ Robot**

        **│                       └────────────────────┘**

        **│                                  │**

  **\[/odom\]◀─────────────────────────────────┘**

        **│                                  │**

        **│                       ┌──────────▼─────────┐**

        **└───────────────────────│ trajectory\_monitor │**

                                **│  (Metrics @ 10Hz)  │**

                                **└────────────────────┘**

                                         **│**

                                  **\[/tracking\_error\]**

                                         **│**

                                    **CSV Export**

### **Node Responsibilities**

**trajectory\_generator.py:**

* **Input: Waypoints from YAML parameter file**  
* **Processing: Cubic spline → arc-length → time parameterization**  
* **Output: `nav_msgs/Path` on `/trajectory` topic (transient\_local QoS)**  
* **Visualization: Red waypoint markers, green smooth path**  
* **Lifecycle: Publishes once, then stays alive republishing every 1s**

**trajectory\_controller.py:**

* **Input: `/trajectory` (transient\_local QoS), `/odom` (default QoS)**  
* **Processing: Pure pursuit control law: ω \= 2v·sin(α)/L**  
* **Output: `geometry_msgs/Twist` on `/cmd_vel`**  
* **Control Loop: Deterministic 20Hz timer (50ms period)**  
* **State Management: Stores latest pose, finds lookahead point, computes velocities**

**trajectory\_monitor.py:**

* **Input: `/trajectory`, `/odom`**  
* **Processing: Compute cross-track error (minimum distance to reference)**  
* **Output: `std_msgs/Float64` on `/tracking_error` for live plotting**  
* **Logging: Running RMS, max, mean errors**  
* **Export: CSV on shutdown with time, x, y, error columns**

## **TurtleBot3 Burger Specifications**

### **Physical Constraints (HARD LIMITS)**

* **Maximum linear velocity: 0.22 m/s**  
* **Maximum angular velocity: 2.84 rad/s**  
* **Wheel radius: 0.033m**  
* **Wheelbase: 0.16m**  
* **Dimensions: 138mm × 178mm × 192mm (L×W×H)**

### **Dynamic Constraints**

* **Linear acceleration limit: \~2.5 m/s²**  
* **Linear deceleration limit: \~-2.5 m/s²**  
* **Angular acceleration limit: \~3.2 rad/s²**  
* **Angular deceleration limit: \~-3.2 rad/s²**

### **Differential Drive Kinematics**

* **Forward: v \= (v\_right \+ v\_left) / 2, ω \= (v\_right \- v\_left) / wheelbase**  
* **Non-holonomic: Cannot move sideways**  
* **All trajectories must respect these kinematic constraints**

## **Implementation Phases (STRICT ORDER)**

### **Phase 1: Core Functionality (Hours 1-14)**

**Milestone 1.1: Trajectory Generator (Hours 1-3)**

* **Create ROS2 package structure**  
* **Implement generator node with YAML parameter loading**  
* **Port validated spline algorithm from offline simulation**  
* **Test: `ros2 topic echo /trajectory` shows published Path**  
* **Test: RViz displays green path and red waypoint markers**

**Milestone 1.2: Controller Skeleton (Hours 4-5)**

* **Implement controller node structure**  
* **Create subscriptions (trajectory, odom)**  
* **Create cmd\_vel publisher**  
* **Implement 20Hz timer with safety checks**  
* **Test: Robot receives trajectory, stays still (zero velocity published)**

**Milestone 1.3: Control Logic (Hours 6-10)**

* **Implement lookahead point finder**  
* **Implement pure pursuit control law**  
* **Add velocity limiting and heading-based deceleration**  
* **Tune lookahead distance (start with 0.30m)**  
* **Test: Robot follows straight line trajectory**

**Milestone 1.4: Monitor Node (Hours 11-14)**

* **Implement odometry recording**  
* **Implement cross-track error computation**  
* **Add real-time error publishing**  
* **Implement CSV export on shutdown**  
* **Test: Monitor logs running statistics every 5 seconds**

**Phase 1 Acceptance Criteria:**

* **✅ Robot follows straight line trajectory from (0,0) to (4,0)**  
* **✅ RMS error \< 0.05m documented via console output**  
* **✅ CSV file exported on Ctrl+C with timestamp**

### **Phase 2: Testing & Validation (Hours 15-19)**

**Milestone 2.1: Test Trajectories (Hours 15-17)**

* **Create YAML configs for: straight, circle, S-curve**  
* **Run each trajectory in Gazebo**  
* **Record metrics: RMS error, max error, completion time**  
* **Generate plots: path overlay, error vs time**  
* **Record screen capture videos (Gazebo \+ RViz side-by-side)**

**Milestone 2.2: Error Handling (Hours 18-19)**

* **Test: Empty waypoints array (should error and exit gracefully)**  
* **Test: Single waypoint (should error or handle degenerately)**  
* **Test: Teleport robot mid-execution (should recover)**  
* **Test: Ctrl+C shutdown (all nodes exit cleanly, CSV saved)**  
* **Document all results in table format**

**Phase 2 Acceptance Criteria:**

* **✅ Three trajectory tests completed with documented metrics**  
* **✅ All metrics within expected thresholds (see below)**  
* **✅ Videos recorded for each test case**  
* **✅ Four error handling scenarios tested and documented**

### **Phase 3: Extensions (Hours 20+ if time permits)**

**Milestone 3.1: Trapezoidal Velocity (Optional)**

* **Modify generator to add acceleration/deceleration phases**  
* **Test on all three trajectories**  
* **Compare metrics: smoother motion expected**

**Milestone 3.2: Obstacle Avoidance (Optional)**

* **Create separate Gazebo world with obstacles**  
* **Implement simple potential field or stop-when-blocked**  
* **Test: Robot navigates around single static obstacle**  
* **Document separately from core tests**

**Phase 3 Entry Condition: Phases 1 & 2 MUST be 100% complete first**

## **Expected Test Results**

### **Test 1: Straight Line Trajectory**

**Config: `config/trajectory_straight.yaml`**

**waypoints: \[0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 4.0, 0.0\]**

**desired\_velocity: 0.20**

**Expected Metrics:**

* **RMS error: \< 0.05m**  
* **Max error: \< 0.10m**  
* **Completion time: \~20s (4m at 0.2 m/s)**

**Validation:**

* **Path overlay shows near-perfect straight line following**  
* **Error plot shows small random fluctuations (±2-3cm)**  
* **No systematic drift or oscillation**

### **Test 2: Circular Trajectory**

**Config: `config/trajectory_circle.yaml`**

**waypoints: \[20 points around circle, radius=1m, center=(1,1)\]**

**desired\_velocity: 0.20**

**Expected Metrics:**

* **RMS error: \< 0.15m**  
* **Max error: \< 0.25m**  
* **Completion time: \~31s (circumference 2πr ≈ 6.28m)**

**Validation:**

* **Actual path slightly inside reference circle (expected for pure pursuit)**  
* **Steady-state error 5-10cm (acceptable)**  
* **Error plot shows bounded oscillation, no divergence**

### **Test 3: S-Curve Trajectory**

**Config: `config/trajectory_curve.yaml`**

**waypoints: \[11 points, y \= 0.5\*sin(0.5\*x), x from 0 to 5\]**

**desired\_velocity: 0.15  \# Slower for curves**

**Expected Metrics:**

* **RMS error: \< 0.20m**  
* **Max error: \< 0.30m**  
* **Completion time: \~33s (depends on path length)**

**Validation:**

* **Larger errors at inflection points (direction changes)**  
* **Error decreases after each inflection**  
* **No stopping or erratic behavior**

## **Critical Design Decisions**

### **Why Pure Pursuit Without PID?**

**Pure pursuit inherently generates both linear and angular velocities from geometry. Adding PID on top creates 6 tuning parameters (3 linear \+ 3 angular) instead of 2 (velocity \+ lookahead). Your approach is simpler, fully explainable, and achieves comparable performance.**

### **Why Transient Local QoS?**

**Solves the late-joiner problem: if controller/monitor starts after generator publishes, they still receive the trajectory from the DDS cache. Critical for robust initialization.**

### **Why Separate Monitor Node?**

**Keeps metrics collection from interfering with control timing. Controller runs at exactly 20Hz for deterministic performance. Monitor can run slower (10Hz) and do expensive computations (find closest point) without affecting control.**

### **Why Arc-Length Parameterization?**

**Ensures uniform spacing of trajectory points regardless of waypoint distribution. Without it, spline parameter samples cluster near closely-spaced waypoints, creating uneven velocity.**

## **Key Differentiators from Reference Implementation**

**Your submission stands out by:**

1. **Proper arc-length parameterization (reference uses simple parameter spacing)**  
2. **Quantitative validation with documented metrics (reference has visual-only testing)**  
3. **Configurable system via YAML files (reference hardcodes waypoints)**  
4. **Simpler control with full explainability (reference adds unnecessary complexity)**  
5. **Professional metrics collection (reference lacks performance monitoring)**

## **File Structure**

**trajectory\_nav/                    \# Your package name**

**├── trajectory\_nav/**

**│   ├── \_\_init\_\_.py**

**│   ├── trajectory\_generator.py   \# \~150 lines**

**│   ├── trajectory\_controller.py  \# \~200 lines**

**│   └── trajectory\_monitor.py     \# \~150 lines**

**├── launch/**

**│   └── trajectory\_system.launch.py**

**├── config/**

**│   ├── trajectory\_straight.yaml**

**│   ├── trajectory\_circle.yaml**

**│   └── trajectory\_curve.yaml**

**├── rviz/**

**│   └── trajectory\_config.rviz**

**├── package.xml                    \# Dependencies listed**

**├── setup.py                       \# Entry points configured**

**├── setup.cfg**

**└── README.md                      \# Run instructions**

**Total: \~500 lines of Python \+ 3 config files \+ documentation**

## **Dependencies (package.xml)**

**\<depend\>rclpy\</depend\>**

**\<depend\>nav\_msgs\</depend\>**

**\<depend\>geometry\_msgs\</depend\>**

**\<depend\>sensor\_msgs\</depend\>**

**\<depend\>std\_msgs\</depend\>**

**\<depend\>visualization\_msgs\</depend\>**

**\<depend\>tf2\_ros\</depend\>**

**\<\!-- Python packages (install separately) \--\>**

**\<\!-- numpy, scipy, matplotlib (for offline validation) \--\>**

## **Common Debugging Commands**

**\# Check trajectory is publishing**

**ros2 topic echo /trajectory**

**\# Check odometry is available**

**ros2 topic echo /odom**

**\# Monitor tracking error live**

**ros2 topic echo /tracking\_error**

**\# List all active parameters**

**ros2 param list**

**\# Record complete test run**

**ros2 bag record /trajectory /odom /cmd\_vel /tracking\_error**

**\# Visualize in RViz**

**rviz2 \-d trajectory\_nav/rviz/trajectory\_config.rviz**

## **Success Metrics**

**Your implementation is complete when:**

* **✅ All three test trajectories run without crashes**  
* **✅ Metrics documented and within expected ranges**  
* **✅ Videos recorded showing Gazebo \+ RViz**  
* **✅ CSV files generated with error data**  
* **✅ Plots created (path overlay \+ error vs time)**  
* **✅ Error handling tests passed**  
* **✅ README explains how to run each test**  
* **✅ Code is clean, commented, and follows ROS2 conventions**

## **Reference Documents**

* **Detailed Pseudocode: See artifact "ROS2 Trajectory System \- Complete Pseudocode Blueprint"**  
* **Offline Validation: Results in uploaded PDF showing 0.016m RMS error**  
* **Assignment Requirements: See ros2\_implementation.md for full context**  
* **Cursor Rules: See `.cursor/rules/ros2-navigation.mdc` for coding standards**

## **Development Philosophy**

**Incremental: One node at a time, test before moving on Validated: Use ROS2 CLI tools to verify each component Quantitative: Measure everything, document with numbers Professional: Configuration over hardcoding, errors over crashes**

**Remember: The goal is a working, tested, documented system \- not the most complex one.**

