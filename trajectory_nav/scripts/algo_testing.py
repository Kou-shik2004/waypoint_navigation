"""
Trajectory Generation & Tracking System
========================================
Project: TurtleBot3 Navigation Assignment
Developer: S. Koushik
Platform: Python (NumPy + Matplotlib + SciPy)

Purpose: Offline validation of trajectory generation and tracking pipeline
         prior to ROS2 integration.

Pipeline Stages:
    1. Cubic Spline Path Smoothing
    2. Arc-Length Computation
    3. Time Parameterization (Constant Velocity)
    4. Pure Pursuit Controller Simulation
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import csv
import math


# =============================================================================
# CONFIGURATION & CONSTANTS
# =============================================================================

# Define waypoints for the trajectory
# Format: [(x0, y0), (x1, y1), ..., (xn, yn)]
waypoints = [
    (0.0, 0.0),
    (1.0, 0.5),
    (2.0, 0.0),
    (3.0, 1.0),
    (4.0, 0.0)
]

# Extract x and y coordinates from waypoints
x_coords, y_coords = zip(*waypoints)
x = np.array(list(x_coords))
y = np.array(list(y_coords))


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def t_cumul(x_list, y_list):
    """
    Compute cumulative chord-length parameterization.
    
    This creates a parameter t that represents cumulative distance along 
    straight line segments between consecutive points.
    
    Args:
        x_list: Array of x-coordinates
        y_list: Array of y-coordinates
    
    Returns:
        t: Array of cumulative distances, starting from 0
    """
    dx = np.diff(x_list)
    dy = np.diff(y_list)
    dist = np.sqrt(dx**2 + dy**2)
    t = np.insert(np.cumsum(dist), 0, 0.0)
    return t


# =============================================================================
# STAGE 1: CUBIC SPLINE PATH SMOOTHING
# =============================================================================

print("="*60)
print("STAGE 1: CUBIC SPLINE PATH SMOOTHING")
print("="*60)

# Compute chord-length parameter for original waypoints
t = t_cumul(x, y)

# Fit independent cubic splines for x(t) and y(t)
# This ensures C² continuity (smooth position, velocity, and acceleration)
cs_x = CubicSpline(t, x)
cs_y = CubicSpline(t, y)

# Generate dense samples along the spline curve
num_samples = 200  # Number of points to sample
t_dense = np.linspace(t[0], t[-1], num_samples)

# Evaluate splines at dense parameter values
x_smooth = cs_x(t_dense)
y_smooth = cs_y(t_dense)

# Visualize the smoothed path
plt.figure(figsize=(7, 5))
plt.plot(x, y, 'o--', label='Waypoints (straight lines)', color='tab:orange')
plt.plot(x_smooth, y_smooth, '-', label='Cubic Spline Curve', color='tab:blue')
plt.axis('equal')
plt.title('Cubic Spline Path Smoothing')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()


# =============================================================================
# STAGE 2: ARC-LENGTH COMPUTATION
# =============================================================================

print("\n" + "="*60)
print("STAGE 2: ARC-LENGTH COMPUTATION")
print("="*60)

# Compute arc length along the smoothed path
# This maps the spline parameter to actual physical distance
arc_length = t_cumul(x_smooth, y_smooth)
total_length = arc_length[-1]

print(f"Number of dense samples: {len(t_dense)}")
print(f"Total path length (approx): {total_length:.4f} m")

# Plot arc length vs. spline parameter
plt.figure(figsize=(6, 3.5))
plt.plot(t_dense, arc_length, '-')
plt.xlabel('Spline parameter t (chord-length units)')
plt.ylabel('Arc length s (m)')
plt.title('Arc length (s) vs spline parameter t')
plt.grid(True)
plt.show()


# =============================================================================
# STAGE 3: TIME PARAMETERIZATION (CONSTANT VELOCITY)
# =============================================================================

print("\n" + "="*60)
print("STAGE 3: TIME PARAMETERIZATION")
print("="*60)

# Define constant cruise velocity (TurtleBot3 safe speed)
v_const = 0.20  # m/s

# Compute time stamps: time = distance / velocity
time_stamps = arc_length / v_const
total_time = time_stamps[-1]

print(f"Total travel time at {v_const:.2f} m/s = {total_time:.2f} seconds")

# Create constant velocity profile
velocity = np.full_like(arc_length, v_const)

# Visualize velocity profile
plt.figure(figsize=(6, 3.5))
plt.plot(arc_length, velocity, '-', color='tab:green')
plt.xlabel('Arc length s (m)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity vs Arc length (Constant Velocity Profile)')
plt.grid(True)
plt.ylim(0, v_const * 1.2)
plt.show()

# Export trajectory to CSV file
csv_fname = 'trajectory_constant_velocity.csv'
with open(csv_fname, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y', 'arc_length_s', 'time_t'])
    for xi, yi, si, ti in zip(x_smooth, y_smooth, arc_length, time_stamps):
        writer.writerow([f"{xi:.6f}", f"{yi:.6f}", f"{si:.6f}", f"{ti:.6f}"])

print(f"✅ Saved constant-velocity trajectory CSV: {csv_fname}")
print(f"Each row = one point → [x, y, s, t]")


# =============================================================================
# STAGE 4: PURE PURSUIT CONTROLLER SIMULATION
# =============================================================================

print("\n" + "="*60)
print("STAGE 4: PURE PURSUIT CONTROLLER SIMULATION")
print("="*60)

# Controller parameters
CSV_PATH = "trajectory_constant_velocity.csv"  # Input trajectory file
LOOKAHEAD = 0.3      # Lookahead distance (m)
V_DES = 0.2          # Constant linear velocity (m/s)
DT = 0.05            # Simulation timestep (s) - 20 Hz
GOAL_TOL = 0.05      # Goal tolerance (m)
MAX_TIME = 30.0      # Max simulation time (s)
MAX_OMEGA = 2.0      # Clamp angular velocity (rad/s)


def normalize_angle(a):
    """
    Wrap angle to [-π, π] range.
    
    Args:
        a: Angle in radians
    
    Returns:
        Normalized angle in range [-π, π]
    """
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_trajectory(path):
    """
    Load trajectory file with columns [x, y, s, t].
    
    Args:
        path: Path to CSV file
    
    Returns:
        Tuple of (x, y, s, t) arrays
    """
    data = np.loadtxt(path, delimiter=',', skiprows=1)  # Skip header
    x, y, s, t = data[:, 0], data[:, 1], data[:, 2], data[:, 3]
    return x, y, s, t


def find_lookahead_index(x_traj, y_traj, xr, yr, start_idx, L):
    """
    Find the first trajectory point at least L meters ahead of the robot.
    
    This implements the lookahead point selection for Pure Pursuit control.
    
    Args:
        x_traj: Trajectory x-coordinates
        y_traj: Trajectory y-coordinates
        xr: Robot x-position
        yr: Robot y-position
        start_idx: Starting index to search from
        L: Lookahead distance
    
    Returns:
        Index of lookahead point (or last point if none found)
    """
    n = len(x_traj)
    for i in range(start_idx, n):
        dx, dy = x_traj[i] - xr, y_traj[i] - yr
        if math.hypot(dx, dy) >= L:
            return i
    return n - 1  # Fallback to last point


# Load reference trajectory
x_traj, y_traj, s_traj, t_traj = load_trajectory(CSV_PATH)
print(f"Loaded trajectory with {len(x_traj)} points, total length = {s_traj[-1]:.2f} m")

# Initialize robot state at first waypoint
xr, yr = x_traj[0], y_traj[0]
if len(x_traj) > 1:
    theta = math.atan2(y_traj[1] - y_traj[0], x_traj[1] - x_traj[0])
else:
    theta = 0.0

# Logging arrays for visualization
xs, ys, thetas, times, errors = [], [], [], [], []

# =============================================================================
# SIMULATION LOOP
# =============================================================================

time = 0.0
idx = 0  # Current trajectory index

while time < MAX_TIME:
    # 1. LOOKAHEAD POINT SELECTION
    # Find target point L meters ahead on the trajectory
    target_idx = find_lookahead_index(x_traj, y_traj, xr, yr, idx, LOOKAHEAD)
    idx = target_idx
    tx, ty = x_traj[target_idx], y_traj[target_idx]
    
    # 2. COMPUTE CONTROL (PURE PURSUIT)
    # Calculate desired heading toward lookahead point
    desired_heading = math.atan2(ty - yr, tx - xr)
    
    # Calculate heading error (alpha)
    alpha = normalize_angle(desired_heading - theta)
    
    # Pure Pursuit control law: ω = 2v·sin(α)/L
    omega = 2.0 * V_DES * math.sin(alpha) / LOOKAHEAD
    omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))  # Clamp to limits
    
    # 3. UPDATE ROBOT STATE (DIFFERENTIAL DRIVE KINEMATICS)
    # x_{k+1} = x_k + v·cos(θ)·Δt
    # y_{k+1} = y_k + v·sin(θ)·Δt
    # θ_{k+1} = θ_k + ω·Δt
    xr += V_DES * math.cos(theta) * DT
    yr += V_DES * math.sin(theta) * DT
    theta = normalize_angle(theta + omega * DT)
    
    # 4. COMPUTE CROSS-TRACK ERROR
    # Minimum distance from robot to reference trajectory
    cte = np.min(np.hypot(x_traj - xr, y_traj - yr))
    
    # 5. LOG DATA
    xs.append(xr)
    ys.append(yr)
    thetas.append(theta)
    times.append(time)
    errors.append(cte)
    
    # 6. CHECK GOAL CONDITION
    if math.hypot(xr - x_traj[-1], yr - y_traj[-1]) < GOAL_TOL:
        print(f"Reached goal in {time:.2f}s.")
        break
    
    # Increment time
    time += DT


# =============================================================================
# COMPUTE PERFORMANCE METRICS
# =============================================================================

errors = np.array(errors)
rms_err = math.sqrt(np.mean(errors ** 2))
max_err = np.max(errors)

print(f"RMS Cross-Track Error: {rms_err:.3f} m")
print(f"Max Cross-Track Error: {max_err:.3f} m")


# =============================================================================
# VISUALIZATION
# =============================================================================

# Plot 1: Trajectory tracking (path overlay)
plt.figure(figsize=(8, 5))
plt.plot(x_traj, y_traj, 'k--', label='Reference Trajectory', linewidth=2)
plt.plot(xs, ys, 'r-', label='Simulated Robot Path', linewidth=1.5)
plt.scatter(x_traj[0], y_traj[0], c='g', s=60, label='Start', zorder=5)
plt.scatter(x_traj[-1], y_traj[-1], c='b', s=60, label='Goal', zorder=5)
plt.axis('equal')
plt.title('Pure Pursuit Trajectory Tracking')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()
plt.grid(True)

# Plot 2: Cross-track error over time
plt.figure(figsize=(8, 3))
plt.plot(times, errors, 'b', linewidth=1)
plt.title(f'Cross-Track Error over Time (RMS={rms_err:.3f} m, Max={max_err:.3f} m)')
plt.xlabel('Time [s]')
plt.ylabel('Error [m]')
plt.grid(True)

plt.show()


# =============================================================================
# NOTES
# =============================================================================

"""
TUNING NOTES:
1. Ideal simulation — no noise, latency, or actuator dynamics.
2. Tune LOOKAHEAD distance:
   - Smaller → More responsive, may oscillate on sharp turns
   - Larger → Smoother tracking, may cut corners
3. Current performance: RMS ~0.016m shows excellent tracking accuracy

NEXT STEP:
Integrate this logic into ROS2 nodes:
- trajectory_generator: Publishes Path message
- trajectory_tracker: Subscribes to /odom and /trajectory, publishes /cmd_vel
- trajectory_monitor: Logs metrics and tracking error
"""