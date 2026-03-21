# Robot Arm Teleoperation via iOS AR

## Overview

Teleoperate a UR10e robot arm using an iPhone as a 6-DOF input device. The phone's pose (tracked via ARKit) drives the robot through two control modes: velocity mode (resolved-rate Jacobian control) and position mode (analytical closed-form IK). The system visualizes the robot in RViz 2 and provides real-time haptic/visual feedback for workspace limits and singularities.

## Architecture

```
┌─────────────┐   WebSocket    ┌──────────────┐   WebSocket    ┌───────────────────┐
│  iOS App    │ ──────────────▶│  FastAPI      │ ──────────────▶│  ROS 2            │
│  (Swift +   │ ◀──────────────│  Relay        │ ◀──────────────│  KinematicsNode   │
│   ARKit)    │   Feedback     │  Server       │   State+FB     │  + RViz + Rosbag  │
└─────────────┘                └──────────────┘                └───────────────────┘
```

### iOS App (`app/`)

- ARKit world tracking (`.gravity` alignment) for 6-DOF phone pose at ~60 fps
- Computes twist velocity (linear + angular) from frame-to-frame pose deltas
- Twist is rotated from ARKit world frame into the reference pose's local frame, so movements are always relative to the phone's orientation at session/reset time
- Configurable axis mapping (phone axes → robot axes) with sign flipping
- Two control modes selectable in settings: velocity and position
- Reset button snaps robot to Q_HOME and re-references the phone pose
- Sends `TeleopMessage` (twist + 4x4 transform + mode + optional command) over WebSocket
- Receives feedback (manipulability, feasibility, joint limit proximity) and displays via UI indicators + haptics
- Feedback display togglable in settings

### FastAPI Server (`server/`)

Pure WebSocket relay — no kinematics computation. Two endpoints:

- `/ws` — iOS app connection. Receives teleop messages, forwards to ROS clients, relays feedback back to iOS
- `/ws/ros` — ROS node connection. Receives state/feedback from ROS, updates dashboard
- `/dashboard` — Web debug dashboard with live charts (joint angles, manipulability, twist magnitude, joint limit proximity)
- `/ws/dashboard` — WebSocket for dashboard telemetry broadcast at ~10 Hz

Files: `main.py` (relay), `dashboard.py` (dashboard state + inline HTML)

### ROS 2 KinematicsNode (`ros/`)

All kinematics runs in `kinematics_node.py`. The node:

- Connects to the server via WebSocket (background thread) to receive twist/transform/commands
- Runs a 30 Hz timer callback that:
  - **Velocity mode**: Resolved-rate IK via damped least-squares Jacobian inverse
  - **Position mode**: Analytical closed-form IK (6 equations, all solutions enumerated), picks closest elbow-up solution with safety checks (no frames below ground)
- Publishes `JointState` on `/joint_states` for `robot_state_publisher` and RViz
- Sends state + feedback back to server over WebSocket
- Handles reset command (snaps to Q_HOME, clears twist and reference pose)
- Subscribes to `/cmd_vel` as a fallback twist input for testing without the server

The launch file (`visual.launch.py`) starts:
- `robot_state_publisher` (URDF → TF)
- `rviz2` (visualization)
- `kinematics_node` (IK + WebSocket bridge)
- `ros2 bag record` (auto-records `/joint_states` to `ros/data/` with datetime filenames)

## Control Modes

### Velocity Mode (Resolved-Rate)
```
q_dot = J^T (J J^T + lambda^2 I)^{-1} * twist
q += q_dot * dt
```
Adaptive damping: lambda increases as manipulability approaches zero (singularity avoidance).

### Position Mode (Analytical IK)
```
target_pose = reference_ee_pose @ relative_transform
solutions = analytical_ik(target_pose)  # all closed-form solutions
q = pick_closest_elbow_up(solutions, q_current)
```
- Reference EE pose captured on first position-mode frame (or after reset)
- Relative transform comes from the iOS app (phone movement since reference)
- Solution selection: prefer elbow-up (elbow frame z > 0), pick closest in joint space, fall back to closest overall if no elbow-up exists
- Safety check: all intermediate frames must have z > 0 (above ground)

## UR10e Parameters

### DH Parameters (Standard Convention)

| Joint | a [m]    | d [m]   | alpha [rad] |
|-------|----------|---------|-------------|
| 1     | 0        | 0.1807  | pi/2        |
| 2     | -0.6127  | 0       | 0           |
| 3     | -0.57155 | 0       | 0           |
| 4     | 0        | 0.17415 | pi/2        |
| 5     | 0        | 0.11985 | -pi/2       |
| 6     | 0        | 0.11655 | 0           |

### Joint Limits
- All joints: -2pi to 2pi
- Max velocity: pi rad/s (joints 1-3), 2*pi rad/s (joints 4-6)
- Home config (Q_HOME): [-pi/2, -pi/2, pi/2, -pi/2, -pi/2, 0]

## Message Protocol

### iOS → Server (TeleopMessage)
```json
{"vx": 0, "vy": 0, "vz": 0, "wx": 0, "wy": 0, "wz": 0,
 "transform": [[...4x4 row-major...]], "mode": "velocity|position",
 "command": "reset"}
```

### ROS → Server (State Payload)
```json
{"q": [...], "qdot": [...], "ee_pos": [x,y,z], "ee_dist": 0,
 "mu": 0, "lam": 0, "feedback": {...}, "dt": 0}
```

### Server → iOS (Feedback)
```json
{"manipulability": 0, "joint_limit_proximity": [...],
 "workspace_proximity": 0, "is_feasible": true}
```

## Tools

### Rosbag Visualization
```bash
python ros/src/visualize_joint_states.py <bag_directory> [--trim-wait]
```
- Plots joint positions and velocities vs time
- `--trim-wait`: removes initial idle period (starts 0.5s after first velocity spike)
- Saves plot as `joint_states.png` in the bag directory
- Uses `rosbags` (pip) — no ROS environment needed

## Project Structure

```
robot-arm-teleop/
├── app/                              # iOS app (Xcode project)
│   └── robot_arm_teleop/
│       └── robot_arm_teleop/
│           ├── robot_arm_teleopApp.swift   # Entry point
│           ├── ContentView.swift           # Main UI + reset/AR buttons
│           ├── TeleopManager.swift         # Coordinator (AR + WebSocket + haptics)
│           ├── ARSessionManager.swift      # ARKit world tracking + twist computation
│           ├── WebSocketManager.swift      # WebSocket client
│           ├── HapticsController.swift     # Haptic feedback from manipulability
│           ├── Models.swift                # TeleopMessage, FeedbackData, AxisMapping
│           └── SettingsView.swift          # Server URL, mode, axis mapping, feedback toggle
├── server/                           # FastAPI WebSocket relay
│   ├── main.py                       # Relay endpoints (/ws, /ws/ros)
│   └── dashboard.py                  # Debug dashboard state + HTML
├── ros/                              # ROS 2 Humble workspace
│   ├── src/
│   │   ├── robot_arm_teleop/         # ROS 2 Python package
│   │   │   ├── launch/visual.launch.py
│   │   │   └── robot_arm_teleop/kinematics_node.py
│   │   └── visualize_joint_states.py # Rosbag plotting script
│   └── data/                         # Recorded rosbags (gitignored)
├── CONTEXT.md
└── pyproject.toml
```

## Technical Details

- **ROS 2 distro**: Humble
- **Robot**: UR10e (visualization only, no real robot driver)
- **Communication**: WebSocket (JSON) — iOS ↔ Server ↔ ROS
- **iOS → Server rate**: ~60 Hz (ARKit frame rate)
- **ROS control loop**: ~30 Hz
- **Build**: `colcon build --symlink-install` in `ros/`
