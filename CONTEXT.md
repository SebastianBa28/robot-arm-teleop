# Robot Arm Teleoperation via iOS AR

## Overview

Teleoperate a UR10e robot arm using an iPhone as a 6-DOF input device. The phone's pose (tracked via ARKit) maps to the desired end-effector pose via resolved-rate (Jacobian-based) control. Since ARKit provides velocity-based pose tracking, we use the geometric Jacobian to map task-space velocities directly to joint velocities — no IK solver needed per frame. The system visualizes the robot in RViz 2 and provides real-time haptic/visual feedback for workspace limits and singularities.

## Architecture

```
┌─────────────┐   WebSocket    ┌──────────────┐   ROS 2 Topics   ┌───────────┐
│  iOS App    │ ──────────────▶│  FastAPI      │ ────────────────▶│  ROS 2    │
│  (Swift +   │ ◀──────────────│  Bridge       │                  │  Nodes    │
│   ARKit)    │   Feedback     │  (standalone) │                  │  + RViz   │
└─────────────┘                └──────────────┘                  └───────────┘
```

### iOS App (Swift)
- Uses ARKit to track phone pose and compute twist velocity (linear + angular velocity)
- Streams twist velocity over WebSocket to FastAPI bridge at configurable rate (~30 Hz default)
- Receives feedback from server: workspace limits, singularity warnings, joint limit warnings
- Displays feedback via visual indicators (color gradient green → yellow → red) and haptic feedback (vibration intensity)

### FastAPI Bridge (Python, standalone process)
- WebSocket server receiving twist velocity data from the iOS app
- Maintains current joint state q
- Computes geometric Jacobian J(q) from DH parameters each frame
- Performs resolved-rate control: q̇ = J⁻¹(q) · ẋ (damped least squares near singularities)
- Integrates: q = q + q̇ · dt
- Publishes joint state to ROS 2 topics
- Computes and sends feedback back to app over the same WebSocket:
  - Manipulability measure (singularity proximity) from sqrt(det(J·Jᵀ))
  - Joint limit proximity (per-joint percentage to limit)
  - Workspace boundary proximity
  - Binary feasibility (if resulting q violates limits → infeasible)

### ROS 2 Nodes (Jazzy)
- **Server node**: Subscribes to joint state from FastAPI bridge, publishes `JointState` messages
- **RViz visualization**: Displays UR10e model with current joint state (visualization only, no real robot)
- Launch file: `visual.launch.py`

## Kinematics: Jacobian-Based Resolved-Rate Control

### Why Jacobian Instead of IK
ARKit provides pose updates as velocity (frame-to-frame deltas). Since we already have the task-space velocity ẋ, we can use the Jacobian to compute joint velocities directly:

```
ẋ = J(q) · q̇   →   q̇ = J⁻¹(q) · ẋ
```

This avoids solving the full inverse kinematics problem every frame, and is computationally cheap (matrix operations on a 6x6 matrix).

### Control Loop

```
each frame (at ~30 Hz):
  1. Receive twist velocity ẋ = [vx, vy, vz, wx, wy, wz] from iOS app
  2. Compute forward kinematics chain T₀¹...T₀⁶ from current q using DH params
  3. Build geometric Jacobian J(q):
     For each joint i:
       z_{i-1} = T₀^{i-1}[0:3, 2]    (joint axis)
       o_{i-1} = T₀^{i-1}[0:3, 3]    (frame origin)
       J_i = [ z_{i-1} × (o₆ - o_{i-1}) ]   (linear, 3×1)
             [        z_{i-1}              ]   (angular, 3×1)
  4. Compute manipulability: μ = sqrt(det(J·Jᵀ))
  5. Compute damped inverse: q̇ = Jᵀ(J·Jᵀ + λ²I)⁻¹ · ẋ
     where λ adapts based on μ (higher damping near singularities)
  6. Clamp q̇ to joint velocity limits
  7. Integrate: q = q + q̇ · dt
  8. Check joint limits, clamp q
  9. Publish q as JointState
  10. Send feedback (μ, joint limit proximity, feasibility) to app
```

### UR10e DH Parameters (Denavit-Hartenberg)

| Joint | a [m]    | d [m]   | α [rad] | θ        |
|-------|----------|---------|---------|----------|
| 1     | 0        | 0.1807  | π/2     | q₁       |
| 2     | -0.6127  | 0       | 0       | q₂       |
| 3     | -0.57155 | 0       | 0       | q₃       |
| 4     | 0        | 0.17415 | π/2     | q₄       |
| 5     | 0        | 0.11985 | -π/2    | q₅       |
| 6     | 0        | 0.11655 | 0       | q₆       |

### UR10e Joint Limits

| Joint | Min [rad] | Max [rad] | Max velocity [rad/s] |
|-------|-----------|-----------|---------------------|
| 1-6   | -2π       | 2π        | ±π (joints 1-3), ±2π (joints 4-6) |

### Singularity Conditions
- **Shoulder**: Wrist center passes through joint 1 z-axis
- **Elbow**: Joints 2-3 fully extended (arm straight)
- **Wrist**: Joint 5 near 0 or π (joints 4 and 6 axes align)

All detected via manipulability measure μ — as μ → 0, damping λ increases and feedback is sent to app.

## Feedback System

| Condition | Detection Method | App Response |
|-----------|-----------------|--------------|
| Normal operation | μ > threshold, joints within limits | Green UI, no haptic |
| Approaching singularity | μ dropping toward threshold | Yellow UI + light haptic |
| Near singularity | μ < threshold | Red UI + strong haptic |
| Near joint limits | Any joint within 10% of limit | Yellow/red per-joint indicator + haptic |
| Infeasible (joint limit hit) | q clamped at limit | Red flash + strong vibration |

## Technical Details

- **ROS 2 distro**: Jazzy
- **Robot**: UR10e (Universal Robots)
- **Kinematics**: Geometric Jacobian from DH parameters (resolved-rate control)
- **Communication**: WebSocket (bidirectional — twist velocity upstream, feedback downstream)
- **Data format**: Twist velocity [vx, vy, vz, wx, wy, wz] from ARKit
- **Update rate**: ~30 Hz, configurable/togglable
- **Visualization**: RViz 2 only (no real robot driver)
- **Singularity handling**: Damped least squares with adaptive λ

## Project Structure

```
robot-arm-teleop/
├── app/                          # iOS app (Swift/SwiftUI + ARKit)
│   └── robot_arm_teleop/
├── ros/                          # ROS 2 workspace
│   └── src/
│       └── robot_arm_teleop/     # ROS 2 Python package
│           ├── launch/
│           ├── robot_arm_teleop/ # Python modules
│           └── test/
├── bridge/                       # FastAPI WebSocket bridge (to be created)
│   ├── main.py                   # FastAPI app + WebSocket endpoint
│   ├── kinematics.py             # DH params, FK, Jacobian, resolved-rate control
│   └── feedback.py               # Manipulability, joint limit checks
└── CONTEXT.md
```

## Constraints

- Deadline: mid-March 2026 (a few days)
- Class project: ME235A
- Keep it simple — minimal viable implementation first
