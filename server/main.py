"""FastAPI WebSocket server for UR10e teleoperation bridge."""

import json
import time

import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from kinematics import (
    Q_HOME,
    Q_MAX,
    Q_MIN,
    QDOT_MAX,
    adaptive_damping,
    compute_manipulability,
    forward_kinematics,
    geometric_jacobian,
    resolved_rate,
)
from feedback import compute_feedback
from dashboard import dashboard_state, register_dashboard_routes

app = FastAPI(title="UR10e Teleop Bridge")
register_dashboard_routes(app)

# Twist scaling factor (tune during testing)
TWIST_SCALE = 1.0

# Max dt to prevent state jumps [s]
MAX_DT = 0.1

# Min dt to skip integration [s]
MIN_DT = 0.001


class RobotState:
    """Per-connection robot joint state."""

    def __init__(self):
        self.q = Q_HOME.copy()
        self.last_time = time.time()


@app.get("/health")
async def health():
    return {"status": "ok"}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    state = RobotState()
    dashboard_state.n_connections += 1

    try:
        while True:
            message = await websocket.receive()

            # Handle both text and binary messages from iOS
            if "text" in message:
                raw = message["text"]
            elif "bytes" in message:
                raw = message["bytes"]
            else:
                continue

            try:
                data = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue

            twist = np.array([
                data.get("vx", 0.0),
                data.get("vy", 0.0),
                data.get("vz", 0.0),
                data.get("wx", 0.0),
                data.get("wy", 0.0),
                data.get("wz", 0.0),
            ]) * TWIST_SCALE

            # Compute dt
            now = time.time()
            dt = min(now - state.last_time, MAX_DT)
            state.last_time = now

            # Kinematics pipeline
            frames = forward_kinematics(state.q)
            J = geometric_jacobian(state.q)
            mu = compute_manipulability(J)
            lam = adaptive_damping(mu)
            qdot = resolved_rate(J, twist, lam)

            # Clamp joint velocities
            qdot = np.clip(qdot, -QDOT_MAX, QDOT_MAX)

            # Integrate joint state
            if dt > MIN_DT:
                state.q = state.q + qdot * dt
                state.q = np.clip(state.q, Q_MIN, Q_MAX)

            # Recompute frames at new q for accurate feedback
            frames = forward_kinematics(state.q)
            J = geometric_jacobian(state.q)

            # Send feedback
            feedback = compute_feedback(state.q, J, frames)
            await websocket.send_text(json.dumps(feedback))

            # Update dashboard state
            dashboard_state.twist = (twist / TWIST_SCALE).tolist()
            dashboard_state.q = state.q.tolist()
            dashboard_state.qdot = qdot.tolist()
            dashboard_state.ee_pos = frames[-1][:3, 3].tolist()
            dashboard_state.ee_dist = float(np.linalg.norm(frames[-1][:3, 3]))
            dashboard_state.mu = float(mu)
            dashboard_state.lam = float(lam)
            dashboard_state.feedback = feedback
            dashboard_state.dt = dt
            dashboard_state.update_msg_rate()

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        dashboard_state.n_connections = max(0, dashboard_state.n_connections - 1)
