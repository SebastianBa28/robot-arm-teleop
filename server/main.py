"""FastAPI WebSocket server: relay between iOS app and ROS KinematicsNode."""

import asyncio
import json

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from dashboard import dashboard_state, register_dashboard_routes

app = FastAPI(title="UR10e Teleop Bridge")
register_dashboard_routes(app)

# Shared state for relay
latest_twist = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "wx": 0.0, "wy": 0.0, "wz": 0.0}
latest_transform = None
latest_mode = "velocity"
latest_feedback = {}

# Connected clients
ros_clients: set[WebSocket] = set()
ios_clients: set[WebSocket] = set()


@app.get("/health")
async def health():
    return {"status": "ok"}


@app.websocket("/ws")
async def ios_endpoint(websocket: WebSocket):
    """iOS app connects here: sends twist, receives feedback."""
    await websocket.accept()
    ios_clients.add(websocket)
    dashboard_state.n_connections += 1

    try:
        while True:
            message = await websocket.receive()

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

            # Store latest twist, transform, and mode
            global latest_twist, latest_transform, latest_mode
            latest_twist = {
                "vx": data.get("vx", 0.0),
                "vy": data.get("vy", 0.0),
                "vz": data.get("vz", 0.0),
                "wx": data.get("wx", 0.0),
                "wy": data.get("wy", 0.0),
                "wz": data.get("wz", 0.0),
            }
            if "transform" in data:
                latest_transform = data["transform"]
            if "mode" in data:
                latest_mode = data["mode"]

            # Update dashboard twist
            dashboard_state.twist = [
                latest_twist["vx"], latest_twist["vy"], latest_twist["vz"],
                latest_twist["wx"], latest_twist["wy"], latest_twist["wz"],
            ]
            dashboard_state.update_msg_rate()

            # Forward full message to all ROS clients
            relay_msg = {**latest_twist, "mode": latest_mode}
            if latest_transform is not None:
                relay_msg["transform"] = latest_transform
            if "command" in data:
                relay_msg["command"] = data["command"]
            dead = set()
            for ros_ws in ros_clients:
                try:
                    await ros_ws.send_text(json.dumps(relay_msg))
                except Exception:
                    dead.add(ros_ws)
            ros_clients.difference_update(dead)

            # Send latest feedback to iOS (if available)
            if latest_feedback:
                await websocket.send_text(json.dumps(latest_feedback))

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"iOS WebSocket error: {e}")
    finally:
        ios_clients.discard(websocket)
        dashboard_state.n_connections = max(0, dashboard_state.n_connections - 1)


@app.websocket("/ws/ros")
async def ros_endpoint(websocket: WebSocket):
    """ROS KinematicsNode connects here: receives twist, sends feedback+state."""
    await websocket.accept()
    ros_clients.add(websocket)
    print("ROS client connected")

    try:
        while True:
            raw = await websocket.receive_text()

            try:
                data = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue

            # Update feedback for iOS
            global latest_feedback
            if "feedback" in data:
                latest_feedback = data["feedback"]

            # Update dashboard state from ROS data
            if "q" in data:
                dashboard_state.q = data["q"]
            if "qdot" in data:
                dashboard_state.qdot = data["qdot"]
            if "ee_pos" in data:
                dashboard_state.ee_pos = data["ee_pos"]
            if "ee_dist" in data:
                dashboard_state.ee_dist = data["ee_dist"]
            if "mu" in data:
                dashboard_state.mu = data["mu"]
            if "lam" in data:
                dashboard_state.lam = data["lam"]
            if "feedback" in data:
                dashboard_state.feedback = data["feedback"]
            if "dt" in data:
                dashboard_state.dt = data["dt"]

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"ROS WebSocket error: {e}")
    finally:
        ros_clients.discard(websocket)
        print("ROS client disconnected")
