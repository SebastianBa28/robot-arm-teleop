"""KinematicsNode: UR10e forward kinematics, IK, feedback, and WebSocket bridge to server."""

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# UR10e DH parameters (standard convention)
DH_A = np.array([0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0])
DH_D = np.array([0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])

# Joint limits
Q_MIN = np.full(6, -2 * np.pi)
Q_MAX = np.full(6, 2 * np.pi)
QDOT_MAX = np.array([np.pi, np.pi, np.pi, 2 * np.pi, 2 * np.pi, 2 * np.pi])
Q_HOME = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])

# Feedback constants
MAX_REACH = 1.1843  # UR10e approximate max reach [m]
MU_THRESHOLD = 0.005


def dh_transform(theta, d, a, alpha):
    """4x4 homogeneous transform from standard DH parameters."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,       ca,      d],
        [0.0,    0.0,      0.0,    1.0],
    ])


def forward_kinematics(q):
    """Compute FK chain. Returns 7 frames: base (identity) through EE."""
    frames = [np.eye(4)]
    for i in range(6):
        T_i = dh_transform(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        frames.append(frames[-1] @ T_i)
    return frames


def geometric_jacobian(q):
    """6x6 geometric Jacobian at configuration q."""
    frames = forward_kinematics(q)
    o_n = frames[6][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z_i = frames[i][:3, 2]
        o_i = frames[i][:3, 3]
        J[:3, i] = np.cross(z_i, o_n - o_i)
        J[3:, i] = z_i
    return J


def compute_manipulability(J):
    """Yoshikawa manipulability measure."""
    return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))


def adaptive_damping(mu, mu_threshold=0.01, lambda_min=0.001, lambda_max=0.1):
    """Damping factor that increases near singularities."""
    if mu > mu_threshold:
        return lambda_min
    ratio = mu / mu_threshold
    return lambda_min + (1.0 - ratio) * (lambda_max - lambda_min)


def resolved_rate(J, twist, lam=0.01):
    """Damped least-squares: qdot = J^T (J J^T + lambda^2 I)^-1 twist."""
    A = J @ J.T + lam ** 2 * np.eye(6)
    x = np.linalg.solve(A, twist)
    return J.T @ x


def compute_feedback(q, J, frames):
    """Compute feedback dict matching iOS FeedbackData format."""
    mu = compute_manipulability(J)

    # Joint limit proximity: 0 = center, 1 = at limit
    center = (Q_MAX + Q_MIN) / 2.0
    half_range = (Q_MAX - Q_MIN) / 2.0
    jl_prox = np.clip(np.abs(q - center) / half_range, 0.0, 1.0).tolist()

    # Workspace proximity: 0 = at base, 1 = at max reach
    ee_pos = frames[-1][:3, 3]
    ws_prox = min(float(np.linalg.norm(ee_pos)) / MAX_REACH, 1.0)

    within_limits = bool(np.all((q >= Q_MIN) & (q <= Q_MAX)))
    is_feasible = mu > MU_THRESHOLD and within_limits

    return {
        "manipulability": float(mu),
        "joint_limit_proximity": jl_prox,
        "workspace_proximity": float(ws_prox),
        "is_feasible": bool(is_feasible),
    }


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        # Parameters
        self.declare_parameter('server_url', 'ws://localhost:8000/ws/ros')

        # State
        self.q = Q_HOME.copy()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.current_twist = np.zeros(6)
        self._twist_lock = threading.Lock()
        self._feedback_to_send = None
        self._feedback_lock = threading.Lock()
        self.last_time = time.time()

        # ROS publishers/subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer for IK and publishing (~30Hz)
        self.dt = 0.033
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # Start WebSocket client thread
        server_url = self.get_parameter('server_url').get_parameter_value().string_value
        self._ws_thread = threading.Thread(target=self._ws_client_loop, args=(server_url,), daemon=True)
        self._ws_thread.start()

        self.get_logger().info('KinematicsNode initialized')

    def _ws_client_loop(self, url):
        """Background thread: connect to server WebSocket, receive twist, send feedback."""
        import asyncio

        async def run():
            try:
                import websockets
            except ImportError:
                self.get_logger().warn('websockets package not installed, server bridge disabled')
                return

            while rclpy.ok():
                try:
                    self.get_logger().info(f'Connecting to server at {url}...')
                    async with websockets.connect(url) as ws:
                        self.get_logger().info('Connected to server')

                        async def receive_twist():
                            async for message in ws:
                                try:
                                    data = json.loads(message)
                                    twist = np.array([
                                        data.get("vx", 0.0), data.get("vy", 0.0), data.get("vz", 0.0),
                                        data.get("wx", 0.0), data.get("wy", 0.0), data.get("wz", 0.0),
                                    ])
                                    with self._twist_lock:
                                        self.current_twist = twist
                                except (json.JSONDecodeError, TypeError):
                                    continue

                        async def send_feedback():
                            while True:
                                await asyncio.sleep(0.033)
                                with self._feedback_lock:
                                    payload = self._feedback_to_send
                                    self._feedback_to_send = None
                                if payload is not None:
                                    await ws.send(json.dumps(payload))

                        await asyncio.gather(receive_twist(), send_feedback())
                except Exception as e:
                    self.get_logger().warn(f'Server connection lost: {e}, reconnecting in 2s...')
                    await asyncio.sleep(2.0)

        asyncio.run(run())

    def cmd_vel_callback(self, msg):
        """Fallback: accept twist from /cmd_vel topic (for testing without server)."""
        twist = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ])
        with self._twist_lock:
            self.current_twist = twist

    def timer_callback(self):
        # Read twist (thread-safe)
        with self._twist_lock:
            twist = self.current_twist.copy()

        # Compute dt
        now = time.time()
        dt = min(now - self.last_time, 0.1)
        self.last_time = now

        # Kinematics pipeline
        J = geometric_jacobian(self.q)
        mu = compute_manipulability(J)
        lam = adaptive_damping(mu)
        q_dot = resolved_rate(J, twist, lam)

        # Clamp per-joint velocities
        q_dot = np.clip(q_dot, -QDOT_MAX, QDOT_MAX)

        # Integrate
        if dt > 0.001:
            self.q = self.q + q_dot * dt
            self.q = np.clip(self.q, Q_MIN, Q_MAX)

        # Compute feedback for server
        frames = forward_kinematics(self.q)
        J_new = geometric_jacobian(self.q)
        feedback = compute_feedback(self.q, J_new, frames)

        ee_pos = frames[-1][:3, 3]
        state_payload = {
            "q": self.q.tolist(),
            "qdot": q_dot.tolist(),
            "ee_pos": ee_pos.tolist(),
            "ee_dist": float(np.linalg.norm(ee_pos)),
            "mu": float(mu),
            "lam": float(lam),
            "feedback": feedback,
            "dt": dt,
        }
        with self._feedback_lock:
            self._feedback_to_send = state_payload

        # Publish joint states for robot_state_publisher / RViz
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.q.tolist()
        msg.velocity = q_dot.tolist()
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
