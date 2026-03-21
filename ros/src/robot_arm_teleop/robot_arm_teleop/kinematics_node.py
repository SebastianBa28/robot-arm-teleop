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
Q_HOME = np.array([-np.pi/2, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])

# Feedback constants
MAX_REACH = 1.1843  # UR10e approximate max reach [m]
MU_THRESHOLD = 0.005

# DH parameters as (a, d, alpha) tuples for analytical IK (from lab2)
IK_PARAMS = [
    (0.0,      0.1807,  np.pi / 2),
    (-0.6127,  0.0,     0.0),
    (-0.57155, 0.0,     0.0),
    (0.0,      0.17415, np.pi / 2),
    (0.0,      0.11985, -np.pi / 2),
    (0.0,      0.11655, 0.0),
]


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


# ---------------------------------------------------------------------------
# Analytical IK (ported from lab2/lab2.py)
# ---------------------------------------------------------------------------

def _getX(alpha, a):
    return np.array([[1, 0, 0, a],
                     [0, np.cos(alpha), -np.sin(alpha), 0],
                     [0, np.sin(alpha),  np.cos(alpha), 0],
                     [0, 0, 0, 1]])


def _getZ(theta, d):
    return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                     [np.sin(theta),  np.cos(theta), 0, 0],
                     [0, 0, 1, d],
                     [0, 0, 0, 1]])


def safety_check(q):
    """Check that all joint frames stay above ground (z > 0)."""
    T = np.eye(4)
    for i, (a, d, alpha) in enumerate(IK_PARAMS):
        T = T @ _getZ(q[i], d) @ _getX(alpha, a)
        if T[2, 3] < 0:
            return False
    return True


def dh_modified_to_classical(q):
    """Convert modified DH solution to classical DH convention."""
    q_c = q.copy()
    q_c[1] = q[1] - np.pi / 2
    q_c[3] = q[3] - np.pi / 2
    q_c[5] = q[5] + np.pi
    return q_c


def ik(T_bt, T_6t=None):
    """Analytical closed-form IK for UR10e. Returns list of solutions in modified DH."""
    if T_6t is None:
        T_6t = np.eye(4)

    d1 = IK_PARAMS[0][1]   # 0.1807
    a2 = -IK_PARAMS[1][0]  # 0.6127
    a3 = -IK_PARAMS[2][0]  # 0.57155
    d4 = IK_PARAMS[3][1]   # 0.17415
    d5 = IK_PARAMS[4][1]   # 0.11985
    L_B = d1

    T_B0 = np.eye(4)
    T_B0[2, 3] = L_B

    T_06 = np.linalg.inv(T_B0) @ T_bt @ np.linalg.inv(T_6t)

    R = T_06[:3, :3]
    x6, y6, z6 = T_06[0, 3], T_06[1, 3], T_06[2, 3]
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]

    solutions = []

    # Step 1: Solve theta1
    E1, F1, G1 = y6, -x6, d4
    disc1 = E1**2 + F1**2 - G1**2
    if disc1 < -1e-8:
        return solutions
    disc1 = max(disc1, 0.0)

    theta1_solutions = []
    denom1 = G1 - E1
    if abs(denom1) < 1e-12:
        if abs(F1) > 1e-12:
            t_half = -(G1 + E1) / (2 * F1)
            theta1_solutions.append(2 * np.arctan(t_half))
    else:
        for sign in [1, -1]:
            t_half = (-F1 + sign * np.sqrt(disc1)) / denom1
            theta1_solutions.append(2 * np.arctan(t_half))

    for theta1 in theta1_solutions:
        c1, s1 = np.cos(theta1), np.sin(theta1)

        # Step 2: Solve theta6
        theta6 = np.arctan2(r12 * s1 - r22 * c1, r21 * c1 - r11 * s1)

        # Step 3: Solve theta5
        c6, s6 = np.cos(theta6), np.sin(theta6)
        theta5 = np.arctan2(
            (r21 * c1 - r11 * s1) * c6 + (r12 * s1 - r22 * c1) * s6,
            r13 * s1 - r23 * c1
        )

        # Step 4: Solve theta2
        c5, s5 = np.cos(theta5), np.sin(theta5)
        B = r32 * c6 + r31 * s6
        if abs(c5) > 1e-12:
            A = (r31 * c6 - r32 * s6) / c5
        else:
            if abs(s5) > 1e-12:
                A = r33 / s5
            else:
                continue

        a_val = -x6 * c1 - y6 * s1 - d5 * A
        b_val = z6 - d5 * B
        E2 = -2 * a2 * b_val
        F2 = -2 * a2 * a_val
        G2 = a2**2 + a_val**2 + b_val**2 - a3**2
        disc2 = E2**2 + F2**2 - G2**2
        if disc2 < -1e-8:
            continue
        disc2 = max(disc2, 0.0)

        theta2_solutions = []
        denom2 = G2 - E2
        if abs(denom2) < 1e-12:
            if abs(F2) > 1e-12:
                t_half2 = -(G2 + E2) / (2 * F2)
                theta2_solutions.append(2 * np.arctan(t_half2))
        else:
            for sign2 in [1, -1]:
                t_half2 = (-F2 + sign2 * np.sqrt(disc2)) / denom2
                theta2_solutions.append(2 * np.arctan(t_half2))

        for theta2 in theta2_solutions:
            c2, s2 = np.cos(theta2), np.sin(theta2)

            # Step 5: Solve theta3
            theta3 = np.arctan2(a_val - a2 * s2, b_val - a2 * c2) - theta2

            # Step 6: Solve theta4
            theta4 = np.arctan2(A, B) - theta2 - theta3

            q_mod = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
            q_classical = dh_modified_to_classical(q_mod)
            if safety_check(q_classical):
                solutions.append(q_mod)

    return solutions


def is_elbow_up(q_classical):
    """Check if the elbow (joint 3 frame) is above the base plane."""
    T = np.eye(4)
    for i in range(3):
        a, d, alpha = IK_PARAMS[i]
        T = T @ _getZ(q_classical[i], d) @ _getX(alpha, a)
    return T[2, 3] > 0


def pick_closest_solution(solutions, q_current):
    """Pick the closest elbow-up IK solution. Falls back to closest overall."""
    candidates = []
    for sol_mod in solutions:
        q_c = dh_modified_to_classical(sol_mod)
        diff = np.arctan2(np.sin(q_c - q_current), np.cos(q_c - q_current))
        dist = np.linalg.norm(diff)
        candidates.append((q_c, dist, is_elbow_up(q_c)))

    elbow_up = [(q, d) for q, d, eu in candidates if eu]
    if elbow_up:
        return min(elbow_up, key=lambda x: x[1])[0]
    if candidates:
        return min(candidates, key=lambda x: x[1])[0]
    return None


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
        self.current_transform = None  # 4x4 relative transform from iOS
        self.current_mode = "velocity"
        self.reference_ee_pose = None  # captured on first position-mode message
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
                                    if data.get("command") == "reset":
                                        with self._twist_lock:
                                            self.q = Q_HOME.copy()
                                            self.current_twist = np.zeros(6)
                                            self.reference_ee_pose = None
                                        self.get_logger().info('Robot reset to home position')
                                        continue
                                    twist = np.array([
                                        data.get("vx", 0.0), data.get("vy", 0.0), data.get("vz", 0.0),
                                        data.get("wx", 0.0), data.get("wy", 0.0), data.get("wz", 0.0),
                                    ])
                                    with self._twist_lock:
                                        self.current_twist = twist
                                        if "mode" in data:
                                            self.current_mode = data["mode"]
                                        if "transform" in data:
                                            self.current_transform = np.array(data["transform"])
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
        # Read state (thread-safe)
        with self._twist_lock:
            twist = self.current_twist.copy()
            mode = self.current_mode
            transform = self.current_transform.copy() if self.current_transform is not None else None

        # Compute dt
        now = time.time()
        dt = min(now - self.last_time, 0.1)
        self.last_time = now

        # Kinematics pipeline
        J = geometric_jacobian(self.q)
        mu = compute_manipulability(J)
        lam = adaptive_damping(mu)

        if mode == "position" and transform is not None:
            # Capture reference EE pose on first position-mode frame
            if self.reference_ee_pose is None:
                self.reference_ee_pose = forward_kinematics(self.q)[-1].copy()

            # Target pose = reference_ee_pose * relative_transform
            target_pose = self.reference_ee_pose @ transform

            # Analytical IK → pick closest safe solution for continuity
            solutions = ik(target_pose)
            q_target = pick_closest_solution(solutions, self.q)

            if q_target is not None:
                q_dot = (q_target - self.q) / max(dt, 0.001)
                self.q = q_target
            else:
                q_dot = np.zeros(6)  # unreachable — hold position
        else:
            # Velocity mode (existing resolved-rate IK)
            if mode == "velocity":
                self.reference_ee_pose = None

            q_dot = resolved_rate(J, twist, lam)
            q_dot = np.clip(q_dot, -QDOT_MAX, QDOT_MAX)

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
