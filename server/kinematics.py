"""UR10e kinematics: DH parameters, forward kinematics, Jacobian, resolved-rate control."""

import numpy as np

# UR10e DH parameters (standard convention)
DH_A = np.array([0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0])
DH_D = np.array([0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])

# Joint limits (all joints: -2pi to 2pi)
Q_MIN = np.full(6, -2 * np.pi)
Q_MAX = np.full(6, 2 * np.pi)

# Max joint velocities [rad/s]
QDOT_MAX = np.array([np.pi, np.pi, np.pi, 2 * np.pi, 2 * np.pi, 2 * np.pi])

# Home configuration (non-singular elbow-up pose)
Q_HOME = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])


def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Compute 4x4 homogeneous transform from standard DH parameters."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,       ca,      d],
        [0.0,    0.0,      0.0,    1.0],
    ])


def forward_kinematics(q: np.ndarray) -> list[np.ndarray]:
    """Compute forward kinematics chain. Returns 7 frames: base (identity) through EE."""
    frames = [np.eye(4)]
    for i in range(6):
        T_i = dh_transform(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        frames.append(frames[-1] @ T_i)
    return frames


def geometric_jacobian(q: np.ndarray) -> np.ndarray:
    """Compute 6x6 geometric Jacobian for the UR10e at configuration q."""
    frames = forward_kinematics(q)
    o_n = frames[6][:3, 3]  # end-effector position

    J = np.zeros((6, 6))
    for i in range(6):
        z_i = frames[i][:3, 2]  # z-axis of frame i
        o_i = frames[i][:3, 3]  # origin of frame i
        J[:3, i] = np.cross(z_i, o_n - o_i)  # linear velocity
        J[3:, i] = z_i                         # angular velocity
    return J


def compute_manipulability(J: np.ndarray) -> float:
    """Yoshikawa manipulability measure: sqrt(det(J @ J.T))."""
    return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))


def adaptive_damping(
    mu: float,
    mu_threshold: float = 0.01,
    lambda_min: float = 0.001,
    lambda_max: float = 0.1,
) -> float:
    """Compute damping factor that increases near singularities."""
    if mu > mu_threshold:
        return lambda_min
    ratio = mu / mu_threshold
    return lambda_min + (1.0 - ratio) * (lambda_max - lambda_min)


def resolved_rate(
    J: np.ndarray, twist: np.ndarray, lam: float = 0.01
) -> np.ndarray:
    """Damped least-squares resolved-rate: qdot = J^T (J J^T + lambda^2 I)^-1 twist."""
    A = J @ J.T + lam**2 * np.eye(6)
    x = np.linalg.solve(A, twist)
    return J.T @ x
