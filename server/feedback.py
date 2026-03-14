"""Feedback computation for iOS app: manipulability, joint limits, workspace proximity."""

import numpy as np

from kinematics import Q_MIN, Q_MAX, compute_manipulability

# UR10e approximate max reach [m]
MAX_REACH = 1.1843  # sum of major links: 0.6127 + 0.57155 ≈ 1.18

# Manipulability threshold for feasibility
MU_THRESHOLD = 0.005


def joint_limit_proximity(q: np.ndarray) -> list[float]:
    """Per-joint proximity to limits. 0 = at center, 1 = at limit."""
    center = (Q_MAX + Q_MIN) / 2.0
    half_range = (Q_MAX - Q_MIN) / 2.0
    proximity = np.abs(q - center) / half_range
    return np.clip(proximity, 0.0, 1.0).tolist()


def workspace_proximity(frames: list[np.ndarray]) -> float:
    """How close the end-effector is to workspace boundary. 0 = at base, 1 = at limit."""
    ee_pos = frames[-1][:3, 3]
    r = float(np.linalg.norm(ee_pos))
    return min(r / MAX_REACH, 1.0)


def compute_feedback(
    q: np.ndarray, J: np.ndarray, frames: list[np.ndarray]
) -> dict:
    """Compute full feedback dict matching iOS FeedbackData format."""
    mu = compute_manipulability(J)
    jl_prox = joint_limit_proximity(q)
    ws_prox = workspace_proximity(frames)

    within_limits = bool(np.all((q >= Q_MIN) & (q <= Q_MAX)))
    is_feasible = mu > MU_THRESHOLD and within_limits

    return {
        "manipulability": float(mu),
        "joint_limit_proximity": jl_prox,
        "workspace_proximity": float(ws_prox),
        "is_feasible": bool(is_feasible),
    }
