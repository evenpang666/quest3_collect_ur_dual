"""Bound Cartesian and joint commands sent to UR servo control."""
from __future__ import annotations

import numpy as np

from .transforms import matrix_to_rotvec, rotvec_to_matrix


def limit_joint_target(previous: np.ndarray, target: np.ndarray,
                       max_speed: float, period: float) -> np.ndarray:
    """Bound each joint's command rate, preserving the joint-space direction.

    Use the previous command (measured joints for the first command), rather
    than re-anchoring every cycle to feedback that may lag or run ahead.
    Do not wrap angles: IK must select the nearby branch using qnear.
    """
    previous, target = np.asarray(previous, dtype=float), np.asarray(target, dtype=float)
    if (previous.shape != (6,) or target.shape != (6,)
            or not np.isfinite(previous).all() or not np.isfinite(target).all()):
        raise ValueError("joint commands must contain six finite angles")
    if not np.isfinite(max_speed) or max_speed <= 0 or not np.isfinite(period) or period <= 0:
        raise ValueError("joint speed and control period must be finite and positive")
    delta = target - previous
    largest = float(np.max(np.abs(delta)))
    bound = max_speed * period
    if largest > bound:
        delta = delta * (bound / largest)
    return previous + delta


def limit_cartesian_target(actual: np.ndarray, target: np.ndarray,
                           max_translation: float, max_rotation: float) -> np.ndarray:
    command = np.asarray(target, dtype=float).copy()
    actual = np.asarray(actual, dtype=float)
    delta = command[:3, 3] - actual[:3, 3]
    distance = float(np.linalg.norm(delta))
    if distance > max_translation:
        command[:3, 3] = actual[:3, 3] + delta / distance * max_translation
    rotation_delta = command[:3, :3] @ actual[:3, :3].T
    rotation_vector = matrix_to_rotvec(rotation_delta)
    angle = float(np.linalg.norm(rotation_vector))
    if angle > max_rotation:
        command[:3, :3] = (rotvec_to_matrix(rotation_vector / angle * max_rotation)
                            @ actual[:3, :3])
    return command


def avoidance_cartesian_limits(config: dict, period: float) -> tuple[float, float]:
    """Convert the 125 Hz servoL per-cycle jumps to one avoidance tick.

    Avoidance sends a checked servoJ step at a slower cadence. Reusing one
    servoL cycle's displacement for that whole tick would reduce hand speed
    in proportion to the cadence difference. Joint speed and FK checks still
    bound the final joint command.
    """
    rtde = config["rtde"]
    scale = float(config.get("control_hz", 125.0)) * float(period)
    return (float(rtde["max_target_jump_m"]) * scale,
            float(rtde["max_target_jump_rad"]) * scale)
