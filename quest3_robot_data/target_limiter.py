"""Bound the Cartesian command that would be sent to UR servoL."""
from __future__ import annotations

import numpy as np

from .transforms import matrix_to_rotvec, rotvec_to_matrix


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
