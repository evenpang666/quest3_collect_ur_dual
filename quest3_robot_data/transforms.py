"""Small, dependency-light rigid transform helpers (quaternions are xyzw)."""
from __future__ import annotations

import math
from typing import Any

import numpy as np


def unit_quaternion(value: Any) -> np.ndarray:
    q = np.asarray(value, dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must contain four finite xyzw values")
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        raise ValueError("zero quaternion is invalid")
    return q / norm


def quaternion_inverse(q: Any) -> np.ndarray:
    x, y, z, w = unit_quaternion(q)
    return np.array([-x, -y, -z, w])


def quaternion_multiply(lhs: Any, rhs: Any) -> np.ndarray:
    x1, y1, z1, w1 = unit_quaternion(lhs)
    x2, y2, z2, w2 = unit_quaternion(rhs)
    return unit_quaternion([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quaternion_to_matrix(q: Any) -> np.ndarray:
    x, y, z, w = unit_quaternion(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(matrix: Any) -> np.ndarray:
    r = np.asarray(matrix, dtype=np.float64)
    if r.shape != (3, 3):
        raise ValueError("rotation must be 3x3")
    # Stable branch-based conversion.
    trace = float(np.trace(r))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) * 2
            q = [0.25 * s, (r[0, 1] + r[1, 0]) / s,
                 (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
        elif i == 1:
            s = math.sqrt(max(0.0, 1 + r[1, 1] - r[0, 0] - r[2, 2])) * 2
            q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s,
                 (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
        else:
            s = math.sqrt(max(0.0, 1 + r[2, 2] - r[0, 0] - r[1, 1])) * 2
            q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s,
                 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    return unit_quaternion(q)


def rotvec_to_matrix(value: Any) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def matrix_to_rotvec(matrix: Any) -> np.ndarray:
    q = matrix_to_quaternion(matrix)
    if q[3] < 0:
        q = -q
    norm = float(np.linalg.norm(q[:3]))
    if norm < 1e-12:
        return np.zeros(3)
    return q[:3] / norm * (2 * math.atan2(norm, float(q[3])))


def transform(translation: Any, quaternion: Any) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = quaternion_to_matrix(quaternion)
    result[:3, 3] = np.asarray(translation, dtype=np.float64)
    return result


def ur_pose_to_matrix(pose: Any) -> np.ndarray:
    value = np.asarray(pose, dtype=np.float64)
    if value.shape != (6,):
        raise ValueError("UR pose must be [x,y,z,rx,ry,rz]")
    result = np.eye(4)
    result[:3, :3] = rotvec_to_matrix(value[3:])
    result[:3, 3] = value[:3]
    return result


def matrix_to_ur_pose(value: Any) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    return np.concatenate((matrix[:3, 3], matrix_to_rotvec(matrix[:3, :3])))


def load_transform(value: dict[str, Any]) -> np.ndarray:
    if "matrix" in value:
        matrix = np.asarray(value["matrix"], dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError("calibration matrix must be 4x4")
        return matrix
    return transform(value["translation"], value["quaternion_xyzw"])


def dump_transform(matrix: Any, parent_frame: str, child_frame: str) -> dict[str, Any]:
    value = np.asarray(matrix, dtype=np.float64)
    return {
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "translation": value[:3, 3].tolist(),
        "quaternion_xyzw": matrix_to_quaternion(value[:3, :3]).tolist(),
        "matrix": value.tolist(),
    }
