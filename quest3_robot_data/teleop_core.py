"""Pure dual-arm WebXR-to-TCP mapping logic, independent of ROS and RTDE."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .transforms import matrix_to_quaternion, quaternion_to_matrix, unit_quaternion


# WebXR: +x right, +y up, -z forward. ROS optical: +x right, +y down, +z forward.
WEBXR_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])


@dataclass(frozen=True)
class ControllerState:
    position: np.ndarray
    orientation: np.ndarray
    squeeze: float
    trigger: float

    @classmethod
    def from_dict(cls, value: Any) -> "ControllerState | None":
        if not isinstance(value, dict):
            return None
        try:
            position = np.asarray(value["position"], dtype=np.float64)
            orientation = unit_quaternion(value["orientation"])
            squeeze = float(value.get("squeeze", 0.0))
            trigger = float(value.get("trigger", 0.0))
        except (KeyError, TypeError, ValueError):
            return None
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            return None
        return cls(position, orientation, squeeze, trigger)


@dataclass(frozen=True)
class MappedTarget:
    pose: np.ndarray | None
    enabled: bool
    # None means the controller is not tracked, so preserve the gripper's last
    # physical command rather than treating tracking loss as an open command.
    gripper: float | None
    just_anchored: bool = False


class AnchoredPoseMapper:
    """Map hand motion relative to a clutch anchor into an absolute TCP target."""

    def __init__(
        self,
        base_to_front_camera: np.ndarray,
        position_gain: float = 1.0,
        rotation_gain: float = 1.0,
        deadman_threshold: float = 0.55,
        gripper_close_threshold: float = 0.15,
        enable_rotation: bool = True,
    ) -> None:
        self.base_to_camera = np.asarray(base_to_front_camera, dtype=np.float64)
        if self.base_to_camera.shape != (4, 4):
            raise ValueError("base_to_front_camera must be 4x4")
        self.position_gain = float(position_gain)
        self.rotation_gain = float(rotation_gain)
        self.deadman_threshold = float(deadman_threshold)
        self.gripper_close_threshold = float(gripper_close_threshold)
        self.enable_rotation = bool(enable_rotation)
        self._hand_anchor: ControllerState | None = None
        self._tcp_anchor: np.ndarray | None = None

    def reset(self) -> None:
        self._hand_anchor = None
        self._tcp_anchor = None

    def update(self, hand: ControllerState | None, current_tcp: np.ndarray | None) -> MappedTarget:
        # The trigger is intentionally independent from the side-grip clutch:
        # the clutch protects Cartesian arm motion, while a tracked hand may
        # operate the gripper without moving the arm.  Holding the trigger
        # commands full close; releasing it commands full open.  The Robotiq
        # speed/force limits are enforced by the RTDE backend configuration.
        gripper = None if hand is None else (1.0 if hand.trigger >= self.gripper_close_threshold else 0.0)
        if hand is None or hand.squeeze < self.deadman_threshold or current_tcp is None:
            self.reset()
            return MappedTarget(None, False, gripper)
        current_tcp = np.asarray(current_tcp, dtype=np.float64)
        if current_tcp.shape != (4, 4):
            raise ValueError("current_tcp must be 4x4")
        if self._hand_anchor is None:
            self._hand_anchor = hand
            self._tcp_anchor = current_tcp.copy()
            return MappedTarget(current_tcp.copy(), True, gripper, just_anchored=True)

        base_from_xr = self.base_to_camera[:3, :3] @ WEBXR_TO_OPTICAL
        hand_delta = hand.position - self._hand_anchor.position
        target = self._tcp_anchor.copy()
        target[:3, 3] += base_from_xr @ hand_delta * self.position_gain
        if self.enable_rotation:
            r_now = quaternion_to_matrix(hand.orientation)
            r_anchor = quaternion_to_matrix(self._hand_anchor.orientation)
            delta_xr = r_now @ r_anchor.T
            # Axis-basis change for the relative controller rotation.
            delta_base = base_from_xr @ delta_xr @ base_from_xr.T
            if self.rotation_gain != 1.0:
                # Scale via quaternion angle to keep the implementation stable.
                q = matrix_to_quaternion(delta_base)
                if q[3] < 0:
                    q = -q
                angle = 2 * np.arctan2(np.linalg.norm(q[:3]), q[3])
                if angle > 1e-9:
                    axis = q[:3] / np.linalg.norm(q[:3])
                    half = angle * self.rotation_gain / 2
                    q = np.concatenate((axis * np.sin(half), [np.cos(half)]))
                    delta_base = quaternion_to_matrix(q)
            target[:3, :3] = delta_base @ self._tcp_anchor[:3, :3]
        return MappedTarget(target, True, gripper)
