"""Pure dual-arm WebXR-to-TCP mapping logic, independent of ROS and RTDE."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from .transforms import matrix_to_quaternion, quaternion_to_matrix, unit_quaternion


# Field-checked grip-to-Tool axis alignment: X same, Y and Z reversed.
# This is a 180-degree rotation about X, so orientation remains right-handed.
GRIP_TO_TOOL_AXES = np.diag([1.0, -1.0, -1.0])


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
        if (
            position.shape != (3,)
            or not np.all(np.isfinite(position))
            or not np.isfinite(squeeze)
            or not np.isfinite(trigger)
        ):
            return None
        return cls(position, orientation, squeeze, trigger)


class ControllerGate:
    """Require fresh tracking for every configured controller role."""

    def __init__(self, roles: tuple[str, ...], timeout_sec: float):
        self.roles = roles
        self.timeout_ns = int(timeout_sec * 1e9)
        self.last_ns: int | None = None
        self.tracked = {role: False for role in roles}

    def observe(self, states: Mapping[str, ControllerState | None], now_ns: int) -> None:
        self.last_ns = now_ns
        self.tracked = {role: states.get(role) is not None for role in self.roles}

    def ready(self, now_ns: int) -> bool:
        return (
            self.last_ns is not None
            and 0 <= now_ns - self.last_ns <= self.timeout_ns
            and all(self.tracked.values())
        )

    def missing(self, now_ns: int) -> tuple[str, ...]:
        if self.last_ns is None or not 0 <= now_ns - self.last_ns <= self.timeout_ns:
            return self.roles
        return tuple(role for role in self.roles if not self.tracked[role])


@dataclass(frozen=True)
class MappedTarget:
    pose: np.ndarray | None
    enabled: bool
    # None means the controller is not tracked, so preserve the gripper's last
    # physical command rather than treating tracking loss as an open command.
    gripper: float | None
    just_anchored: bool = False


class AnchoredPoseMapper:
    """Map each hand grip frame onto its arm's Tool/TCP frame at clutch time."""

    def __init__(
        self,
        base_to_front_camera: np.ndarray,
        position_gain: float = 1.0,
        rotation_gain: float = 1.0,
        deadman_threshold: float = 0.55,
        gripper_close_threshold: float = 0.15,
        enable_rotation: bool = True,
    ) -> None:
        # Retained for the dual-arm TCP separation check in pose_mapper.
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

        hand_anchor_rotation = quaternion_to_matrix(self._hand_anchor.orientation)
        hand_rotation = quaternion_to_matrix(hand.orientation)
        tool_rotation = self._tcp_anchor[:3, :3]
        # Express displacement in the anchored grip frame, correct its axes,
        # then convert through the anchored TCP orientation into the base.
        hand_delta = hand_anchor_rotation.T @ (hand.position - self._hand_anchor.position)
        target = self._tcp_anchor.copy()
        target[:3, 3] += tool_rotation @ GRIP_TO_TOOL_AXES @ hand_delta * self.position_gain
        if self.enable_rotation:
            delta_grip = hand_anchor_rotation.T @ hand_rotation
            delta_tool = GRIP_TO_TOOL_AXES @ delta_grip @ GRIP_TO_TOOL_AXES.T
            if self.rotation_gain != 1.0:
                # Scale via quaternion angle to keep the implementation stable.
                q = matrix_to_quaternion(delta_tool)
                if q[3] < 0:
                    q = -q
                angle = 2 * np.arctan2(np.linalg.norm(q[:3]), q[3])
                if angle > 1e-9:
                    axis = q[:3] / np.linalg.norm(q[:3])
                    half = angle * self.rotation_gain / 2
                    q = np.concatenate((axis * np.sin(half), [np.cos(half)]))
                    delta_tool = quaternion_to_matrix(q)
            target[:3, :3] = tool_rotation @ delta_tool
        return MappedTarget(target, True, gripper)
