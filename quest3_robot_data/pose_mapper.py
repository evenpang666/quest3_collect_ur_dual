"""Convert left/right WebXR controller offsets to calibrated UR TCP targets."""
from __future__ import annotations

import argparse
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import json
import os
import time
import traceback
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, String

from .teleop_core import AnchoredPoseMapper, ControllerGate, ControllerState, MappedTarget
from .teleop_avoidance_client import SnapshotExpiredError
from .target_limiter import avoidance_cartesian_limits, limit_cartesian_target
from .transforms import load_transform, matrix_to_quaternion, matrix_to_rotvec, transform


AVOIDANCE_HAND_MAX_AGE_S = 0.15
AVOIDANCE_FEEDBACK_MAX_AGE_S = 0.2
AVOIDANCE_MAX_STATE_DRIFT_RAD = 0.01
AVOIDANCE_PROJECTION_MAX_LATENCY_S = 0.45


@dataclass(frozen=True)
class AvoidanceIntent:
    results: dict[str, MappedTarget]
    moving: dict[str, bool]
    received_at: float
    sequence: int


@dataclass(frozen=True)
class AvoidanceInFlight:
    generation: int
    intent: AvoidanceIntent
    q: np.ndarray
    measured_tcp: dict[str, np.ndarray]
    command_goals: dict[str, np.ndarray]
    started_at: float
    stage: str


def _project_avoidance(client, reset: bool, q: np.ndarray, grip: np.ndarray,
                       goals: dict[str, np.ndarray], moving: dict[str, bool],
                       measured_tcp: dict[str, np.ndarray], inspect: bool) -> tuple[np.ndarray | None, dict]:
    if reset:
        client.reset()
    return client.compute(q, grip, goals, moving, measured_tcp, inspect=inspect)


def _resolve(config_path: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        # First support paths relative to cwd/repository as shown in hardware.yaml,
        # then paths relative to the configuration file itself.
        cwd_path = path.resolve()
        path = cwd_path if cwd_path.exists() else (config_path.parent / path).resolve()
    return path


def _load_calibration(
    config_path: Path,
    value: str,
    expected_parent: str,
    *,
    allow_identity: bool = False,
) -> np.ndarray:
    path = _resolve(config_path, value)
    if not path.is_file():
        if allow_identity:
            return np.eye(4)
        raise FileNotFoundError(f"required calibration not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not data.get("calibrated", False):
        if allow_identity:
            return np.eye(4)
        raise ValueError(f"calibration is not marked calibrated:true: {path}")
    if data.get("parent_frame") != expected_parent:
        if allow_identity:
            return np.eye(4)
        raise ValueError(f"{path}: parent_frame must be {expected_parent!r}")
    return load_transform(data)


class ArmMapping:
    def __init__(
        self,
        node: Node,
        name: str,
        config: dict[str, Any],
        config_path: Path,
        allow_uncalibrated: bool,
    ):
        self.node, self.name, self.config = node, name, config
        calibration = _load_calibration(
            config_path,
            config.get("front_calibration", ""),
            config["base_frame"],
            allow_identity=allow_uncalibrated,
        )
        self.mapper = AnchoredPoseMapper(
            calibration,
            position_gain=float(config.get("position_gain", 1.0)),
            rotation_gain=float(config.get("rotation_gain", 1.0)),
            gripper_close_threshold=float(config.get("gripper_close_threshold", 0.15)),
            enable_rotation=bool(config.get("enable_rotation", True)),
        )
        self.current_tcp: np.ndarray | None = None
        self.tcp_time = 0.0
        self.avoidance_available = os.environ.get("QUEST3_ENABLE_AVOIDANCE") == "1"
        self.target_pub = node.create_publisher(PoseStamped, config["target_topic"], 10)
        self.joint_target_pub = (node.create_publisher(Float64MultiArray, config.get(
            "avoidance_joint_target_topic", f"/{name}/avoidance_joint_target"), 10)
            if self.avoidance_available else None)
        self.avoidance_hold_pub = (node.create_publisher(Bool, f"/{name}/avoidance_hold", 10)
                                   if self.avoidance_available else None)
        self.enable_pub = node.create_publisher(Bool, config["enabled_topic"], 10)
        self.gripper_pub = node.create_publisher(Float32, config["gripper_command_topic"], 10)
        self.state_sub = node.create_subscription(PoseStamped, config["tcp_state_topic"], self._tcp, 10)
        self.joints: np.ndarray | None = None
        self.joint_time = 0.0
        self.gripper_position: float | None = None
        self.gripper_time = 0.0
        self.joint_sub = (node.create_subscription(JointState, config["joint_topic"], self._joints, 10)
                          if self.avoidance_available else None)
        self.gripper_sub = (node.create_subscription(Float32, f"/{name}/gripper_position", self._gripper, 10)
                            if self.avoidance_available else None)
        self.enabled = False

    def _joints(self, message: JointState) -> None:
        value = np.asarray(message.position, dtype=float)
        if value.shape == (6,) and np.isfinite(value).all():
            self.joints, self.joint_time = value, time.monotonic()

    def _gripper(self, message: Float32) -> None:
        if np.isfinite(message.data) and 0 <= message.data <= 1:
            self.gripper_position, self.gripper_time = float(message.data), time.monotonic()

    def _tcp(self, message: PoseStamped) -> None:
        p, q = message.pose.position, message.pose.orientation
        self.current_tcp = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        self.tcp_time = time.monotonic()

    def reset(self) -> None:
        self.mapper.reset()
        self.set_enabled(False)

    def set_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.enable_pub.publish(msg)
        self.enabled = bool(enabled)

    def compute(self, hand: ControllerState | None, recenter: bool) -> MappedTarget:
        if recenter:
            self.mapper.reset()
        return self.mapper.update(hand, self.current_tcp)

    def publish(self, result: MappedTarget) -> None:
        self.set_enabled(result.enabled)
        # Gripper control is deliberately independent of the Cartesian
        # deadman clutch.  This makes trigger press/release usable while the
        # arm stays fixed.  A lost controller publishes None and preserves the
        # last gripper state instead of opening unexpectedly.
        if result.gripper is not None:
            grip = Float32()
            grip.data = float(result.gripper)
            self.gripper_pub.publish(grip)
        if result.pose is None:
            return
        q = matrix_to_quaternion(result.pose[:3, :3])
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.config["base_frame"]
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = result.pose[:3, 3]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.target_pub.publish(msg)

    def publish_avoidance(self, result: MappedTarget, target_q: np.ndarray | None) -> None:
        if not result.enabled:
            self.set_enabled(False)
        if result.gripper is not None:
            grip = Float32()
            grip.data = float(result.gripper)
            self.gripper_pub.publish(grip)
        if result.enabled and target_q is not None:
            message = Float64MultiArray()
            message.data = target_q.tolist()
            assert self.joint_target_pub is not None
            self.joint_target_pub.publish(message)
            self.set_enabled(True)

    def hold_avoidance(self) -> None:
        if self.avoidance_hold_pub is not None:
            self.avoidance_hold_pub.publish(Bool(data=True))
            self.enabled = False


class PoseMapperNode(Node):
    def __init__(self, config: dict[str, Any], config_path: Path):
        super().__init__("quest3_pose_mapper")
        self.config = config
        motion_enabled = bool(config.get("rtde", {}).get("enable_motion", False))
        debug_bypass = bool(config.get("debug", {}).get("allow_uncalibrated_receive_only", False))
        if debug_bypass and motion_enabled:
            raise ValueError("uncalibrated debug mode is forbidden when enable_motion=true")
        allow_uncalibrated = debug_bypass and not motion_enabled
        if allow_uncalibrated:
            self.get_logger().warning(
                "UNCALIBRATED RECEIVE-ONLY DEBUG: using identity front-camera transforms; targets are diagnostic only"
            )
        self.arms = {
            cfg["role"]: ArmMapping(self, name, cfg, config_path, allow_uncalibrated)
            for name, cfg in config["arms"].items()
        }
        if set(self.arms) != ({"left"} if config["mode"] == "single" else {"left", "right"}):
            raise ValueError("arm role values must match mode: single={left}, dual={left,right}")
        self.controller_sub = self.create_subscription(
            String, config.get("controller_topic", "/quest3/controllers"), self._controllers,
            1 if os.environ.get("QUEST3_ENABLE_AVOIDANCE") == "1" else 10
        )
        self.status_pub = self.create_publisher(String, "/quest3/system_status", 10)
        self.avoidance_mode_pub = (self.create_publisher(Bool, "/quest3/avoidance_mode", 10)
                                   if os.environ.get("QUEST3_ENABLE_AVOIDANCE") == "1" else None)
        self.controller_gate = ControllerGate(tuple(self.arms), float(config.get("controller_timeout_sec", 0.25)))
        self.rearm_required = True
        self.min_tcp_separation = float(config.get("min_tcp_separation_m", 0.20))
        self.collision_stop = False
        self.avoidance_available = os.environ.get("QUEST3_ENABLE_AVOIDANCE") == "1"
        self.avoidance_enabled = False
        self.avoidance = None
        self.avoidance_error = ""
        self.avoidance_failure_kind = ""
        self.avoidance_failure_last_log = 0.0
        self.avoidance_event = ""
        self.avoidance_event_seq = 0
        self.avoidance_stale_since: float | None = None
        self.avoidance_generation = 0
        self.avoidance_sequence = 0
        self.avoidance_last_attempt_sequence = -1
        self.avoidance_last_dispatch_at = 0.0
        self.avoidance_latest: AvoidanceIntent | None = None
        self.avoidance_future: Future | None = None
        self.avoidance_inflight: AvoidanceInFlight | None = None
        self.avoidance_reset_pending = True
        self.avoidance_executor = (ThreadPoolExecutor(max_workers=1, thread_name_prefix="quest3-avoidance")
                                   if self.avoidance_available else None)
        self.create_timer(0.05, self._watchdog)
        if self.avoidance_available:
            self.create_timer(0.01, self._avoidance_tick)

    def _controllers(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        now_ns = self.get_clock().now().nanoseconds
        avoidance_available = getattr(self, "avoidance_available", False)
        if avoidance_available:
            source_ns = data.get("host_timestamp_ns")
            if isinstance(source_ns, int) and not 0 <= now_ns - source_ns <= int(AVOIDANCE_HAND_MAX_AGE_S * 1e9):
                if getattr(self, "avoidance_enabled", False):
                    self.avoidance_latest = None
                    self.avoidance_generation += 1
                    for arm in self.arms.values():
                        arm.hold_avoidance()
                return
        avoidance_enabled = getattr(self, "avoidance_enabled", False)
        toggle_avoidance = avoidance_available and data.get("avoidance_toggle") is True
        force_avoidance_off = data.get("avoidance_off") is True or data.get("disconnected") is True
        requested_avoidance = (not avoidance_enabled if toggle_avoidance else avoidance_enabled)
        if force_avoidance_off:
            requested_avoidance = False
        if requested_avoidance != avoidance_enabled:
            self.avoidance_enabled = requested_avoidance
            self.avoidance_error = ""
            self.avoidance_failure_kind = ""
            self.avoidance_stale_since = None
            self.avoidance_generation += 1
            self.avoidance_latest = None
            self.avoidance_reset_pending = True
            self.avoidance_last_attempt_sequence = -1
            self.avoidance_last_dispatch_at = 0.0
            for arm in self.arms.values():
                arm.reset()
            self.rearm_required = requested_avoidance
            if requested_avoidance:
                try:
                    from .teleop_avoidance_client import TeleopAvoidanceClient
                    self.avoidance = TeleopAvoidanceClient(self.config)
                except Exception as exc:
                    self.avoidance = None
                    self.avoidance_enabled = False
                    self.avoidance_error = str(exc)
                    self.rearm_required = False
                    self.get_logger().error(
                        f"避障启用失败，已退出避障模式 ({type(exc).__name__}): "
                        f"{exc}\n{traceback.format_exc()}"
                    )
            else:
                self.avoidance = None
            self.avoidance_event = (
                "enabled" if self.avoidance_enabled else
                "failed" if self.avoidance_error else "disabled")
            self.avoidance_event_seq += 1
            if self.avoidance_mode_pub is not None:
                self.avoidance_mode_pub.publish(Bool(data=self.avoidance_enabled))
        controller_states = {role: ControllerState.from_dict(data.get(role)) for role in self.arms}
        self.controller_gate.observe(controller_states, now_ns)
        if not self.controller_gate.ready(now_ns):
            self.rearm_required = True
            if avoidance_available:
                self.avoidance_latest = None
                self.avoidance_generation += 1
            for arm in self.arms.values():
                arm.reset()
            return
        if self.rearm_required:
            # Tracking may return while a clutch is still held. Require a
            # deliberate release before a new motion anchor can be acquired.
            for role, arm in self.arms.items():
                arm.reset()
                state = controller_states[role]
                assert state is not None
                arm.publish(MappedTarget(None, False, float(state.trigger >= arm.mapper.gripper_close_threshold)))
            if all(
                state is not None and state.squeeze < self.arms[role].mapper.deadman_threshold
                for role, state in controller_states.items()
            ):
                self.rearm_required = False
                self.collision_stop = False
            return
        if self.collision_stop:
            released = all(
                state is None or state.squeeze < self.arms[role].mapper.deadman_threshold
                for role, state in controller_states.items()
            )
            if not released:
                if avoidance_available:
                    self.avoidance_latest = None
                    self.avoidance_generation += 1
                for arm in self.arms.values():
                    arm.reset()
                return
            self.collision_stop = False
        results = {
            role: arm.compute(
                controller_states[role],
                bool(data.get(f"recenter_{role}", False)),
            )
            for role, arm in self.arms.items()
        }
        if "left" in results and "right" in results:
            left, right = results["left"], results["right"]
            if left.enabled and right.enabled and left.pose is not None and right.pose is not None:
                front_to_left = np.linalg.inv(self.arms["left"].mapper.base_to_camera) @ left.pose
                front_to_right = np.linalg.inv(self.arms["right"].mapper.base_to_camera) @ right.pose
                separation = float(np.linalg.norm(front_to_left[:3, 3] - front_to_right[:3, 3]))
                if separation < self.min_tcp_separation:
                    self.collision_stop = True
                    if avoidance_available:
                        self.avoidance_latest = None
                        self.avoidance_generation += 1
                    for arm in self.arms.values():
                        arm.reset()
                    return
        if getattr(self, "avoidance_enabled", False):
            for role, result in results.items():
                self.arms[role].publish_avoidance(result, None)
                if result.just_anchored:
                    self.arms[role].set_enabled(False)
            moving = {role: bool(result.enabled and result.pose is not None and not result.just_anchored)
                      for role, result in results.items()}
            if not any(moving.values()):
                self.avoidance_latest = None
                self.avoidance_generation += 1
                self.avoidance_reset_pending = True
                for arm in self.arms.values():
                    arm.hold_avoidance()
                return
            self.avoidance_sequence += 1
            self.avoidance_latest = AvoidanceIntent(results, moving, time.monotonic(),
                                                    self.avoidance_sequence)
        else:
            for role, arm in self.arms.items():
                arm.publish(results[role])

    def _avoidance_failure(self, exc: Exception) -> None:
        now = time.monotonic()
        kind = ("snapshot_expired" if isinstance(exc, SnapshotExpiredError)
                else f"{type(exc).__name__}: {exc}")
        newly_blocked = not self.avoidance_error or kind != self.avoidance_failure_kind
        self.avoidance_error = str(exc)
        self.avoidance_failure_kind = kind
        if isinstance(exc, SnapshotExpiredError):
            if self.avoidance_stale_since is None:
                self.avoidance_stale_since = now
        if newly_blocked:
            self.avoidance_event = "blocked"
            self.avoidance_event_seq += 1
        if newly_blocked or now - self.avoidance_failure_last_log >= 5.0:
            details = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
            self.get_logger().error(
                f"避障本次动作失败，保持避障模式并等待下一动作 ({type(exc).__name__}): "
                f"{exc}\n{details}"
            )
            self.avoidance_failure_last_log = now
        for arm in self.arms.values():
            arm.hold_avoidance()

    def _avoidance_tick(self) -> None:
        future = self.avoidance_future
        if future is not None:
            if not future.done():
                return
            self.avoidance_future = None
            inflight = self.avoidance_inflight
            self.avoidance_inflight = None
            try:
                target, info = future.result()
            except Exception as exc:
                if inflight is not None and inflight.generation == self.avoidance_generation and self.avoidance_enabled:
                    self._avoidance_failure(exc)
                return
            if inflight is None or inflight.generation != self.avoidance_generation or not self.avoidance_enabled:
                return
            now = time.monotonic()
            if now - inflight.started_at > AVOIDANCE_PROJECTION_MAX_LATENCY_S:
                if inflight.stage == "project" or info.get("needs_projection") is True:
                    for arm in self.arms.values():
                        arm.hold_avoidance()
                return
            latest = self.avoidance_latest
            if (latest is None or not self.controller_gate.ready(self.get_clock().now().nanoseconds)
                    or self.rearm_required or self.collision_stop
                    or now - latest.received_at > AVOIDANCE_HAND_MAX_AGE_S
                    or latest.moving != inflight.intent.moving):
                for arm in self.arms.values():
                    arm.hold_avoidance()
                return
            try:
                q, _, _ = self._avoidance_feedback(now)
            except Exception as exc:
                self._avoidance_failure(exc)
                return
            if np.max(np.abs(q - inflight.q)) > AVOIDANCE_MAX_STATE_DRIFT_RAD:
                # A clear command may still be finishing while inspection runs.
                # Resample it on the next tick without creating a stop-go cycle.
                if inflight.stage == "project" or info.get("needs_projection") is True:
                    for arm in self.arms.values():
                        arm.hold_avoidance()
                return
            max_translation, max_rotation = avoidance_cartesian_limits(
                self.config, self.avoidance.period)
            for role in ("left", "right"):
                if not latest.moving[role]:
                    continue
                pose = latest.results[role].pose
                if pose is None:
                    return
                new_command = limit_cartesian_target(
                    inflight.measured_tcp[role], pose, max_translation, max_rotation,
                )
                old_command = inflight.command_goals[role]
                position_change = np.linalg.norm(new_command[:3, 3] - old_command[:3, 3])
                rotation_change = np.linalg.norm(matrix_to_rotvec(
                    new_command[:3, :3] @ old_command[:3, :3].T))
                if (position_change > max_translation / 2 or
                        rotation_change > max_rotation / 2):
                    for arm in self.arms.values():
                        arm.hold_avoidance()
                    return
            if inflight.stage == "inspect" and info.get("needs_projection") is True:
                # Stop the previous servo target before the slower QP solve.
                # Sample the current robot and latest hand command again after
                # holding; never optimize the earlier inspected command.
                for arm in self.arms.values():
                    arm.hold_avoidance()
                self._submit_avoidance("project", latest, time.monotonic())
                return
            if target is None:
                self._avoidance_failure(RuntimeError("avoidance worker returned no action"))
                return
            if self.avoidance_error:
                self.get_logger().info("避障已恢复，继续发送经检查的动作")
                self.avoidance_error = ""
                self.avoidance_failure_kind = ""
                self.avoidance_stale_since = None
                self.avoidance_event = "resumed"
                self.avoidance_event_seq += 1
            for index, role in enumerate(("left", "right")):
                if latest.moving[role]:
                    self.arms[role].publish_avoidance(
                        latest.results[role], target[index * 6:(index + 1) * 6])

        if not self.avoidance_enabled or self.avoidance is None:
            return
        now = time.monotonic()
        intent = self.avoidance_latest
        if (intent is None or intent.sequence == self.avoidance_last_attempt_sequence
                or now - intent.received_at > AVOIDANCE_HAND_MAX_AGE_S
                or now - self.avoidance_last_dispatch_at < self.avoidance.period):
            return
        if not self.controller_gate.ready(self.get_clock().now().nanoseconds):
            return
        self._submit_avoidance("inspect", intent, now)

    def _submit_avoidance(self, stage: str, intent: AvoidanceIntent, now: float) -> None:
        # Mark only this input as attempted. A newer hand sample that arrived
        # during a failed solve must remain eligible on the next timer tick.
        self.avoidance_last_attempt_sequence = intent.sequence
        # Count computation inside the control period, as safety's replay
        # loop does. Waiting another full period after the result doubles
        # the gap between otherwise clear servoJ targets.
        self.avoidance_last_dispatch_at = now
        try:
            q, grip, measured_tcp = self._avoidance_feedback(now)
        except Exception as exc:
            self._avoidance_failure(exc)
            return
        goals = {role: result.pose for role, result in intent.results.items()
                 if intent.moving[role] and result.pose is not None}
        max_translation, max_rotation = avoidance_cartesian_limits(
            self.config, self.avoidance.period)
        commands = {
            role: limit_cartesian_target(measured_tcp[role], goals[role],
                                         max_translation, max_rotation)
            for role in goals
        }
        self.avoidance_inflight = AvoidanceInFlight(
            self.avoidance_generation, intent, q, measured_tcp, commands, now, stage)
        self.avoidance_future = self.avoidance_executor.submit(
            _project_avoidance, self.avoidance, self.avoidance_reset_pending,
            q, grip, goals, intent.moving, measured_tcp, stage == "inspect")
        self.avoidance_reset_pending = False

    def _avoidance_feedback(self, now: float) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray]]:
        if any(arm.joints is None or arm.gripper_position is None or arm.current_tcp is None or
               now - arm.tcp_time > AVOIDANCE_FEEDBACK_MAX_AGE_S or
               now - arm.joint_time > AVOIDANCE_FEEDBACK_MAX_AGE_S or
               now - arm.gripper_time > AVOIDANCE_FEEDBACK_MAX_AGE_S
               for arm in self.arms.values()):
            raise RuntimeError("joint, TCP or gripper feedback is stale")
        q = np.concatenate([self.arms[role].joints.copy() for role in ("left", "right")])
        grip = np.array([self.arms[role].gripper_position for role in ("left", "right")])
        measured_tcp = {role: self.arms[role].current_tcp.copy() for role in ("left", "right")}
        return q, grip, measured_tcp

    def _watchdog(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        ready = self.controller_gate.ready(now_ns)
        if not ready:
            self.rearm_required = True
            if self.avoidance_available and self.avoidance_latest is not None:
                self.avoidance_latest = None
                self.avoidance_generation += 1
            for arm in self.arms.values():
                if arm.enabled:
                    arm.reset()
                elif self.avoidance_enabled:
                    arm.hold_avoidance()
        status = String()
        values = {f"{role}_arm": "enabled" if arm.enabled else "stopped" for role, arm in self.arms.items()}
        values["controller_ready"] = ready
        values["controller_rearm_required"] = self.rearm_required
        values["controller_missing"] = list(self.controller_gate.missing(now_ns))
        values["dual_tcp_guard"] = "STOP" if self.collision_stop else "ok"
        values["avoidance_available"] = getattr(self, "avoidance_available", False)
        values["avoidance_enabled"] = getattr(self, "avoidance_enabled", False)
        values["avoidance_status"] = getattr(self, "avoidance_error", "") or (
            "active" if values["avoidance_enabled"] else "off")
        values["avoidance_event"] = getattr(self, "avoidance_event", "")
        values["avoidance_event_seq"] = getattr(self, "avoidance_event_seq", 0)
        status.data = json.dumps(values)
        self.status_pub.publish(status)

    def stop(self) -> None:
        for arm in self.arms.values():
            arm.reset()
        if self.avoidance_executor is not None:
            self.avoidance_executor.shutdown(wait=True, cancel_futures=True)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    plain = remove_ros_args(args=argv)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args(plain[1:])
    config_path = args.config.expanduser().resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    rclpy.init(args=argv)
    node = PoseMapperNode(config, config_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
