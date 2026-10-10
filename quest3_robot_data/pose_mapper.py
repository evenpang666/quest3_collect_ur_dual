"""Convert left/right WebXR controller offsets to calibrated UR TCP targets."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, Float32, String

from .teleop_core import AnchoredPoseMapper, ControllerGate, ControllerState, MappedTarget
from .transforms import load_transform, matrix_to_quaternion, transform


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
        self.target_pub = node.create_publisher(PoseStamped, config["target_topic"], 10)
        self.enable_pub = node.create_publisher(Bool, config["enabled_topic"], 10)
        self.gripper_pub = node.create_publisher(Float32, config["gripper_command_topic"], 10)
        self.state_sub = node.create_subscription(PoseStamped, config["tcp_state_topic"], self._tcp, 10)
        self.enabled = False

    def _tcp(self, message: PoseStamped) -> None:
        p, q = message.pose.position, message.pose.orientation
        self.current_tcp = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

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


class PoseMapperNode(Node):
    def __init__(self, config: dict[str, Any], config_path: Path):
        super().__init__("quest3_pose_mapper")
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
            String, config.get("controller_topic", "/quest3/controllers"), self._controllers, 10
        )
        self.status_pub = self.create_publisher(String, "/quest3/system_status", 10)
        self.controller_gate = ControllerGate(tuple(self.arms), float(config.get("controller_timeout_sec", 0.25)))
        self.rearm_required = True
        self.min_tcp_separation = float(config.get("min_tcp_separation_m", 0.20))
        self.collision_stop = False
        self.create_timer(0.05, self._watchdog)

    def _controllers(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        now_ns = self.get_clock().now().nanoseconds
        controller_states = {role: ControllerState.from_dict(data.get(role)) for role in self.arms}
        self.controller_gate.observe(controller_states, now_ns)
        if not self.controller_gate.ready(now_ns):
            self.rearm_required = True
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
                    for arm in self.arms.values():
                        arm.reset()
                    return
        for role, arm in self.arms.items():
            arm.publish(results[role])

    def _watchdog(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        ready = self.controller_gate.ready(now_ns)
        if not ready:
            self.rearm_required = True
            for arm in self.arms.values():
                if arm.enabled:
                    arm.reset()
        status = String()
        values = {f"{role}_arm": "enabled" if arm.enabled else "stopped" for role, arm in self.arms.items()}
        values["controller_ready"] = ready
        values["controller_rearm_required"] = self.rearm_required
        values["controller_missing"] = list(self.controller_gate.missing(now_ns))
        values["dual_tcp_guard"] = "STOP" if self.collision_stop else "ok"
        status.data = json.dumps(values)
        self.status_pub.publish(status)

    def stop(self) -> None:
        for arm in self.arms.values():
            arm.reset()


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
