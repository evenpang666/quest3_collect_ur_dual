"""Dual UR RTDE servoL backend. Cartesian targets use each robot's internal IK."""
from __future__ import annotations

import argparse
import json
import queue
import threading
import time
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

from .pose_mapper import _load_calibration, _resolve
from .robotiq_socket import Robotiq2FClient
from .transforms import (
    matrix_to_quaternion,
    matrix_to_rotvec,
    matrix_to_ur_pose,
    rotvec_to_matrix,
    transform,
    ur_pose_to_matrix,
)


class GripperWorker(threading.Thread):
    def __init__(self, ip: str, port: int, speed: int, force: int):
        super().__init__(name=f"gripper-{ip}", daemon=True)
        self.ip, self.port, self.speed, self.force = ip, port, speed, force
        self.commands: queue.Queue[float | None] = queue.Queue(maxsize=1)
        self.position = 0.0
        self.error: str | None = None
        self.start()

    def command(self, value: float) -> None:
        try:
            self.commands.get_nowait()
        except queue.Empty:
            pass
        try:
            self.commands.put_nowait(float(value))
        except queue.Full:
            pass

    def run(self) -> None:
        client = None
        try:
            client = Robotiq2FClient(self.ip, self.port)
            client.activate(self.speed, self.force)
            self.position = client.position_normalized()
            self.error = None
            last_value = None
            while True:
                value = self.commands.get()
                if value is None:
                    return
                if last_value is not None and abs(value - last_value) < 0.02:
                    continue
                client.move_normalized(value)
                self.position = client.position_normalized()
                last_value = value
        except Exception as exc:
            self.error = repr(exc)
        finally:
            if client:
                client.close()

    def stop(self) -> None:
        try:
            self.commands.get_nowait()
        except queue.Empty:
            pass
        try:
            self.commands.put_nowait(None)
        except queue.Full:
            pass
        self.join(timeout=2)


class ArmWorker(threading.Thread):
    def __init__(
        self,
        node: Node,
        name: str,
        config: dict[str, Any],
        common: dict[str, Any],
        config_path: Path,
        allow_uncalibrated: bool = False,
    ):
        super().__init__(name=f"rtde-{name}", daemon=True)
        self.node, self.name, self.config, self.common = node, name, config, common
        self._lock = threading.Lock()
        self.target: np.ndarray | None = None
        self.target_time = 0.0
        self.enabled = False
        self.stopping = threading.Event()
        self.status = "connecting"
        self.workspace_min = np.asarray(config["workspace_min"], dtype=float)
        self.workspace_max = np.asarray(config["workspace_max"], dtype=float)
        self.use_active_robot_tcp = config.get("tcp_source", "calibration_file") == "robot_active_tcp"
        self.tcp_offset: list[float] | None = None
        if not self.use_active_robot_tcp:
            tcp_path = _resolve(config_path, config.get("tcp_calibration", ""))
            if not tcp_path.is_file():
                if not allow_uncalibrated:
                    raise FileNotFoundError(f"required TCP calibration not found: {tcp_path}")
                self.tcp_offset = [0.0] * 6
            else:
                tcp_data = yaml.safe_load(tcp_path.read_text(encoding="utf-8"))
                if not tcp_data.get("calibrated", False):
                    if not allow_uncalibrated:
                        raise ValueError(f"TCP calibration is not marked calibrated:true: {tcp_path}")
                    self.tcp_offset = [0.0] * 6
                else:
                    self.tcp_offset = list(tcp_data["tcp_offset_xyz_rotvec"])
        self.joint_pub = node.create_publisher(JointState, config["joint_topic"], 20)
        self.tcp_pub = node.create_publisher(PoseStamped, config["tcp_state_topic"], 20)
        self.action_pub = node.create_publisher(Float64MultiArray, config["commanded_action_topic"], 20)
        self.gripper_pub = node.create_publisher(Float32, f"/{name}/gripper_position", 10)
        self.motion_enabled = bool(common.get("enable_motion", False))
        self.gripper = (
            GripperWorker(config["robot_ip"], int(common["gripper_port"]), int(common["gripper_speed"]), int(common["gripper_force"]))
            if self.motion_enabled else None
        )
        self.start()

    def set_target(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.config["base_frame"]:
            self.status = f"rejected target frame {message.header.frame_id}"
            return
        p, q = message.pose.position, message.pose.orientation
        with self._lock:
            self.target = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            self.target_time = time.monotonic()

    def set_enabled(self, value: bool) -> None:
        with self._lock:
            self.enabled = bool(value)

    def command_gripper(self, value: float) -> None:
        if self.gripper is not None:
            self.gripper.command(value)

    def _publish(self, receive: Any) -> None:
        q = np.asarray(receive.getActualQ(), dtype=float)
        qd = np.asarray(receive.getActualQd(), dtype=float)
        target_q = np.asarray(receive.getTargetQ(), dtype=float)
        stamp = self.node.get_clock().now().to_msg()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        joints.position, joints.velocity = q.tolist(), qd.tolist()
        self.joint_pub.publish(joints)
        tcp = receive.getActualTCPPose()
        matrix = ur_pose_to_matrix(tcp)
        quat = matrix_to_quaternion(matrix[:3, :3])
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, self.config["base_frame"]
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = matrix[:3, 3]
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat
        self.tcp_pub.publish(pose)
        action = Float64MultiArray()
        action.data = (target_q - q).tolist()
        self.action_pub.publish(action)
        grip = Float32()
        grip.data = float(self.gripper.position) if self.gripper else float("nan")
        self.gripper_pub.publish(grip)

    def run(self) -> None:
        control = receive = None
        active = False
        try:
            from rtde_control import RTDEControlInterface
            from rtde_receive import RTDEReceiveInterface
            receive = RTDEReceiveInterface(self.config["robot_ip"])
            if self.motion_enabled:
                control = RTDEControlInterface(self.config["robot_ip"])
                # For robot_active_tcp, leave the pendant-selected TCP alone.
                # URScript's active TCP is used by Cartesian commands; calling
                # setTcp without the pendant's six exact calibration values
                # would silently switch control to a different point.
                if not self.use_active_robot_tcp:
                    assert self.tcp_offset is not None
                    control.setTcp(self.tcp_offset)
            hz = float(self.common.get("control_hz", 125.0))
            dt = 1.0 / hz
            timeout = float(self.common["target_timeout_sec"])
            self.status = "ready" if self.motion_enabled else "receive-only (enable_motion=false)"
            publish_divisor = max(1, round(hz / 30.0))
            iteration = 0
            while not self.stopping.is_set():
                period_start = control.initPeriod() if control else None
                with self._lock:
                    enabled, target, age = self.enabled, self.target, time.monotonic() - self.target_time
                valid = self.motion_enabled and enabled and target is not None and age <= timeout
                if valid:
                    position = target[:3, 3]
                    if np.any(position < self.workspace_min) or np.any(position > self.workspace_max):
                        valid = False
                        self.status = "workspace stop"
                    else:
                        actual = np.asarray(receive.getActualTCPPose(), dtype=float)
                        actual_matrix = ur_pose_to_matrix(actual)
                        safe_target = target.copy()
                        delta = safe_target[:3, 3] - actual[:3]
                        max_jump = float(self.common["max_target_jump_m"])
                        norm = float(np.linalg.norm(delta))
                        if norm > max_jump:
                            safe_target[:3, 3] = actual[:3] + delta / norm * max_jump
                        rotation_delta = safe_target[:3, :3] @ actual_matrix[:3, :3].T
                        rotation_vector = matrix_to_rotvec(rotation_delta)
                        rotation_norm = float(np.linalg.norm(rotation_vector))
                        max_rotation = float(self.common["max_target_jump_rad"])
                        if rotation_norm > max_rotation:
                            safe_target[:3, :3] = (
                                rotvec_to_matrix(rotation_vector / rotation_norm * max_rotation)
                                @ actual_matrix[:3, :3]
                            )
                        command = matrix_to_ur_pose(safe_target)
                        control.servoL(command.tolist(), float(self.common["speed"]), float(self.common["acceleration"]), dt, float(self.common["lookahead_time"]), float(self.common["gain"]))
                        active = True
                        self.status = "enabled"
                if not valid and active:
                    control.servoStop()
                    active = False
                    if self.status == "enabled":
                        self.status = "stopped"
                if iteration % publish_divisor == 0:
                    self._publish(receive)
                iteration += 1
                if control:
                    control.waitPeriod(period_start)
                else:
                    time.sleep(dt)
        except Exception as exc:
            self.status = f"error: {exc!r}"
        finally:
            if control:
                try:
                    control.servoStop()
                    control.stopScript()
                except Exception:
                    pass
            if self.gripper:
                self.gripper.stop()

    def stop(self) -> None:
        self.stopping.set()
        self.join(timeout=5)


class RTDEServoNode(Node):
    def __init__(self, config: dict[str, Any], config_path: Path):
        super().__init__("quest3_rtde_servo")
        common = dict(config["rtde"])
        common["control_hz"] = config.get("control_hz", 125.0)
        motion_enabled = bool(common.get("enable_motion", False))
        debug_bypass = bool(config.get("debug", {}).get("allow_uncalibrated_receive_only", False))
        if debug_bypass and motion_enabled:
            raise ValueError("uncalibrated debug mode is forbidden when enable_motion=true")
        allow_uncalibrated = debug_bypass and not motion_enabled
        if allow_uncalibrated:
            self.get_logger().warning(
                "UNCALIBRATED RECEIVE-ONLY DEBUG: RTDE state will be read, but robot and gripper motion are disabled"
            )
        self.workers: dict[str, ArmWorker] = {}
        self._subscriptions = []
        # Validate every arm completely before starting any network/control
        # worker, so a bad right-arm file cannot leave the left arm active.
        for name, arm in config["arms"].items():
            if "CHANGE_ME" in arm["robot_ip"]:
                raise ValueError(f"set a real robot_ip for {name} in hardware.yaml")
            if allow_uncalibrated:
                continue
            # The front camera is the VR control frame.  Side-camera calibration
            # is optional dataset metadata and must not block a single-arm
            # commissioning session where no side camera is connected.
            _load_calibration(config_path, arm["front_calibration"], arm["base_frame"])
            if arm.get("tcp_source", "calibration_file") != "robot_active_tcp":
                tcp_path = _resolve(config_path, arm["tcp_calibration"])
                if not tcp_path.is_file():
                    raise FileNotFoundError(f"required TCP calibration not found: {tcp_path}")
                tcp = yaml.safe_load(tcp_path.read_text(encoding="utf-8"))
                if not tcp.get("calibrated", False) or len(tcp.get("tcp_offset_xyz_rotvec", [])) != 6:
                    raise ValueError(f"invalid TCP calibration: {tcp_path}")
        for name, arm in config["arms"].items():
            worker = ArmWorker(self, name, arm, common, config_path, allow_uncalibrated)
            self.workers[name] = worker
            self._subscriptions.extend([
                self.create_subscription(PoseStamped, arm["target_topic"], worker.set_target, 10),
                self.create_subscription(Bool, arm["enabled_topic"], lambda m, w=worker: w.set_enabled(m.data), 10),
                self.create_subscription(Float32, arm["gripper_command_topic"], lambda m, w=worker: w.command_gripper(m.data), 10),
            ])
        self.status_pub = self.create_publisher(String, "/quest3/system_status", 10)
        self.create_timer(0.2, self._status)

    def _status(self) -> None:
        message = String()
        values: dict[str, str] = {}
        for name, worker in self.workers.items():
            values[f"{name}_rtde"] = worker.status
            if worker.gripper is None:
                values[f"{name}_gripper"] = "disabled (enable_motion=false)"
            elif worker.gripper.error:
                values[f"{name}_gripper"] = f"error: {worker.gripper.error}"
            else:
                values[f"{name}_gripper"] = "ready"
        message.data = json.dumps(values)
        self.status_pub.publish(message)

    def stop(self) -> None:
        for worker in self.workers.values():
            worker.set_enabled(False)
        for worker in self.workers.values():
            worker.stop()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    plain = remove_ros_args(args=argv)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args(plain[1:])
    config_path = args.config.expanduser().resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    rclpy.init(args=argv)
    node = RTDEServoNode(config, config_path)
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
