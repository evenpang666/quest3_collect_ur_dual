"""Dual UR RTDE servoL backend. Cartesian targets use each robot's internal IK."""
from __future__ import annotations

import argparse
import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any, Callable

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
from .target_limiter import limit_cartesian_target
from .transforms import (
    matrix_to_quaternion,
    matrix_to_ur_pose,
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
        safety_event_callback: Callable[[str, str, str], None] | None = None,
    ):
        super().__init__(name=f"rtde-{name}", daemon=True)
        self.node, self.name, self.config, self.common = node, name, config, common
        self.avoidance_available = bool(common.get("avoidance_available", False))
        self.avoidance_mode = False
        self._lock = threading.Lock()
        self.target: np.ndarray | None = None
        self.joint_target: np.ndarray | None = None
        self.target_time = 0.0
        self.enabled = False
        # A protective stop/E-stop terminates the injected RTDE control
        # program.  Never accept a target made before that event after RTDE
        # reconnects; require a new clutch cycle instead.
        self._rearm_required = False
        self._released_since_reconnect = False
        self._safety_event_callback = safety_event_callback
        self._fault_active = False
        self.stopping = threading.Event()
        self.status = "connecting"
        # A virtual Cartesian workspace is optional.  The deployed teleop
        # configurations deliberately omit it: the UR controller's safety
        # planes/limits remain the authoritative motion boundary.
        workspace_min = config.get("workspace_min")
        workspace_max = config.get("workspace_max")
        if (workspace_min is None) != (workspace_max is None):
            raise ValueError(
                f"{name}: workspace_min and workspace_max must be set together, or both omitted"
            )
        self.workspace_min = None if workspace_min is None else np.asarray(workspace_min, dtype=float)
        self.workspace_max = None if workspace_max is None else np.asarray(workspace_max, dtype=float)
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
            if self.avoidance_mode:
                return
            self.target = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            self.joint_target = None
            self.target_time = time.monotonic()

    def set_joint_target(self, message: Float64MultiArray) -> None:
        if not self.avoidance_available:
            return
        value = np.asarray(message.data, dtype=float)
        if value.shape != (6,) or not np.isfinite(value).all():
            self.status = "rejected avoidance joint target"
            return
        with self._lock:
            if not self.avoidance_mode:
                return
            self.joint_target = value
            self.target = None
            self.target_time = time.monotonic()

    def hold_avoidance(self) -> None:
        """Stop a pending projected step without treating it as clutch release."""
        if not self.avoidance_available:
            return
        with self._lock:
            self.target = None
            self.joint_target = None
            self.target_time = 0.0
            self.enabled = False

    def set_avoidance_mode(self, enabled: bool) -> None:
        if not self.avoidance_available:
            return
        with self._lock:
            self.avoidance_mode = bool(enabled)
            self.target = None
            self.joint_target = None
            self.target_time = 0.0
            self.enabled = False

    def set_enabled(self, value: bool) -> None:
        with self._lock:
            requested = bool(value)
            if not requested:
                self.target = None
                self.joint_target = None
                self.target_time = 0.0
            if not self._rearm_required:
                self.enabled = requested
            elif not requested:
                # A release clears the way for the following press.  The pose
                # mapper resets its hand/TCP anchors while the clutch is open.
                self._released_since_reconnect = True
                self.enabled = False
            elif self._released_since_reconnect:
                self._rearm_required = False
                self.enabled = True
                self.status = "ready"
            else:
                # Do not honor a clutch that has been held since before the
                # robot-side control script was stopped.
                self.enabled = False

    def _require_rearm(self) -> None:
        """Discard pre-stop commands and wait for release then clutch press."""
        with self._lock:
            self.enabled = False
            self.target = None
            self.joint_target = None
            self.target_time = 0.0
            self._rearm_required = True
            self._released_since_reconnect = False

    def _report_safety_event(self, event: str, detail: str) -> None:
        """Report one loss-of-control event until normal RTDE state returns."""
        with self._lock:
            if self._fault_active:
                return
            self._fault_active = True
        if self._safety_event_callback is not None:
            self._safety_event_callback(self.name, event, detail)

    def _clear_safety_event(self) -> None:
        with self._lock:
            self._fault_active = False

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
        try:
            from rtde_control import RTDEControlInterface
            from rtde_receive import RTDEReceiveInterface
        except Exception as exc:
            self.status = f"error importing RTDE: {exc!r}"
            return

        hz = float(self.common.get("control_hz", 125.0))
        dt = 1.0 / hz
        timeout = float(self.common["target_timeout_sec"])
        publish_divisor = max(1, round(hz / 30.0))
        reconnect_delay = max(0.1, float(self.common.get("rtde_reconnect_interval_sec", 1.0)))
        attempt = 0
        while not self.stopping.is_set():
            control = receive = None
            active = False
            try:
                attempt += 1
                self.status = "connecting" if attempt == 1 else f"reconnecting (attempt {attempt})"
                receive = RTDEReceiveInterface(self.config["robot_ip"])
                if self.motion_enabled:
                    control = RTDEControlInterface(self.config["robot_ip"])
                    # For robot_active_tcp, leave the pendant-selected TCP alone.
                    if not self.use_active_robot_tcp:
                        assert self.tcp_offset is not None
                        control.setTcp(self.tcp_offset)
                self.status = (
                    "reconnected; release and re-press clutch"
                    if self._rearm_required
                    else ("ready" if self.motion_enabled else "receive-only (enable_motion=false)")
                )
                attempt = 0
                iteration = 0
                active_mode = None
                last_safety_check = 0.0
                while not self.stopping.is_set():
                    period_start = control.initPeriod() if control else None
                    # RTDE's receive interface exposes the controller safety
                    # state even while no clutch is held.  Poll it separately
                    # from servoL so an E-stop also interrupts an episode when
                    # the operator is not actively moving the arm.
                    now = time.monotonic()
                    if now - last_safety_check >= 0.10:
                        last_safety_check = now
                        if receive.isEmergencyStopped():
                            self._report_safety_event(
                                "emergency_stop", "UR emergency stop is active"
                            )
                            raise RuntimeError("UR emergency stop is active")
                        if receive.isProtectiveStopped():
                            self._report_safety_event(
                                "protective_stop", "UR protective stop is active"
                            )
                            raise RuntimeError("UR protective stop is active")
                        self._clear_safety_event()
                    with self._lock:
                        enabled, target, joint_target, avoidance_mode, age = (
                            self.enabled, self.target, self.joint_target, self.avoidance_mode,
                            time.monotonic() - self.target_time)
                    if avoidance_mode:
                        target = None
                    else:
                        joint_target = None
                    valid = self.motion_enabled and enabled and (target is not None or joint_target is not None) and age <= timeout
                    if valid:
                        mode = "joint" if joint_target is not None else "cartesian"
                        if active and mode != active_mode:
                            control.servoStop()
                            active = False
                        active_mode = mode
                        if joint_target is not None:
                            actual_q = np.asarray(receive.getActualQ(), dtype=float)
                            max_step = float(self.common.get("avoidance_joint_max_step_rad", 0.025))
                            # Reject a target whose source feedback no longer
                            # matches the real robot. Never clip an optimized
                            # CBF action after the safety check.
                            if np.max(np.abs(joint_target - actual_q)) > max_step + 0.02:
                                valid = False
                                self.status = "avoidance target expired"
                            else:
                                servo_ok = control.servoJ(
                                    joint_target.tolist(), float(self.common["speed"]),
                                    float(self.common["acceleration"]), dt,
                                    float(self.common["lookahead_time"]), float(self.common["gain"]))
                                if servo_ok is False:
                                    self._report_safety_event("rtde_control_lost", "UR RTDE control script is not running")
                                    raise RuntimeError("UR RTDE control script is not running")
                                active = True
                                self.status = "avoidance enabled"
                        else:
                            position = target[:3, 3]
                        if target is not None and (
                            self.workspace_min is not None
                            and self.workspace_max is not None
                            and (np.any(position < self.workspace_min) or np.any(position > self.workspace_max))
                        ):
                            valid = False
                            self.status = "workspace stop"
                        elif target is not None:
                            command_target = target
                            if self.common.get("limit_cartesian_targets", True):
                                actual = np.asarray(receive.getActualTCPPose(), dtype=float)
                                command_target = limit_cartesian_target(
                                    ur_pose_to_matrix(actual), target,
                                    float(self.common["max_target_jump_m"]),
                                    float(self.common["max_target_jump_rad"]),
                                )
                            command = matrix_to_ur_pose(command_target)
                            # ur_rtde reports a robot-side URScript loss (for
                            # example after an E-stop/protective stop) by
                            # returning False.  It does not necessarily raise
                            # a Python exception, so treating this exactly as
                            # a connection failure is essential: discard the
                            # stale target, tear down this interface and let
                            # the outer loop reconnect indefinitely.
                            servo_ok = control.servoL(
                                command.tolist(),
                                float(self.common["speed"]),
                                float(self.common["acceleration"]),
                                dt,
                                float(self.common["lookahead_time"]),
                                float(self.common["gain"]),
                            )
                            if servo_ok is False:
                                self._report_safety_event(
                                    "rtde_control_lost", "UR RTDE control script is not running"
                                )
                                raise RuntimeError("UR RTDE control script is not running")
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
                if self.stopping.is_set():
                    break
                self._report_safety_event("rtde_connection_lost", str(exc))
                self._require_rearm()
                self.status = f"RTDE lost; retrying: {exc!s}"
            finally:
                if control:
                    try:
                        if active:
                            control.servoStop()
                        control.stopScript()
                    except Exception:
                        pass
                for interface in (control, receive):
                    if interface:
                        try:
                            interface.disconnect()
                        except Exception:
                            pass
            if not self.stopping.is_set():
                self.stopping.wait(reconnect_delay)

        # The gripper connection is independent of the UR RTDE program and is
        # intentionally kept alive while the arm is recovering from an E-stop.
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
        common["avoidance_available"] = os.environ.get("QUEST3_ENABLE_AVOIDANCE") == "1"
        avoid = config.get("obstacle_avoidance", {})
        common["avoidance_joint_max_step_rad"] = float(avoid.get("joint_speed_rad_s", 0.5)) * float(
            avoid.get("period_s", 0.05))
        motion_enabled = bool(common.get("enable_motion", False))
        debug_bypass = bool(config.get("debug", {}).get("allow_uncalibrated_receive_only", False))
        if debug_bypass and motion_enabled:
            raise ValueError("uncalibrated debug mode is forbidden when enable_motion=true")
        allow_uncalibrated = debug_bypass and not motion_enabled
        if allow_uncalibrated:
            self.get_logger().warning(
                "UNCALIBRATED RECEIVE-ONLY DEBUG: RTDE state will be read, but robot and gripper motion are disabled"
            )
        # Create publishers before starting worker threads: a connection may
        # fail immediately during startup and must still be able to signal the
        # recorder/UI safely.
        self.status_pub = self.create_publisher(String, "/quest3/system_status", 10)
        self.safety_event_pub = self.create_publisher(String, "/quest3/safety_event", 10)
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
            worker = ArmWorker(
                self,
                name,
                arm,
                common,
                config_path,
                allow_uncalibrated,
                self._publish_safety_event,
            )
            self.workers[name] = worker
            subscriptions = [
                self.create_subscription(PoseStamped, arm["target_topic"], worker.set_target, 10),
                self.create_subscription(Bool, arm["enabled_topic"], lambda m, w=worker: w.set_enabled(m.data), 10),
                self.create_subscription(Float32, arm["gripper_command_topic"], lambda m, w=worker: w.command_gripper(m.data), 10),
            ]
            if common["avoidance_available"]:
                subscriptions.append(self.create_subscription(
                    Float64MultiArray, arm.get("avoidance_joint_target_topic", f"/{name}/avoidance_joint_target"),
                    worker.set_joint_target, 10))
                subscriptions.append(self.create_subscription(
                    Bool, f"/{name}/avoidance_hold",
                    lambda m, w=worker: w.hold_avoidance() if m.data else None, 10))
            self._subscriptions.extend(subscriptions)
        if common["avoidance_available"]:
            self._subscriptions.append(self.create_subscription(
                Bool, "/quest3/avoidance_mode", self._set_avoidance_mode, 10))
        self.create_timer(0.2, self._status)

    def _set_avoidance_mode(self, message: Bool) -> None:
        for worker in self.workers.values():
            worker.set_avoidance_mode(message.data)

    def _publish_safety_event(self, arm: str, event: str, detail: str) -> None:
        message = String()
        message.data = json.dumps({"event": event, "arm": arm, "detail": detail})
        self.safety_event_pub.publish(message)
        self.get_logger().error(f"Safety event from {arm}: {event}: {detail}")

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
