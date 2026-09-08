"""Synchronized ROS 2 recorder for real UR robot observations.

The writer deliberately runs outside ROS callbacks.  A callback only converts and
copies a synchronized sample into a bounded queue; slow HDF5 I/O can therefore
never stall the robot state subscriber.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import os
import queue
import re
import sys
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import h5py
import numpy as np
import yaml

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, String


@dataclass
class Sample:
    """One front-camera-clocked robot observation."""

    timestamp_ns: int
    source_timestamps_ns: dict[str, int]
    images: dict[str, dict[str, np.ndarray]]
    qpos: np.ndarray
    qvel: np.ndarray
    commanded_action: np.ndarray
    gripper_position: np.ndarray
    gripper_command: np.ndarray


@dataclass
class StartEpisode:
    path: Path
    metadata: dict[str, Any]


class EpisodeWriter(threading.Thread):
    """Serial HDF5 writer with explicit commanded/executed action semantics."""

    STOP = object()
    SHUTDOWN = object()

    def __init__(self, max_queue_size: int, rgb_storage: str = "raw_lzf", jpeg_quality: int = 90):
        super().__init__(name="hdf5-writer", daemon=True)
        self.events: queue.Queue[StartEpisode | Sample | object] = queue.Queue(max_queue_size)
        self.rgb_storage = rgb_storage
        self.jpeg_quality = int(jpeg_quality)
        self._error: Exception | None = None
        self.start()

    @property
    def failed(self) -> Exception | None:
        return self._error

    def put_control(self, item: StartEpisode | object) -> bool:
        """Control messages must not be dropped; they preserve episode boundaries."""
        while self.is_alive():
            try:
                self.events.put(item, timeout=0.1)
                return True
            except queue.Full:
                continue
        return False

    def put_sample(self, sample: Sample) -> bool:
        try:
            self.events.put_nowait(sample)
            return True
        except queue.Full:
            return False

    def run(self) -> None:
        file: h5py.File | None = None
        final_path: Path | None = None
        pending: Sample | None = None
        count = 0
        try:
            while True:
                event = self.events.get()
                if event is self.SHUTDOWN:
                    if file is not None:
                        self._close_episode(file, final_path, pending, count)
                    return
                if isinstance(event, StartEpisode):
                    if file is not None:
                        # This cannot happen through the Node API, but protects
                        # against accidental overlapping episode commands.
                        self._close_episode(file, final_path, pending, count)
                    event.path.parent.mkdir(parents=True, exist_ok=True)
                    inprogress = event.path.with_suffix(".inprogress")
                    file = h5py.File(inprogress, "x")
                    for key, value in event.metadata.items():
                        file.attrs[key] = value
                    file.attrs["complete"] = False
                    file.attrs["action_definition"] = "commanded target_q - actual_q (radians)"
                    file.attrs["executed_action_definition"] = "qpos[t+1] - qpos[t] (radians)"
                    file.attrs["final_action"] = "action_valid=false; executed_action=zeros"
                    final_path, pending, count = event.path, None, 0
                    print(f"[writer] RECORDING {event.path}", flush=True)
                    continue
                if event is self.STOP:
                    if file is None:
                        print("[writer] empty episode discarded", flush=True)
                    else:
                        self._close_episode(file, final_path, pending, count)
                    file, final_path, pending, count = None, None, None, 0
                    continue
                if not isinstance(event, Sample):
                    continue
                if file is None:
                    continue
                if pending is None:
                    pending = event
                else:
                    executed_action = event.qpos - pending.qpos
                    self._append(file, pending, executed_action, action_valid=True, terminal=False)
                    count += 1
                    pending = event
        except Exception as exc:  # Keep the in-progress file for recovery/debugging.
            self._error = exc
            if file is not None:
                try:
                    file.flush()
                    file.close()
                except Exception:
                    pass
            print(f"[writer] ERROR: {exc!r}; retained incomplete file", file=sys.stderr, flush=True)

    def _close_episode(
        self, file: h5py.File, final_path: Path | None, pending: Sample | None, count: int
    ) -> None:
        if pending is not None:
            self._append(
                file, pending, np.zeros_like(pending.qpos), action_valid=False, terminal=True
            )
            count += 1
        file.attrs["num_samples"] = count
        file.attrs["complete"] = True
        inprogress_path = Path(file.filename)
        file.flush()
        file.close()
        if count and final_path is not None:
            os.replace(inprogress_path, final_path)
            print(f"[writer] SAVED {final_path} ({count} samples)", flush=True)
        else:
            # Empty HDF5 episodes are not useful and removing this exact temp file
            # is safe; it was created solely by this writer.
            inprogress_path.unlink(missing_ok=True)
            print("[writer] empty episode discarded", flush=True)

    @staticmethod
    def _resize_write(dataset: h5py.Dataset, value: np.ndarray | int) -> None:
        index = dataset.shape[0]
        dataset.resize(index + 1, axis=0)
        dataset[index] = value

    def _append(
        self,
        file: h5py.File,
        sample: Sample,
        executed_action: np.ndarray,
        action_valid: bool,
        terminal: bool,
    ) -> None:
        if "timestamps_ns" not in file:
            self._create_layout(file, sample)
        self._resize_write(file["timestamps_ns"], np.uint64(sample.timestamp_ns))
        self._resize_write(file["observations/qpos"], sample.qpos.astype(np.float32, copy=False))
        self._resize_write(file["observations/qvel"], sample.qvel.astype(np.float32, copy=False))
        self._resize_write(file["observations/gripper_position"], sample.gripper_position.astype(np.float32, copy=False))
        self._resize_write(file["action"], sample.commanded_action.astype(np.float32, copy=False))
        self._resize_write(file["executed_action"], executed_action.astype(np.float32, copy=False))
        self._resize_write(file["gripper_action"], sample.gripper_command.astype(np.float32, copy=False))
        self._resize_write(file["action_valid"], np.bool_(action_valid))
        self._resize_write(file["is_terminal"], np.bool_(terminal))
        for source, timestamp in sample.source_timestamps_ns.items():
            self._resize_write(file[f"source_timestamps_ns/{source}"], np.uint64(timestamp))
        for camera, streams in sample.images.items():
            for stream, image in streams.items():
                dataset = file[f"observations/images/{camera}/{stream}"]
                if stream == "rgb" and dataset.attrs.get("storage") == "jpeg":
                    import cv2

                    ok, encoded = cv2.imencode(
                        ".jpg", image[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                    )
                    if not ok:
                        raise RuntimeError(f"JPEG encoding failed for {camera}")
                    self._resize_write(dataset, encoded.reshape(-1))
                    continue
                if tuple(dataset.shape[1:]) != image.shape or dataset.dtype != image.dtype:
                    raise ValueError(
                        f"{camera}/{stream} changed from {dataset.shape[1:]}/{dataset.dtype} "
                        f"to {image.shape}/{image.dtype} within one episode"
                    )
                self._resize_write(dataset, image)

    @staticmethod
    def _create_dataset(file: h5py.File, name: str, shape: tuple[int, ...], dtype: Any) -> None:
        file.create_dataset(
            name,
            shape=(0, *shape),
            maxshape=(None, *shape),
            dtype=dtype,
            chunks=(1, *shape),
            compression="lzf",
        )

    def _create_layout(self, file: h5py.File, sample: Sample) -> None:
        self._create_dataset(file, "timestamps_ns", (), np.uint64)
        self._create_dataset(file, "observations/qpos", sample.qpos.shape, np.float32)
        self._create_dataset(file, "observations/qvel", sample.qvel.shape, np.float32)
        self._create_dataset(file, "observations/gripper_position", sample.gripper_position.shape, np.float32)
        self._create_dataset(file, "action", sample.commanded_action.shape, np.float32)
        self._create_dataset(file, "executed_action", sample.qpos.shape, np.float32)
        self._create_dataset(file, "gripper_action", sample.gripper_command.shape, np.float32)
        self._create_dataset(file, "action_valid", (), np.bool_)
        self._create_dataset(file, "is_terminal", (), np.bool_)
        for source in sample.source_timestamps_ns:
            self._create_dataset(file, f"source_timestamps_ns/{source}", (), np.uint64)
        for camera, streams in sample.images.items():
            for stream, image in streams.items():
                name = f"observations/images/{camera}/{stream}"
                if stream == "rgb" and self.rgb_storage == "jpeg":
                    file.create_dataset(
                        name,
                        shape=(0,),
                        maxshape=(None,),
                        dtype=h5py.vlen_dtype(np.dtype("uint8")),
                        chunks=(1,),
                    )
                else:
                    self._create_dataset(file, name, image.shape, image.dtype)
                dataset = file[f"observations/images/{camera}/{stream}"]
                dataset.attrs["encoding"] = (
                    "jpeg bytes decoded as RGB8"
                    if stream == "rgb" and self.rgb_storage == "jpeg"
                    else "rgb8" if stream == "rgb" else "source passthrough"
                )
                dataset.attrs["storage"] = "jpeg" if stream == "rgb" and self.rgb_storage == "jpeg" else "raw_lzf"
                dataset.attrs["original_shape"] = image.shape
                dataset.attrs["unit"] = "" if stream == "rgb" else "driver-defined (usually mm for 16UC1)"


class DatasetRecorder(Node):
    """Camera-clocked nearest-frame aggregator with interpolated robot state."""

    def __init__(self, config: dict[str, Any], output_dir: Path):
        super().__init__("quest3_dataset_recorder")
        self.config = self._validate_config(config)
        self.output_dir = output_dir.expanduser().resolve()
        self.bridge = CvBridge()
        self.writer = EpisodeWriter(
            int(self.config["writer_queue_size"]),
            self.config["rgb_storage"],
            int(self.config["jpeg_quality"]),
        )
        self.recording = False
        self.synced_frames = 0
        self.enqueued_frames = 0
        self.dropped_frames = 0
        self._ui_commands: queue.SimpleQueue[str] = queue.SimpleQueue()
        self._episode_number = self._next_episode_number()
        self._last_reference_stamp = -1
        buffer_size = int(self.config["source_buffer_size"])
        self._buffers: dict[str, deque[tuple[int, Any]]] = {}
        self._subscriptions = []

        self.record_sub = self.create_subscription(
            Bool, self.config["record_topic"], self._record_command, 10
        )
        self.status_pub = self.create_publisher(String, "/quest3/system_status", 10)
        self.create_timer(0.005, self._aggregate_latest_reference)
        self.create_timer(0.1, self._process_ui_commands)
        self.create_timer(2.0, self._report_stats)

        self.camera_order = list(self.config["cameras"])
        self.arm_order = list(self.config["arms"])
        self._stream_keys: list[tuple[str, str]] = []
        for name in self.camera_order:
            camera = self.config["cameras"][name]
            key = f"image:{name}:rgb"
            self._buffers[key] = deque(maxlen=buffer_size)
            self._subscriptions.append(self.create_subscription(
                Image, camera["rgb_topic"], lambda m, k=key: self._buffer_header_message(k, m), qos_profile_sensor_data
            ))
            self._stream_keys.append((name, "rgb"))
            if "depth_topic" in camera:
                key = f"image:{name}:depth"
                self._buffers[key] = deque(maxlen=buffer_size)
                self._subscriptions.append(self.create_subscription(
                    Image, camera["depth_topic"], lambda m, k=key: self._buffer_header_message(k, m), qos_profile_sensor_data
                ))
                self._stream_keys.append((name, "depth"))
        for name in self.arm_order:
            arm = self.config["arms"][name]
            joint_key, action_key, grip_key, grip_command_key = (
                f"joint:{name}", f"action:{name}", f"gripper:{name}", f"gripper_command:{name}"
            )
            for key in (joint_key, action_key, grip_key, grip_command_key):
                self._buffers[key] = deque(maxlen=buffer_size)
            self._subscriptions.extend([
                self.create_subscription(JointState, arm["joint_topic"], lambda m, k=joint_key: self._buffer_header_message(k, m), qos_profile_sensor_data),
                self.create_subscription(Float64MultiArray, arm["commanded_action_topic"], lambda m, k=action_key: self._buffer_arrival_message(k, m), 20),
                self.create_subscription(Float32, arm["gripper_position_topic"], lambda m, k=grip_key: self._buffer_arrival_message(k, m), 10),
                self.create_subscription(Float32, arm["gripper_command_topic"], lambda m, k=grip_command_key: self._buffer_arrival_message(k, m), 10),
            ])
        self.get_logger().info(
            f"READY mode={self.config['mode']}, reference=front/rgb, "
            f"record topic={self.config['record_topic']}. Send Bool true to start."
        )

    @staticmethod
    def _validate_config(config: dict[str, Any]) -> dict[str, Any]:
        required = {"mode", "record_topic", "cameras", "arms"}
        missing = required - config.keys()
        if missing:
            raise ValueError(f"config missing: {sorted(missing)}")
        mode = config["mode"]
        if mode not in {"single", "dual"}:
            raise ValueError("mode must be 'single' or 'dual'")
        expected_arms = 1 if mode == "single" else 2
        if len(config["arms"]) != expected_arms:
            raise ValueError(f"mode={mode} requires exactly {expected_arms} arm configuration(s)")
        # A complete dataset has front and side RGB-D.  During single-arm
        # commissioning the side D455 is intentionally absent, so only front
        # RGB-D is mandatory; any configured side camera must still be RGB-D.
        # A side D455 is optional during staged hardware bring-up, including
        # dual-arm teleoperation.  If it is configured it must be RGB-D; a
        # complete production recording should include it.
        for camera in ("front",):
            if camera not in config["cameras"] or "depth_topic" not in config["cameras"][camera]:
                raise ValueError(f"{camera} must specify rgb_topic and depth_topic")
        if "side" in config["cameras"] and "depth_topic" not in config["cameras"]["side"]:
            raise ValueError("side must specify rgb_topic and depth_topic")
        expected_wrist_cameras = 1 if mode == "single" else 2
        actual_wrist_cameras = [name for name in config["cameras"] if name.endswith("wrist")]
        if len(actual_wrist_cameras) != expected_wrist_cameras:
            raise ValueError(f"mode={mode} requires {expected_wrist_cameras} '*_wrist' RGB camera(s)")
        for name, camera in config["cameras"].items():
            if "rgb_topic" not in camera:
                raise ValueError(f"camera {name} missing rgb_topic")
        for name, arm in config["arms"].items():
            if not arm.get("joint_topic") or not arm.get("joint_order"):
                raise ValueError(f"arm {name} requires joint_topic and non-empty joint_order")
        for name, arm in config["arms"].items():
            for field in ("commanded_action_topic", "gripper_position_topic", "gripper_command_topic"):
                if not arm.get(field):
                    raise ValueError(f"arm {name} missing {field}")
        config.setdefault("max_skew_sec", config.get("sync_slop_sec", 0.05))
        config.setdefault("max_joint_interp_gap_sec", 0.1)
        config.setdefault("source_buffer_size", 30)
        config.setdefault("writer_queue_size", 30)
        config.setdefault("rgb_storage", "raw_lzf")
        config.setdefault("jpeg_quality", 90)
        if config["rgb_storage"] not in {"raw_lzf", "jpeg"}:
            raise ValueError("rgb_storage must be raw_lzf or jpeg")
        if not 1 <= int(config["jpeg_quality"]) <= 100:
            raise ValueError("jpeg_quality must be in [1,100]")
        return config

    def _next_episode_number(self) -> int:
        if not self.output_dir.exists():
            return 0
        numbers = []
        for path in self.output_dir.glob("episode_*"):
            match = re.fullmatch(r"episode_(\d+)\.(?:hdf5|inprogress)", path.name)
            if match:
                numbers.append(int(match.group(1)))
        return max(numbers, default=-1) + 1

    @staticmethod
    def _stamp_ns(message: Any) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _joint_positions(self, arm_name: str, message: JointState) -> np.ndarray:
        positions = dict(zip(message.name, message.position))
        ordered_names = self.config["arms"][arm_name]["joint_order"]
        missing = [name for name in ordered_names if name not in positions]
        if missing:
            raise ValueError(f"{arm_name} JointState missing configured joints: {missing}")
        return np.asarray([positions[name] for name in ordered_names], dtype=np.float32)

    def _joint_velocity(self, arm_name: str, message: JointState) -> np.ndarray:
        if len(message.velocity) != len(message.name):
            return np.full(len(self.config["arms"][arm_name]["joint_order"]), np.nan, np.float32)
        values = dict(zip(message.name, message.velocity))
        return np.asarray([values[name] for name in self.config["arms"][arm_name]["joint_order"]], np.float32)

    def _buffer_header_message(self, key: str, message: Any) -> None:
        stamp = self._stamp_ns(message)
        if stamp > 0:
            self._buffers[key].append((stamp, message))

    def _buffer_arrival_message(self, key: str, message: Any) -> None:
        self._buffers[key].append((self.get_clock().now().nanoseconds, message))

    def _nearest(self, key: str, target: int) -> tuple[int, Any]:
        values = self._buffers[key]
        if not values:
            raise LookupError(f"no data for {key}")
        result = min(values, key=lambda item: abs(item[0] - target))
        if abs(result[0] - target) > int(float(self.config["max_skew_sec"]) * 1e9):
            raise LookupError(f"{key} skew is {abs(result[0]-target)/1e6:.1f} ms")
        return result

    def _interpolated_joint(self, arm: str, target: int) -> tuple[np.ndarray, np.ndarray, int, int]:
        values = list(self._buffers[f"joint:{arm}"])
        before = [item for item in values if item[0] <= target]
        after = [item for item in values if item[0] >= target]
        if before and after:
            left, right = before[-1], after[0]
            gap = right[0] - left[0]
            if gap <= int(float(self.config["max_joint_interp_gap_sec"]) * 1e9):
                alpha = 0.0 if gap == 0 else (target - left[0]) / gap
                q0, q1 = self._joint_positions(arm, left[1]), self._joint_positions(arm, right[1])
                v0, v1 = self._joint_velocity(arm, left[1]), self._joint_velocity(arm, right[1])
                return q0 * (1-alpha) + q1 * alpha, v0 * (1-alpha) + v1 * alpha, left[0], right[0]
        stamp, message = self._nearest(f"joint:{arm}", target)
        return self._joint_positions(arm, message), self._joint_velocity(arm, message), stamp, stamp

    def _aggregate_latest_reference(self) -> None:
        reference = self._buffers.get("image:front:rgb")
        if not reference:
            return
        target = reference[-1][0]
        if target == self._last_reference_stamp:
            return
        if not self.recording or self.writer.failed:
            self._last_reference_stamp = target
            return
        try:
            images: dict[str, dict[str, np.ndarray]] = {}
            source_stamps: dict[str, int] = {}
            for camera, stream in self._stream_keys:
                stamp, message = self._nearest(f"image:{camera}:{stream}", target)
                # Explicit rgb8 conversion guarantees the stored channel order is RGB.
                encoding = "rgb8" if stream == "rgb" else "passthrough"
                image = self.bridge.imgmsg_to_cv2(message, desired_encoding=encoding)
                images.setdefault(camera, {})[stream] = np.ascontiguousarray(image).copy()
                source_stamps[f"{camera}_{stream}"] = stamp
            positions, velocities, actions, grippers, gripper_commands = [], [], [], [], []
            for arm in self.arm_order:
                q, qd, before, after = self._interpolated_joint(arm, target)
                positions.append(q)
                velocities.append(qd)
                source_stamps[f"{arm}_joint_before"] = before
                source_stamps[f"{arm}_joint_after"] = after
                action_stamp, action_msg = self._nearest(f"action:{arm}", target)
                action = np.asarray(action_msg.data, np.float32)
                if action.shape != q.shape:
                    raise ValueError(f"{arm} commanded action shape {action.shape}, expected {q.shape}")
                actions.append(action)
                source_stamps[f"{arm}_commanded_action"] = action_stamp
                grip_stamp, grip = self._nearest(f"gripper:{arm}", target)
                grip_cmd_stamp, grip_cmd = self._nearest(f"gripper_command:{arm}", target)
                grippers.append(grip.data)
                gripper_commands.append(grip_cmd.data)
                source_stamps[f"{arm}_gripper"] = grip_stamp
                source_stamps[f"{arm}_gripper_command"] = grip_cmd_stamp
            sample = Sample(
                timestamp_ns=target,
                source_timestamps_ns=source_stamps,
                images=images,
                qpos=np.concatenate(positions),
                qvel=np.concatenate(velocities),
                commanded_action=np.concatenate(actions),
                gripper_position=np.asarray(grippers, np.float32),
                gripper_command=np.asarray(gripper_commands, np.float32),
            )
            self._last_reference_stamp = target
            self.synced_frames += 1
            if self.writer.put_sample(sample):
                self.enqueued_frames += 1
            else:
                self.dropped_frames += 1
        except LookupError as exc:
            # Other USB streams commonly arrive a few milliseconds after front.
            # Retry this reference until the allowed skew window has elapsed.
            age_ns = self.get_clock().now().nanoseconds - target
            if age_ns <= int(float(self.config["max_skew_sec"]) * 1e9):
                return
            self._last_reference_stamp = target
            self.dropped_frames += 1
            self.get_logger().error(f"Dropped synchronized frame: {exc}")
        except Exception as exc:
            self._last_reference_stamp = target
            self.dropped_frames += 1
            self.get_logger().error(f"Dropped synchronized frame: {exc}")

    def _record_command(self, message: Bool) -> None:
        self._set_recording(message.data, origin="ROS")

    def _set_recording(self, enabled: bool, origin: str) -> None:
        if enabled == self.recording:
            return
        if self.writer.failed:
            if not enabled:
                self.recording = False
            self.get_logger().error(f"Cannot record: writer failed: {self.writer.failed!r}")
            return
        self.recording = enabled
        if enabled:
            path = self.output_dir / f"episode_{self._episode_number:05d}.hdf5"
            self._episode_number += 1
            joint_names = [
                f"{arm}/{joint}"
                for arm in self.arm_order
                for joint in self.config["arms"][arm]["joint_order"]
            ]
            metadata = {
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "mode": self.config["mode"],
                "qpos_joint_names": json.dumps(joint_names),
                "camera_names": json.dumps(self.camera_order),
                "action_joint_names": json.dumps(joint_names),
                "config_yaml": yaml.safe_dump(self.config, sort_keys=True),
            }
            self.writer.put_control(StartEpisode(path, metadata))
            self.get_logger().info(f"{origin}: start requested: {path.name}")
        else:
            self.writer.put_control(EpisodeWriter.STOP)
            self.get_logger().info(f"{origin}: stop requested")

    def request_keyboard_command(self, command: str) -> None:
        self._ui_commands.put(command)

    def _process_ui_commands(self) -> None:
        while not self._ui_commands.empty():
            command = self._ui_commands.get()
            if command == "toggle":
                self._set_recording(not self.recording, origin="keyboard")
            elif command == "quit":
                if self.recording:
                    self._set_recording(False, origin="keyboard")
                rclpy.shutdown()

    def _report_stats(self) -> None:
        if self.writer.failed and self.recording:
            self.recording = False
        self.get_logger().info(
            f"recording={self.recording} synced={self.synced_frames} "
            f"queued={self.enqueued_frames} dropped={self.dropped_frames}"
        )
        status = String()
        status.data = json.dumps({
            "recording": self.recording,
            "synced_frames": self.synced_frames,
            "dropped_frames": self.dropped_frames,
            "writer_queue": self.writer.events.qsize(),
            "writer_error": repr(self.writer.failed) if self.writer.failed else None,
        })
        self.status_pub.publish(status)

    def close_writer(self) -> None:
        if self.recording:
            self._set_recording(False, origin="shutdown")
        self.writer.put_control(EpisodeWriter.SHUTDOWN)
        self.writer.join(timeout=30)
        if self.writer.is_alive():
            self.get_logger().error("writer did not exit within 30 seconds; .inprogress file was retained")


def _keyboard_loop(node: DatasetRecorder) -> None:
    """Optional terminal controls; ROS topic control remains the normal Quest path."""
    if not sys.stdin.isatty():
        return
    print("[keyboard] r = start/stop episode, q = stop recorder", flush=True)
    for line in sys.stdin:
        key = line.strip().lower()
        if key == "r":
            node.request_keyboard_command("toggle")
        elif key == "q":
            node.request_keyboard_command("quit")
            return


def _embed_hardware_metadata(config: dict[str, Any], hardware_path: Path) -> None:
    hardware = yaml.safe_load(hardware_path.read_text(encoding="utf-8"))
    config["hardware_config_yaml"] = yaml.safe_dump(hardware, sort_keys=True)
    embedded: dict[str, str] = {}
    for arm, arm_config in hardware.get("arms", {}).items():
        for field in ("front_calibration", "side_calibration", "tcp_calibration"):
            value = arm_config.get(field)
            if not value:
                continue
            path = Path(value).expanduser()
            if not path.is_absolute():
                cwd_candidate = path.resolve()
                path = cwd_candidate if cwd_candidate.exists() else (hardware_path.parent / path).resolve()
            if not path.is_file():
                raise FileNotFoundError(f"hardware metadata references missing calibration: {path}")
            embedded[f"{arm}/{field}"] = path.read_text(encoding="utf-8")
    config["calibration_yaml"] = embedded


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Record synchronized UR/Realsense demonstrations to HDF5.")
    parser.add_argument("--config", required=True, type=Path, help="YAML topic and joint-order configuration")
    parser.add_argument("--output-dir", required=True, type=Path, help="directory for episode_XXXXX.hdf5")
    parser.add_argument("--hardware-config", type=Path, help="embed robot/TCP/camera calibration metadata")
    plain_args = remove_ros_args(args=argv)
    args = parser.parse_args(plain_args[1:])
    with args.config.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError("configuration root must be a YAML mapping")
    if args.hardware_config:
        _embed_hardware_metadata(config, args.hardware_config.expanduser().resolve())

    rclpy.init(args=argv)
    node = DatasetRecorder(config, args.output_dir)
    keyboard = threading.Thread(target=_keyboard_loop, args=(node,), daemon=True)
    keyboard.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_writer()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
