"""Synchronized ROS 2 recorder for real UR robot observations.

Callbacks match message references and robot state in timestamp order. Image
conversion, resizing and disk I/O run in the bounded background writer.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import os
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import h5py
import numpy as np
import yaml
import cv2

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, String

from .episode_paths import episode_output_dir
from .teleop_core import ControllerGate, ControllerState


@dataclass
class BufferedImage:
    """Retain the ROS image buffer until the writer is ready to convert it."""

    message: Any
    stream: str
    record_size: tuple[int, int] | None = None


def prepare_image(image: np.ndarray, stream: str, record_size: Any = None) -> np.ndarray:
    """Center-crop to the configured aspect ratio, preserving depth values."""
    if record_size is None:
        return image
    if image.ndim not in (2, 3):
        raise ValueError(f"{stream} image must be HxW or HxWxC, got {image.shape}")
    target_w, target_h = map(int, record_size)
    source_h, source_w = image.shape[:2]
    if (source_w, source_h) == (target_w, target_h):
        return image
    source_ratio, target_ratio = source_w / source_h, target_w / target_h
    if source_ratio > target_ratio:
        crop_w = round(source_h * target_ratio)
        left = (source_w - crop_w) // 2
        image = image[:, left:left + crop_w]
    elif source_ratio < target_ratio:
        crop_h = round(source_w / target_ratio)
        top = (source_h - crop_h) // 2
        image = image[top:top + crop_h, :]
    interpolation = cv2.INTER_NEAREST if stream == "depth" else cv2.INTER_AREA
    return cv2.resize(image, (target_w, target_h), interpolation=interpolation)


@dataclass
class Sample:
    """One front-camera-clocked robot observation."""

    timestamp_ns: int
    source_timestamps_ns: dict[str, int]
    images: dict[str, dict[str, np.ndarray | BufferedImage]]
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

    def __init__(self, max_queue_size: int, rgb_storage: str = "raw", jpeg_quality: int = 90):
        super().__init__(name="hdf5-writer", daemon=True)
        if max_queue_size <= 0:
            raise ValueError("max_queue_size must be positive")
        # Limit samples, while allowing STOP/SHUTDOWN to enter the same FIFO
        # immediately even under disk backpressure. Episode order is preserved.
        self.events: queue.Queue[StartEpisode | Sample | object] = queue.Queue()
        self._sample_slots = threading.BoundedSemaphore(max_queue_size)
        self._bridge = CvBridge()
        self._datasets: dict[str, h5py.Dataset] = {}
        self.rgb_storage = rgb_storage
        self.jpeg_quality = int(jpeg_quality)
        self._error: Exception | None = None
        self.start()

    @property
    def failed(self) -> Exception | None:
        return self._error

    def put_control(self, item: StartEpisode | object) -> bool:
        """Control messages must not be dropped; they preserve episode boundaries."""
        if not self.is_alive() or self.failed:
            return False
        self.events.put_nowait(item)
        return True

    def put_sample(self, sample: Sample) -> bool:
        if self.failed or not self.is_alive() or not self._sample_slots.acquire(blocking=False):
            return False
        self.events.put_nowait(sample)
        return True

    def _next_event(self) -> StartEpisode | Sample | object:
        event = self.events.get()
        if isinstance(event, Sample):
            self._sample_slots.release()
        return event

    def _prepare_sample(self, sample: Sample) -> None:
        for streams in sample.images.values():
            for stream, value in streams.items():
                if isinstance(value, BufferedImage):
                    image = self._bridge.imgmsg_to_cv2(
                        value.message, desired_encoding="rgb8" if stream == "rgb" else "passthrough"
                    )
                    # NumPy keeps the backing buffer alive; ROS messages are
                    # immutable after enqueue, so an extra full-frame copy is unnecessary.
                    streams[stream] = np.ascontiguousarray(prepare_image(image, stream, value.record_size))

    def run(self) -> None:
        file: h5py.File | None = None
        final_path: Path | None = None
        pending: Sample | None = None
        count = 0
        try:
            while True:
                event = self._next_event()
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
                    self._datasets = {}
                    for key, value in event.metadata.items():
                        file.attrs[key] = value
                    file.attrs["complete"] = False
                    file.attrs["action_definition"] = "commanded target_q - actual_q (radians)"
                    file.attrs["executed_action_definition"] = "qpos[t+1] - qpos[t] (radians)"
                    file.attrs["final_action"] = "action_valid=false; executed_action=zeros"
                    file.attrs["sample_alignment"] = (
                        "Every leading dataset index i contains images, interpolated robot state, "
                        "and commanded action from one synchronized Sample."
                    )
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
                self._prepare_sample(event)
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
        self._assert_sample_alignment(file, count)
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
    def _assert_sample_alignment(file: h5py.File, expected_count: int) -> None:
        """Refuse to mark complete an episode with mismatched sample indices."""
        mismatched: list[str] = []

        def visit(name: str, value: Any) -> None:
            if isinstance(value, h5py.Dataset) and (not value.shape or value.shape[0] != expected_count):
                actual = "scalar" if not value.shape else str(value.shape[0])
                mismatched.append(f"{name}={actual}")

        file.visititems(visit)
        if mismatched:
            raise RuntimeError(
                f"refusing incomplete/misaligned episode: expected {expected_count} samples; "
                + ", ".join(mismatched)
            )

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
            self._datasets = {}

            def cache_dataset(name: str, value: Any) -> None:
                if isinstance(value, h5py.Dataset):
                    self._datasets[name] = value

            file.visititems(cache_dataset)
        datasets = self._datasets
        self._resize_write(datasets["timestamps_ns"], np.uint64(sample.timestamp_ns))
        self._resize_write(datasets["observations/qpos"], sample.qpos.astype(np.float32, copy=False))
        self._resize_write(datasets["observations/qvel"], sample.qvel.astype(np.float32, copy=False))
        self._resize_write(datasets["observations/gripper_position"], sample.gripper_position.astype(np.float32, copy=False))
        self._resize_write(datasets["action"], sample.commanded_action.astype(np.float32, copy=False))
        self._resize_write(datasets["executed_action"], executed_action.astype(np.float32, copy=False))
        self._resize_write(datasets["gripper_action"], sample.gripper_command.astype(np.float32, copy=False))
        self._resize_write(datasets["action_valid"], np.bool_(action_valid))
        self._resize_write(datasets["is_terminal"], np.bool_(terminal))
        for source, timestamp in sample.source_timestamps_ns.items():
            self._resize_write(datasets[f"source_timestamps_ns/{source}"], np.uint64(timestamp))
        for camera, streams in sample.images.items():
            for stream, image in streams.items():
                dataset = datasets[f"observations/images/{camera}/{stream}"]
                if stream == "rgb" and dataset.attrs.get("storage") == "jpeg":
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
    def _create_dataset(
        file: h5py.File, name: str, shape: tuple[int, ...], dtype: Any,
        compression: str | None = "lzf",
    ) -> None:
        file.create_dataset(
            name,
            shape=(0, *shape),
            maxshape=(None, *shape),
            dtype=dtype,
            chunks=(1 if len(shape) >= 2 else 64, *shape),
            compression=compression,
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
                    compression = None if stream == "rgb" and self.rgb_storage == "raw" else "lzf"
                    self._create_dataset(file, name, image.shape, image.dtype, compression)
                dataset = file[f"observations/images/{camera}/{stream}"]
                dataset.attrs["encoding"] = (
                    "jpeg bytes decoded as RGB8"
                    if stream == "rgb" and self.rgb_storage == "jpeg"
                    else "rgb8" if stream == "rgb" else "source passthrough"
                )
                dataset.attrs["storage"] = self.rgb_storage if stream == "rgb" else "raw_lzf"
                dataset.attrs["original_shape"] = image.shape
                dataset.attrs["unit"] = "" if stream == "rgb" else "driver-defined (usually mm for 16UC1)"


class FolderEpisodeWriter(EpisodeWriter):
    """Legacy folder writer: RGB JPEG, lossless depth NPY and JSONL state."""

    @staticmethod
    def _image_directory(camera: str, stream: str) -> str:
        if camera == "front":
            return f"{'rgb' if stream == 'rgb' else 'depth'}_front_cam"
        if camera == "side":
            return f"{'rgb' if stream == 'rgb' else 'depth'}_side_cam"
        if camera == "left_wrist" and stream == "rgb":
            return "rgb_wrist_left_cam"
        if camera == "right_wrist" and stream == "rgb":
            return "rgb_wrist_right_cam"
        return f"{stream}_{camera}_cam"

    @staticmethod
    def _state_record(
        index: int, sample: Sample, executed_action: np.ndarray, action_valid: bool, terminal: bool
    ) -> dict[str, Any]:
        return {
            "index": index,
            "timestamp_ns": int(sample.timestamp_ns),
            "source_timestamps_ns": {key: int(value) for key, value in sample.source_timestamps_ns.items()},
            "qpos": sample.qpos.astype(float).tolist(),
            "qvel": sample.qvel.astype(float).tolist(),
            "commanded_action": sample.commanded_action.astype(float).tolist(),
            "executed_action": executed_action.astype(float).tolist(),
            "gripper_position": sample.gripper_position.astype(float).tolist(),
            "gripper_command": sample.gripper_command.astype(float).tolist(),
            "action_valid": bool(action_valid),
            "is_terminal": bool(terminal),
        }

    def _write_frame(
        self,
        directory: Path,
        state_file: Any,
        sample: Sample,
        executed_action: np.ndarray,
        action_valid: bool,
        terminal: bool,
        index: int,
        image_dirs: set[str],
    ) -> None:
        for camera, streams in sample.images.items():
            for stream, image in streams.items():
                image_dir = self._image_directory(camera, stream)
                image_dirs.add(image_dir)
                target = directory / image_dir
                target.mkdir(exist_ok=True)
                if stream == "rgb":
                    # RGB remains directly inspectable in ordinary image tools.
                    # OpenCV writes BGR while the recorder stores RGB arrays.
                    ok = cv2.imwrite(
                        str(target / f"{index:06d}.jpg"),
                        np.ascontiguousarray(image[..., ::-1]),
                        [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                    )
                    if not ok:
                        raise RuntimeError(f"failed to write RGB JPEG: {camera}/{index:06d}")
                else:
                    # Depth remains a lossless uint16 NPY array.
                    np.save(target / f"{index:06d}.npy", image, allow_pickle=False)
        state_file.write(json.dumps(self._state_record(index, sample, executed_action, action_valid, terminal)) + "\n")

    def _close_folder_episode(
        self,
        directory: Path,
        final_path: Path,
        state_file: Any,
        metadata: dict[str, Any],
        count: int,
        image_dirs: set[str],
    ) -> None:
        state_file.flush()
        state_file.close()
        for image_dir in image_dirs:
            actual = len(list((directory / image_dir).glob("*.npy"))) + len(list((directory / image_dir).glob("*.jpg")))
            if actual != count:
                raise RuntimeError(f"folder episode misalignment: {image_dir} has {actual}, expected {count}")
        manifest = dict(metadata)
        manifest.update({
            "complete": True,
            "num_samples": count,
            "storage_backend": "folder_npy",
            "image_format": f"RGB JPEG quality {self.jpeg_quality}; depth NPY (lossless uint16)",
            "state_format": "robot_state.jsonl; line i matches every image file i",
        })
        (directory / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
        if count:
            os.replace(directory, final_path)
            print(f"[folder-writer] SAVED {final_path} ({count} samples)", flush=True)
        else:
            for child in directory.iterdir():
                child.unlink()
            directory.rmdir()
            print("[folder-writer] empty episode discarded", flush=True)

    def run(self) -> None:
        directory: Path | None = None
        final_path: Path | None = None
        state_file: Any = None
        metadata: dict[str, Any] = {}
        pending: Sample | None = None
        count = 0
        image_dirs: set[str] = set()
        try:
            while True:
                event = self._next_event()
                if event is self.SHUTDOWN:
                    if directory is not None and final_path is not None and state_file is not None:
                        if pending is not None:
                            self._write_frame(directory, state_file, pending, np.zeros_like(pending.qpos), False, True, count, image_dirs)
                            count += 1
                        self._close_folder_episode(directory, final_path, state_file, metadata, count, image_dirs)
                    return
                if isinstance(event, StartEpisode):
                    if directory is not None:
                        raise RuntimeError("overlapping folder episodes are not supported")
                    final_path = event.path
                    directory = event.path.with_name(event.path.name + ".inprogress")
                    directory.mkdir(parents=True, exist_ok=False)
                    metadata = dict(event.metadata)
                    metadata.update({"complete": False, "storage_backend": "folder_npy"})
                    (directory / "manifest.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
                    state_file = (directory / "robot_state.jsonl").open("w", encoding="utf-8")
                    pending, count, image_dirs = None, 0, set()
                    print(f"[folder-writer] RECORDING {final_path}", flush=True)
                    continue
                if event is self.STOP:
                    if directory is not None and final_path is not None and state_file is not None:
                        if pending is not None:
                            self._write_frame(directory, state_file, pending, np.zeros_like(pending.qpos), False, True, count, image_dirs)
                            count += 1
                        self._close_folder_episode(directory, final_path, state_file, metadata, count, image_dirs)
                    directory = final_path = state_file = None
                    pending, count, image_dirs = None, 0, set()
                    continue
                if not isinstance(event, Sample) or directory is None or state_file is None:
                    continue
                self._prepare_sample(event)
                if pending is None:
                    pending = event
                else:
                    self._write_frame(directory, state_file, pending, event.qpos - pending.qpos, True, False, count, image_dirs)
                    count += 1
                    pending = event
        except Exception as exc:
            self._error = exc
            if state_file is not None:
                try:
                    state_file.flush()
                    state_file.close()
                except Exception:
                    pass
            print(f"[folder-writer] ERROR: {exc!r}; retained incomplete folder", file=sys.stderr, flush=True)


class DatasetRecorder(Node):
    """Camera-clocked nearest-frame aggregator with interpolated robot state."""

    def __init__(self, config: dict[str, Any], output_dir: Path):
        super().__init__("quest3_dataset_recorder")
        self.config = self._validate_config(config)
        self.output_dir = episode_output_dir(
            output_dir, self.config["mode"], self.config["task_description"]
        )
        self.bridge = CvBridge()
        writer_type = FolderEpisodeWriter if self.config["storage_backend"] == "folder_npy" else EpisodeWriter
        writer_quality = self.config["folder_rgb_jpeg_quality"] if self.config["storage_backend"] == "folder_npy" else self.config["jpeg_quality"]
        self.writer = writer_type(
            int(self.config["writer_queue_size"]),
            self.config["rgb_storage"],
            int(writer_quality),
        )
        self.recording = False
        self.recording_started_ns: int | None = None
        self.recording_interrupted_reason: str | None = None
        self.synced_frames = 0
        self.enqueued_frames = 0
        self.dropped_frames = 0
        self.drop_reasons: Counter[str] = Counter()
        self.last_sync_error: str | None = None
        self._reference_wait_started: tuple[int, int] | None = None
        self._ui_commands: queue.SimpleQueue[str] = queue.SimpleQueue()
        self._episode_number = self._next_episode_number()
        self._last_reference_stamp = -1
        self.controller_gate = ControllerGate(
            ("left",) if self.config["mode"] == "single" else ("left", "right"),
            float(self.config.get("controller_timeout_sec", 0.25)),
        )
        buffer_size = int(self.config["source_buffer_size"])
        sensor_qos = QoSProfile(depth=buffer_size, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._buffers: dict[str, deque[tuple[int, Any]]] = {}
        self._subscriptions = []

        self.record_sub = self.create_subscription(
            Bool, self.config["record_topic"], self._record_command, 10
        )
        self.controller_sub = self.create_subscription(
            String, self.config.get("controller_topic", "/quest3/controllers"), self._controller_state, 10
        )
        self.safety_sub = self.create_subscription(
            String, "/quest3/safety_event", self._safety_event, 10
        )
        self.status_pub = self.create_publisher(String, "/quest3/system_status", 10)
        self.create_timer(0.005, self._aggregate_latest_reference)
        self.create_timer(0.1, self._process_ui_commands)
        self.create_timer(0.05, self._check_controller_tracking)
        self.create_timer(2.0, self._report_stats)

        self.camera_order = list(self.config["cameras"])
        self.arm_order = list(self.config["arms"])
        self._stream_keys: list[tuple[str, str]] = []
        for name in self.camera_order:
            camera = self.config["cameras"][name]
            key = f"image:{name}:rgb"
            self._buffers[key] = deque(maxlen=buffer_size)
            self._subscriptions.append(self.create_subscription(
                Image, camera["rgb_topic"], lambda m, k=key: self._buffer_header_message(k, m), sensor_qos
            ))
            self._stream_keys.append((name, "rgb"))
            if self.config["record_depth"] and "depth_topic" in camera:
                key = f"image:{name}:depth"
                self._buffers[key] = deque(maxlen=buffer_size)
                self._subscriptions.append(self.create_subscription(
                    Image, camera["depth_topic"], lambda m, k=key: self._buffer_header_message(k, m), sensor_qos
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
                self.create_subscription(JointState, arm["joint_topic"], lambda m, k=joint_key: self._buffer_header_message(k, m), sensor_qos),
                self.create_subscription(Float64MultiArray, arm["commanded_action_topic"], lambda m, k=action_key: self._buffer_arrival_message(k, m), 20),
                self.create_subscription(Float32, arm["gripper_position_topic"], lambda m, k=grip_key: self._buffer_arrival_message(k, m), 10),
                self.create_subscription(Float32, arm["gripper_command_topic"], lambda m, k=grip_command_key: self._buffer_arrival_message(k, m), 10),
            ])
        self.get_logger().info(
            f"READY mode={self.config['mode']}, reference=front/rgb, "
            f"record_depth={self.config['record_depth']}, "
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
        config.setdefault("record_depth", False)
        if not isinstance(config["record_depth"], bool):
            raise ValueError("record_depth must be true or false")
        if "front" not in config["cameras"]:
            raise ValueError("front must specify rgb_topic")
        # Topics remain in the YAML as opt-in sources, but RGB-only recording
        # must neither subscribe to depth nor wait for it during aggregation.
        if config["record_depth"]:
            for camera in ("front", "side"):
                if camera in config["cameras"] and not config["cameras"][camera].get("depth_topic"):
                    raise ValueError(f"{camera} must specify depth_topic when record_depth=true")
        expected_wrist_cameras = 1 if mode == "single" else 2
        actual_wrist_cameras = [name for name in config["cameras"] if name.endswith("wrist")]
        if len(actual_wrist_cameras) != expected_wrist_cameras:
            raise ValueError(f"mode={mode} requires {expected_wrist_cameras} '*_wrist' RGB camera(s)")
        for name, camera in config["cameras"].items():
            if "rgb_topic" not in camera:
                raise ValueError(f"camera {name} missing rgb_topic")
            if "record_size" in camera:
                size = camera["record_size"]
                if (
                    not isinstance(size, (list, tuple))
                    or len(size) != 2
                    or any(not isinstance(value, int) or value <= 0 for value in size)
                ):
                    raise ValueError(f"camera {name} record_size must be [width, height] of positive integers")
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
        config.setdefault("aggregate_batch_size", 4)
        for field in ("source_buffer_size", "writer_queue_size", "aggregate_batch_size"):
            if type(config[field]) is not int or config[field] <= 0:
                raise ValueError(f"{field} must be a positive integer")
        for field in ("max_skew_sec", "max_joint_interp_gap_sec"):
            if not np.isfinite(float(config[field])) or float(config[field]) <= 0:
                raise ValueError(f"{field} must be finite and positive")
        config.setdefault("rgb_storage", "raw")
        config.setdefault("jpeg_quality", 90)
        config.setdefault("folder_rgb_jpeg_quality", config["jpeg_quality"])
        config.setdefault("storage_backend", "hdf5")
        if config["rgb_storage"] not in {"raw", "raw_lzf", "jpeg"}:
            raise ValueError("rgb_storage must be raw, raw_lzf or jpeg")
        if not 1 <= int(config["jpeg_quality"]) <= 100:
            raise ValueError("jpeg_quality must be in [1,100]")
        if not 1 <= int(config["folder_rgb_jpeg_quality"]) <= 100:
            raise ValueError("folder_rgb_jpeg_quality must be in [1,100]")
        if config["storage_backend"] not in {"hdf5", "folder_npy"}:
            raise ValueError("storage_backend must be 'hdf5' or 'folder_npy'")
        return config

    def _next_episode_number(self) -> int:
        if not self.output_dir.exists():
            return 0
        numbers = []
        for path in self.output_dir.glob("episode_*"):
            match = re.fullmatch(r"episode_(\d+)(?:\.hdf5|\.inprogress)?", path.name)
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
            values = self._buffers[key]
            if values and stamp <= values[-1][0]:
                return  # Ignore duplicate/late packets; interpolation requires timestamp order.
            if (key == "image:front:rgb" and self.recording and len(values) == values.maxlen
                    and values[0][0] > self._last_reference_stamp):
                self._drop_frame("reference_overflow")
                self._last_reference_stamp = values[0][0]
            values.append((stamp, message))

    def _buffer_arrival_message(self, key: str, message: Any) -> None:
        self._buffers[key].append((self.get_clock().now().nanoseconds, message))

    def _prepare_image_for_recording(self, camera: str, stream: str, image: np.ndarray) -> np.ndarray:
        return prepare_image(image, stream, self.config["cameras"][camera].get("record_size"))

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

    def _drop_frame(self, reason: str) -> None:
        self.dropped_frames += 1
        self.drop_reasons[reason] += 1

    def _aggregate_latest_reference(self, finalize: bool = False) -> None:
        """Drain oldest references first, bounding callback work between subscriptions."""
        reference = self._buffers.get("image:front:rgb")
        if not reference:
            return
        if not self.recording or self.writer.failed:
            self._last_reference_stamp = reference[-1][0]
            self._reference_wait_started = None
            return
        pending = [(stamp, message) for stamp, message in reference if stamp > self._last_reference_stamp]
        limit = len(pending) if finalize else self.config["aggregate_batch_size"]
        for target, message in pending[:limit]:
            if not self._aggregate_reference(target, message, finalize):
                break

    def _aggregate_reference(self, target: int, reference_message: Any, finalize: bool) -> bool:
        try:
            images: dict[str, dict[str, np.ndarray | BufferedImage]] = {}
            source_stamps: dict[str, int] = {}
            for camera, stream in self._stream_keys:
                stamp, message = ((target, reference_message) if (camera, stream) == ("front", "rgb")
                                  else self._nearest(f"image:{camera}:{stream}", target))
                size = self.config["cameras"][camera].get("record_size")
                images.setdefault(camera, {})[stream] = BufferedImage(
                    message, stream, tuple(size) if size else None
                )
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
                self._drop_frame("writer_queue_full")
        except LookupError as exc:
            # Wait on the oldest frame even if newer front frames arrive. Use
            # host monotonic time: camera stamps can have a different offset.
            now = time.monotonic_ns()
            if self._reference_wait_started is None or self._reference_wait_started[0] != target:
                self._reference_wait_started = (target, now)
            waited = now - self._reference_wait_started[1]
            if not finalize and waited < int(float(self.config["max_skew_sec"]) * 1e9):
                return False
            self._last_reference_stamp = target
            self._drop_frame("sync_timeout")
            self.last_sync_error = str(exc)
        except Exception as exc:
            self._last_reference_stamp = target
            self._drop_frame("invalid_sample")
            self.get_logger().error(f"Dropped synchronized frame: {exc}")
        self._reference_wait_started = None
        return True

    def _record_command(self, message: Bool) -> None:
        self._set_recording(message.data, origin="ROS")

    def _controller_state(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        states = {role: ControllerState.from_dict(data.get(role)) for role in self.controller_gate.roles}
        now_ns = self.get_clock().now().nanoseconds
        self.controller_gate.observe(states, now_ns)
        self._check_controller_tracking()

    def _check_controller_tracking(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.recording and not self.controller_gate.ready(now_ns):
            missing = ", ".join(self.controller_gate.missing(now_ns))
            self.recording_interrupted_reason = f"手柄追踪中断（{missing}）；录制已停止"
            self._set_recording(False, origin="CONTROLLER TRACKING LOST")
            self._publish_status()

    def _safety_event(self, message: String) -> None:
        """Finish an active episode when RTDE reports loss of robot control."""
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(value, dict):
            return
        event = str(value.get("event", "robot safety event"))
        arm = str(value.get("arm", "robot"))
        descriptions = {
            "emergency_stop": "急停已触发",
            "protective_stop": "保护停机",
            "rtde_control_lost": "RTDE 控制脚本已停止",
            "rtde_connection_lost": "RTDE 连接已丢失",
        }
        arm_label = {"left_arm": "左臂", "right_arm": "右臂"}.get(arm, arm)
        reason = f"{arm_label}：{descriptions.get(event, event)}；录制已安全中断并保存"
        if not self.recording:
            return
        self.recording_interrupted_reason = reason
        self._set_recording(False, origin=f"SAFETY INTERRUPT ({reason})")
        # Do not wait for the two-second statistics timer before the headset
        # learns that the episode has been safely finalized.
        self._publish_status()

    def _set_recording(self, enabled: bool, origin: str) -> None:
        if enabled == self.recording:
            return
        now_ns = self.get_clock().now().nanoseconds
        if enabled and not self.controller_gate.ready(now_ns):
            missing = ", ".join(self.controller_gate.missing(now_ns))
            self.recording_interrupted_reason = f"手柄未就绪（{missing}）；未开始录制"
            self.get_logger().error(self.recording_interrupted_reason)
            self._publish_status()
            return
        if self.writer.failed:
            if not enabled:
                self.recording = False
            self.get_logger().error(f"Cannot record: writer failed: {self.writer.failed!r}")
            return
        if not enabled:
            self._aggregate_latest_reference(finalize=True)
        self.recording = enabled
        if enabled:
            # Include the current front frame, excluding pre-recording history.
            reference = self._buffers.get("image:front:rgb")
            if reference:
                self._last_reference_stamp = reference[-1][0] - 1
            self._reference_wait_started = None
            self.recording_interrupted_reason = None
            self.recording_started_ns = self.get_clock().now().nanoseconds
            suffix = ".hdf5" if self.config["storage_backend"] == "hdf5" else ""
            path = self.output_dir / f"episode_{self._episode_number:05d}{suffix}"
            self._episode_number += 1
            joint_names = [
                f"{arm}/{joint}"
                for arm in self.arm_order
                for joint in self.config["arms"][arm]["joint_order"]
            ]
            metadata = {
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "mode": self.config["mode"],
                # Fixed for the recorder process, so every episode made by
                # one teleoperation run has an unambiguous task label.
                "task_description": self.config["task_description"],
                "qpos_joint_names": json.dumps(joint_names),
                "camera_names": json.dumps(self.camera_order),
                "record_depth": self.config["record_depth"],
                "action_joint_names": json.dumps(joint_names),
                "config_yaml": yaml.safe_dump(self.config, sort_keys=True),
            }
            self.writer.put_control(StartEpisode(path, metadata))
            self.get_logger().info(f"{origin}: start requested: {path.name}")
        else:
            self.recording_started_ns = None
            self.writer.put_control(EpisodeWriter.STOP)
            self.get_logger().info(f"{origin}: stop requested")
        self._publish_status()

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
            f"queued={self.enqueued_frames} dropped={self.dropped_frames} "
            f"writer_queue={self.writer.events.qsize()} reasons={dict(self.drop_reasons)} "
            f"last_sync_error={self.last_sync_error}"
        )
        self._publish_status()

    def _publish_status(self) -> None:
        status = String()
        status.data = json.dumps({
            "recording": self.recording,
            "recording_elapsed_sec": (
                0.0
                if self.recording_started_ns is None
                else max(0.0, (self.get_clock().now().nanoseconds - self.recording_started_ns) / 1e9)
            ),
            "synced_frames": self.synced_frames,
            "dropped_frames": self.dropped_frames,
            "enqueued_frames": self.enqueued_frames,
            "drop_reasons": dict(self.drop_reasons),
            "last_sync_error": self.last_sync_error,
            "writer_queue": self.writer.events.qsize(),
            "writer_error": repr(self.writer.failed) if self.writer.failed else None,
            "recording_interrupted": self.recording_interrupted_reason,
            "controller_ready": self.controller_gate.ready(self.get_clock().now().nanoseconds),
            "controller_missing": list(self.controller_gate.missing(self.get_clock().now().nanoseconds)),
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
    if hardware.get("dual_calibration"):
        path = Path(hardware["dual_calibration"]).expanduser()
        if not path.is_absolute():
            path = (hardware_path.parent / path).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"hardware metadata references missing dual calibration: {path}")
        embedded["dual_calibration"] = path.read_text(encoding="utf-8")
    config["calibration_yaml"] = embedded


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Record synchronized UR/RealSense demonstrations.")
    parser.add_argument("--config", required=True, type=Path, help="YAML topic and joint-order configuration")
    parser.add_argument(
        "--output-dir", required=True, type=Path,
        help="base directory for episodes (dual mode adds a task-name subdirectory)",
    )
    parser.add_argument("--hardware-config", type=Path, help="embed robot/TCP/camera calibration metadata")
    parser.add_argument(
        "--record-depth", nargs="?", const="true", choices=("true", "false"),
        help="include configured depth streams; default: record_depth in YAML, or false",
    )
    parser.add_argument(
        "--task-description",
        required=True,
        help="fixed natural-language task label written into every episode of this run",
    )
    plain_args = remove_ros_args(args=argv)
    args = parser.parse_args(plain_args[1:])
    with args.config.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError("configuration root must be a YAML mapping")
    if args.record_depth is not None:
        config["record_depth"] = args.record_depth == "true"
    task_description = args.task_description.strip()
    if not task_description:
        raise ValueError("--task-description must not be empty")
    config["task_description"] = task_description
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
