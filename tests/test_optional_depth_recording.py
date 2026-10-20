from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import h5py
import numpy as np
import pytest
import yaml

pytest.importorskip("rclpy")

from launch import LaunchContext
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray
from quest3_robot_data.recorder import DatasetRecorder, EpisodeWriter, Node


ROOT = Path(__file__).resolve().parents[1]


def recorder_config():
    config = yaml.safe_load((ROOT / "config/dual_arm.yaml").read_text())
    config["task_description"] = "optional depth test"
    return config


def test_depth_is_opt_in_and_rgb_only_configs_are_valid():
    config = recorder_config()
    config.pop("record_depth")
    for camera in config["cameras"].values():
        camera.pop("depth_topic", None)
    assert DatasetRecorder._validate_config(deepcopy(config))["record_depth"] is False
    config["record_depth"] = True
    with pytest.raises(ValueError, match="front.*depth_topic"):
        DatasetRecorder._validate_config(config)


@pytest.mark.parametrize("record_depth", [False, True])
def test_recorder_subscriptions_aggregation_and_hdf5_follow_depth_option(tmp_path, monkeypatch, record_depth):
    config = recorder_config()
    config["record_depth"] = record_depth
    for camera in config["cameras"].values():
        camera["record_size"] = [16, 16]
    subscriptions = []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
    logger = SimpleNamespace(info=lambda message: None, error=lambda message: pytest.fail(message))
    monkeypatch.setattr(Node, "__init__", lambda *args, **kwargs: None)
    monkeypatch.setattr(DatasetRecorder, "create_subscription", lambda self, kind, topic, *args: subscriptions.append(topic))
    monkeypatch.setattr(DatasetRecorder, "create_publisher", lambda *args: SimpleNamespace(publish=lambda message: None))
    monkeypatch.setattr(DatasetRecorder, "create_timer", lambda *args: None)
    monkeypatch.setattr(DatasetRecorder, "get_clock", lambda self: clock)
    monkeypatch.setattr(DatasetRecorder, "get_logger", lambda self: logger)
    node = DatasetRecorder(config, tmp_path)
    try:
        for name in ("front", "side"):
            assert (config["cameras"][name]["depth_topic"] in subscriptions) is record_depth
            assert ((name, "depth") in node._stream_keys) is record_depth
        # Only supply the streams actually selected by the recorder. Default
        # RGB recording must complete even when no depth messages exist.
        for camera, stream in node._stream_keys:
            image = np.ones((16, 16, 3), np.uint8) if stream == "rgb" else np.full((16, 16), 1234, np.uint16)
            message = node.bridge.cv2_to_imgmsg(image, encoding="rgb8" if stream == "rgb" else "16UC1")
            node._buffers[f"image:{camera}:{stream}"].append((1_000_000_000, message))
        for name, arm in config["arms"].items():
            joint = JointState(name=arm["joint_order"], position=[0.0] * 6, velocity=[0.0] * 6)
            for key, message in (
                (f"joint:{name}", joint),
                (f"action:{name}", Float64MultiArray(data=[0.0] * 6)),
                (f"gripper:{name}", Float32(data=0.0)),
                (f"gripper_command:{name}", Float32(data=0.0)),
            ):
                node._buffers[key].append((1_000_000_000, message))
        node.controller_gate.observe({"left": object(), "right": object()}, 1_000_000_000)
        node._set_recording(True, origin="test")
        node._aggregate_latest_reference()
        assert node.enqueued_frames == 1
        node._set_recording(False, origin="test")
    finally:
        node.writer.put_control(EpisodeWriter.SHUTDOWN)
        node.writer.join(timeout=5)
    assert not node.writer.is_alive()
    assert node.writer.failed is None
    with h5py.File(node.output_dir / "episode_00000.hdf5") as file:
        assert bool(file.attrs["record_depth"]) is record_depth
        for camera in config["cameras"]:
            assert file[f"observations/images/{camera}/rgb"].shape[0] == 1
        for camera in ("front", "side"):
            assert (f"observations/images/{camera}/depth" in file) is record_depth
            assert (f"source_timestamps_ns/{camera}_depth" in file) is record_depth


@pytest.mark.parametrize("enable_depth", ["false", "true"])
def test_camera_launch_enables_depth_only_for_opted_in_scene_cameras(tmp_path, monkeypatch, enable_depth):
    spec = importlib.util.spec_from_file_location("optional_depth_camera_launch", ROOT / "launch/realsense_four.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    context = LaunchContext()
    context.launch_configurations.update({
        "hardware_config": str(ROOT / "config/hardware_dual_teleop.yaml"),
        "enable_depth": enable_depth,
    })
    nodes = module.launch_cameras(context)
    assert len(nodes) == 4
    for node in nodes:
        scene = node["namespace"] in {"front_camera", "side_camera"}
        expected = scene and enable_depth == "true"
        assert node["parameters"][0]["enable_depth"] is expected
        assert node["parameters"][0]["align_depth.enable"] is expected
