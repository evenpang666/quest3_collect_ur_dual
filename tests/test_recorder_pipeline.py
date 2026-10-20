"""Regression coverage for queued references, backpressure and lossless storage."""
from collections import Counter, deque
from copy import deepcopy
from types import SimpleNamespace
import threading

import h5py
import numpy as np
import pytest

pytest.importorskip("rclpy")

from cv_bridge import CvBridge
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray

import quest3_robot_data.recorder as recorder_module
from quest3_robot_data.recorder import BufferedImage, DatasetRecorder, EpisodeWriter, Sample, StartEpisode
from quest3_robot_data.episode_io import read_rgb


@pytest.fixture
def recorder(monkeypatch):
    node = DatasetRecorder.__new__(DatasetRecorder)
    node.config = DatasetRecorder._validate_config({
        "mode": "single", "record_topic": "/record", "task_description": "test",
        "cameras": {"front": {"rgb_topic": "/front", "record_size": [4, 4]},
                    "left_wrist": {"rgb_topic": "/wrist"}},
        "arms": {"left_arm": {"joint_topic": "/joint", "joint_order": ["joint"],
                              "commanded_action_topic": "/action", "gripper_position_topic": "/grip",
                              "gripper_command_topic": "/grip_command"}},
    })
    node.arm_order = ["left_arm"]
    node._stream_keys = [("front", "rgb"), ("left_wrist", "rgb")]
    node._buffers = {key: deque(maxlen=30) for key in (
        "image:front:rgb", "image:left_wrist:rgb", "joint:left_arm", "action:left_arm",
        "gripper:left_arm", "gripper_command:left_arm",
    )}
    node.recording = True
    node.synced_frames = node.enqueued_frames = node.dropped_frames = 0
    node.drop_reasons = Counter()
    node._last_reference_stamp = -1
    node._reference_wait_started = None
    samples = []
    node.writer = SimpleNamespace(failed=None, put_sample=lambda sample: samples.append(sample) or True)
    monkeypatch.setattr(DatasetRecorder, "get_logger", lambda self: SimpleNamespace(error=pytest.fail))
    now = [0]
    monkeypatch.setattr(recorder_module.time, "monotonic_ns", lambda: now[0])
    bridge = CvBridge()

    def add_frame(stamp):
        image = bridge.cv2_to_imgmsg(np.full((6, 8, 3), len(samples) + 1, np.uint8), encoding="rgb8")
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        for key in ("image:front:rgb", "image:left_wrist:rgb"):
            node._buffer_header_message(key, image)
        for key, value in (
            ("joint:left_arm", JointState(name=["joint"], position=[stamp / 1e9], velocity=[0.])),
            ("action:left_arm", Float64MultiArray(data=[0.])),
            ("gripper:left_arm", Float32(data=0.)),
            ("gripper_command:left_arm", Float32(data=0.)),
        ):
            node._buffers[key].append((stamp, value))

    return node, samples, add_frame, now


def test_burst_references_are_drained_in_order_without_image_conversion(recorder):
    node, samples, add, _ = recorder
    for index in range(7):
        add(1_000_000_000 + index * 33_333_333)
    node._aggregate_latest_reference()
    assert len(samples) == 4  # Bound callback work so subscribers can run.
    node._aggregate_latest_reference()
    node._aggregate_latest_reference()
    assert [sample.timestamp_ns for sample in samples] == [1_000_000_000 + i * 33_333_333 for i in range(7)]
    assert all(isinstance(sample.images["front"]["rgb"], BufferedImage) for sample in samples)
    assert node.dropped_frames == 0


def test_missing_source_retries_oldest_frame_when_new_reference_arrives(recorder):
    node, samples, add, _ = recorder
    add(1_000_000_000)
    wrist = node._buffers["image:left_wrist:rgb"].popleft()
    node._aggregate_latest_reference()
    assert not samples and not node.dropped_frames
    add(1_033_333_333)
    node._buffers["image:left_wrist:rgb"].appendleft(wrist)
    node._aggregate_latest_reference()
    assert [sample.timestamp_ns for sample in samples] == [1_000_000_000, 1_033_333_333]


def test_sync_timeout_and_overwritten_references_are_counted(recorder):
    node, samples, add, now = recorder
    node._buffers["image:front:rgb"] = deque(maxlen=2)
    for stamp in (1_000_000_000, 1_033_333_333, 1_066_666_666):
        add(stamp)
    assert node.drop_reasons == {"reference_overflow": 1}
    node._buffers["image:left_wrist:rgb"].clear()
    node._aggregate_latest_reference()
    now[0] = 51_000_000
    node._aggregate_latest_reference()
    assert node.drop_reasons == {"reference_overflow": 1, "sync_timeout": 1}
    assert not samples
    # Explicit stop resolves the remaining reference without waiting again.
    node._aggregate_latest_reference(finalize=True)
    assert node.dropped_frames == 3


def test_writer_backpressure_is_counted_without_retrying_or_reordering(recorder):
    node, samples, add, _ = recorder
    add(1_000_000_000)
    node.writer.put_sample = lambda sample: False
    node._aggregate_latest_reference()
    node._aggregate_latest_reference()
    assert node.drop_reasons == {"writer_queue_full": 1}
    assert node.synced_frames == 1 and node.enqueued_frames == 0


def sample(index):
    rgb = np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3)
    message = CvBridge().cv2_to_imgmsg(rgb, encoding="rgb8")
    return Sample(
        timestamp_ns=index, source_timestamps_ns={"front_rgb": index},
        images={"front": {"rgb": BufferedImage(message, "rgb", (4, 4)),
                           "depth": np.full((4, 4), 1234, np.uint16)}},
        qpos=np.full(6, index, np.float32), qvel=np.zeros(6, np.float32),
        commanded_action=np.ones(6, np.float32), gripper_position=np.zeros(1, np.float32),
        gripper_command=np.zeros(1, np.float32),
    )


@pytest.mark.parametrize("storage", ["raw", "raw_lzf", "jpeg"])
def test_writer_storage_and_actions_remain_aligned_across_episodes(tmp_path, storage):
    writer = EpisodeWriter(8, storage)
    try:
        for episode in range(2):
            assert writer.put_control(StartEpisode(tmp_path / f"{episode}.hdf5", {}))
            for index in (1, 2, 3):
                assert writer.put_sample(sample(index))
            assert writer.put_control(writer.STOP)
    finally:
        writer.put_control(writer.SHUTDOWN)
        writer.join(5)
    assert not writer.is_alive() and writer.failed is None
    for episode in range(2):
        with h5py.File(tmp_path / f"{episode}.hdf5") as file:
            rgb = file["observations/images/front/rgb"]
            assert rgb.attrs["storage"] == storage
            assert rgb.compression == ("lzf" if storage == "raw_lzf" else None)
            assert read_rgb(rgb, 0).shape == (4, 4, 3)
            if storage != "jpeg":
                expected = recorder_module.prepare_image(
                    np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3), "rgb", (4, 4)
                )
                np.testing.assert_array_equal(read_rgb(rgb, 0), expected)
            np.testing.assert_array_equal(file["observations/images/front/depth"][:], 1234)
            np.testing.assert_array_equal(file["executed_action"][:], [[1.] * 6, [1.] * 6, [0.] * 6])
            np.testing.assert_array_equal(file["action_valid"][:], [True, True, False])
            np.testing.assert_array_equal(file["is_terminal"][:], [False, False, True])
            EpisodeWriter._assert_sample_alignment(file, 3)


def test_stop_does_not_block_when_sample_queue_is_full(tmp_path):
    entered, release, controls_done = threading.Event(), threading.Event(), threading.Event()

    class SlowWriter(EpisodeWriter):
        def _prepare_sample(self, value):
            entered.set()
            assert release.wait(5)
            super()._prepare_sample(value)

    writer = SlowWriter(1)
    writer.put_control(StartEpisode(tmp_path / "episode.hdf5", {}))
    assert writer.put_sample(sample(1))
    try:
        assert entered.wait(2)
        assert writer.put_sample(sample(2))
        assert not writer.put_sample(sample(3))

        def stop():
            writer.put_control(writer.STOP)
            writer.put_control(writer.SHUTDOWN)
            controls_done.set()

        control = threading.Thread(target=stop, daemon=True)
        control.start()
        assert controls_done.wait(1), "STOP must not wait for disk I/O in a ROS callback"
    finally:
        release.set()
        if not controls_done.is_set():
            writer.put_control(writer.SHUTDOWN)
        writer.join(5)
    assert not writer.is_alive() and writer.failed is None
    with h5py.File(tmp_path / "episode.hdf5") as file:
        assert file.attrs["num_samples"] == 2


@pytest.mark.parametrize("field,value", [("writer_queue_size", 0), ("source_buffer_size", -1),
                                         ("aggregate_batch_size", 1.5), ("max_skew_sec", float("nan"))])
def test_invalid_buffer_settings_are_rejected(recorder, field, value):
    config = deepcopy(recorder[0].config)
    config[field] = value
    with pytest.raises(ValueError, match=field):
        DatasetRecorder._validate_config(config)
