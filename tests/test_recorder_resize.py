import numpy as np
import pytest
import h5py

pytest.importorskip("rclpy")
from quest3_robot_data.recorder import DatasetRecorder, EpisodeWriter, FolderEpisodeWriter, Sample, StartEpisode


def test_front_preview_resolution_is_converted_to_existing_record_size():
    recorder = DatasetRecorder.__new__(DatasetRecorder)
    recorder.config = {"cameras": {"front": {"record_size": [640, 480]}}}

    rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
    depth = np.full((720, 1280), 1234, dtype=np.uint16)

    stored_rgb = recorder._prepare_image_for_recording("front", "rgb", rgb)
    stored_depth = recorder._prepare_image_for_recording("front", "depth", depth)

    assert stored_rgb.shape == (480, 640, 3)
    assert stored_rgb.dtype == np.uint8
    assert stored_depth.shape == (480, 640)
    assert stored_depth.dtype == np.uint16
    assert np.all(stored_depth == 1234)


def test_episode_alignment_rejects_any_dataset_with_a_different_sample_count(tmp_path):
    path = tmp_path / "episode.hdf5"
    with h5py.File(path, "w") as file:
        file.create_dataset("timestamps_ns", data=[1, 2])
        file.create_dataset("action", data=[[0.0], [0.0]])
        EpisodeWriter._assert_sample_alignment(file, 2)
        file.create_dataset("observations/images/front/rgb", data=[[[[0, 0, 0]]]])
        with pytest.raises(RuntimeError, match="misaligned"):
            EpisodeWriter._assert_sample_alignment(file, 2)


def test_folder_writer_keeps_image_and_robot_state_indices_aligned(tmp_path):
    def sample(index):
        return Sample(
            timestamp_ns=index,
            source_timestamps_ns={"front_rgb": index},
            images={"front": {"rgb": np.full((2, 3, 3), index, np.uint8), "depth": np.full((2, 3), index, np.uint16)}},
            qpos=np.full(6, index, np.float32), qvel=np.zeros(6, np.float32),
            commanded_action=np.ones(6, np.float32), gripper_position=np.array([0.0], np.float32),
            gripper_command=np.array([0.0], np.float32),
        )

    writer = FolderEpisodeWriter(8)
    episode = tmp_path / "episode_00000"
    assert writer.put_control(StartEpisode(episode, {"mode": "single"}))
    assert writer.put_sample(sample(1))
    assert writer.put_sample(sample(2))
    assert writer.put_control(FolderEpisodeWriter.STOP)
    assert writer.put_control(FolderEpisodeWriter.SHUTDOWN)
    writer.join(timeout=5)

    assert writer.failed is None
    assert (episode / "rgb_front_cam" / "000000.jpg").is_file()
    assert (episode / "rgb_front_cam" / "000001.jpg").is_file()
    assert (episode / "depth_front_cam" / "000000.npy").is_file()
    assert len((episode / "robot_state.jsonl").read_text().splitlines()) == 2
