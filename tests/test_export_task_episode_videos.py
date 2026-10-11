import h5py
import numpy as np
import pytest

from scripts.export_task_episode_videos import export_task


def test_export_task_continues_after_bad_episode_and_skips_completed_one(tmp_path):
    task_dir = tmp_path / "pick_up_cube"
    task_dir.mkdir()
    good = task_dir / "episode_00000.hdf5"
    with h5py.File(good, "w") as file:
        file.create_dataset("timestamps_ns", data=[0, 33_333_333])
        file.create_dataset(
            "observations/images/front/rgb",
            data=np.full((2, 16, 16, 3), 120, dtype=np.uint8),
        )
    with h5py.File(task_dir / "episode_00001.hdf5", "w"):
        pass

    assert export_task(task_dir) == (1, 0, 1)
    videos = task_dir / "episode_00000_videos"
    assert sorted(path.name for path in videos.glob("*.mp4")) == [
        "front_cam.mp4", "side_cam.mp4", "wrist_left_cam.mp4", "wrist_right_cam.mp4"
    ]
    assert all(path.stat().st_size > 0 for path in videos.glob("*.mp4"))
    assert export_task(task_dir) == (0, 1, 1)
    (videos / "side_cam.mp4").write_bytes(b"incomplete")
    assert export_task(task_dir) == (1, 0, 1)
    assert (videos / "side_cam.mp4").stat().st_size > len(b"incomplete")


def test_export_task_requires_hdf5_files(tmp_path):
    with pytest.raises(FileNotFoundError, match="no .hdf5 files"):
        export_task(tmp_path)
