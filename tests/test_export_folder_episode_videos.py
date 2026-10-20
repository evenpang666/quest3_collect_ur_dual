import cv2
import numpy as np

from scripts.export_folder_episode_videos import DEFAULT_CODEC, export


def test_folder_export_uses_h264_for_rgb_and_depth(tmp_path):
    episode = tmp_path / "episode_00000"
    (episode / "rgb_front_cam").mkdir(parents=True)
    (episode / "depth_front_cam").mkdir()
    np.save(episode / "rgb_front_cam/000000.npy", np.full((16, 16, 3), 120, np.uint8))
    np.save(episode / "depth_front_cam/000000.npy", np.full((16, 16), 500, np.uint16))
    paths = export(episode, tmp_path / "videos", 30.0, DEFAULT_CODEC, False, 200, 2000)

    assert len(paths) == 8
    for path in paths:
        capture = cv2.VideoCapture(str(path))
        try:
            assert int(capture.get(cv2.CAP_PROP_FOURCC)) == cv2.VideoWriter_fourcc(*"avc1")
            assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 1
            assert capture.read()[0]
        finally:
            capture.release()
