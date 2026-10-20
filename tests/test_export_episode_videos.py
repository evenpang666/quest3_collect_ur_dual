import cv2
import h5py
import numpy as np

from scripts.export_episode_videos import DEFAULT_CODEC, export, output_fps


def test_export_defaults_to_30_fps_per_synchronized_sample():
    assert output_fps(None) == 30.0
    assert output_fps(24.0) == 24.0


def test_export_h264_retains_frames_colors_and_black_missing_cameras(tmp_path):
    episode = tmp_path / "episode_00000.hdf5"
    with h5py.File(episode, "w") as file:
        file.create_dataset("timestamps_ns", data=[0, 33_333_333, 66_666_666])
        frames = np.zeros((3, 16, 16, 3), dtype=np.uint8)
        frames[..., 0] = 220  # RGB red must remain red after OpenCV BGR conversion.
        file.create_dataset("observations/images/front/rgb", data=frames)

    paths = export(episode, tmp_path / "videos", None, DEFAULT_CODEC, False)

    assert len(paths) == 4
    for path in paths:
        capture = cv2.VideoCapture(str(path))
        try:
            assert int(capture.get(cv2.CAP_PROP_FOURCC)) == cv2.VideoWriter_fourcc(*"avc1")
            assert capture.get(cv2.CAP_PROP_FPS) == 30.0
            decoded = []
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                decoded.append(frame)
            assert len(decoded) == 3
            assert all(frame.shape == (16, 16, 3) for frame in decoded)
            if path.name == "front_cam.mp4":
                assert all(frame[..., 2].mean() > 200 and frame[..., 0].mean() < 15 for frame in decoded)
            else:
                assert all(frame.max() < 5 for frame in decoded)
        finally:
            capture.release()
