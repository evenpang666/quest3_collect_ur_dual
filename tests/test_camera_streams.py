"""Regression coverage for USB 2 RGB profiles and deferred touch-point selection."""
from types import SimpleNamespace
import sys
import warnings

import cv2
import numpy as np
import pytest
import yaml

from quest3_robot_data import calibrate_camera_charuco as charuco


class FakeStream:
    def __init__(self, width, height, fps):
        self.values = width, height, fps

    def stream_type(self):
        return "color"

    def format(self):
        return "bgr8"

    def as_video_stream_profile(self):
        return self

    def width(self):
        return self.values[0]

    def height(self):
        return self.values[1]

    def fps(self):
        return self.values[2]

    def get_intrinsics(self):
        return SimpleNamespace(width=self.width(), height=self.height(), fx=600, fy=610,
                               ppx=self.width() / 2, ppy=self.height() / 2, coeffs=[0.0] * 5)


def fake_realsense(monkeypatch, available, failures=None):
    attempts, stopped = [], []
    failures = failures or {}
    streams = [FakeStream(*value) for value in available]
    device = SimpleNamespace(get_info=lambda field: "side_serial",
                             query_sensors=lambda: [SimpleNamespace(get_stream_profiles=lambda: streams)])
    context = SimpleNamespace(query_devices=lambda: [device])

    class Config:
        def enable_device(self, serial):
            assert serial == "side_serial"

        def enable_stream(self, stream, width, height, fmt, fps):
            assert stream == "color" and fmt == "bgr8"
            self.request = (width, height, fps)

    class Pipeline:
        def __init__(self, context):
            self.request = None

        def start(self, cfg):
            self.request = cfg.request
            attempts.append(self.request)
            if self.request in failures:
                raise RuntimeError(failures[self.request])
            assert self.request in available
            stream = FakeStream(*self.request)
            return SimpleNamespace(get_stream=lambda kind: stream)

        def stop(self):
            stopped.append(self.request)

        def wait_for_frames(self):
            width, height, _ = self.request
            image = np.zeros((height, width, 3), dtype=np.uint8)
            return SimpleNamespace(get_color_frame=lambda: SimpleNamespace(get_data=lambda: image))

    module = SimpleNamespace(context=lambda: context, pipeline=Pipeline, config=Config,
                             camera_info=SimpleNamespace(serial_number="serial_number"),
                             stream=SimpleNamespace(color="color"), format=SimpleNamespace(bgr8="bgr8"))
    monkeypatch.setitem(sys.modules, "pyrealsense2", module)
    return attempts, stopped


def test_empty_deferred_touch_list_has_no_runtime_warning():
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        assert charuco.describe_touch_points([], {}, 10) == []


@pytest.mark.parametrize("available,expected", [
    ([(1280, 720, 30), (1280, 720, 15), (640, 480, 30)], (1280, 720, 30)),
    ([(1280, 720, 15), (640, 480, 30)], (1280, 720, 15)),
    ([(640, 480, 30)], (640, 480, 30)),
])
def test_starts_only_advertised_profiles(monkeypatch, available, expected):
    attempts, stopped = fake_realsense(monkeypatch, available)
    pipeline, profile = charuco.start_color_pipeline("side_serial")
    assert profile.get_stream("color").values == expected
    assert attempts == [expected]
    pipeline.stop()
    assert stopped == [expected]


def test_failed_start_releases_resources_and_tries_lower_profile(monkeypatch):
    high, low = (1280, 720, 30), (640, 480, 30)
    attempts, stopped = fake_realsense(monkeypatch, [high, low], {high: "Couldn't resolve requests"})
    pipeline, profile = charuco.start_color_pipeline("side_serial")
    assert attempts == [high, low]
    assert stopped == [high]
    assert profile.get_stream("color").values == low
    pipeline.stop()


def test_busy_device_reports_actionable_error_without_retrying(monkeypatch):
    high, low = (1280, 720, 30), (640, 480, 30)
    attempts, stopped = fake_realsense(monkeypatch, [high, low], {high: "Device or resource busy"})
    with pytest.raises(SystemExit, match="stop camera-owning processes"):
        charuco.start_color_pipeline("side_serial")
    assert attempts == stopped == [high]


def test_missing_device_and_missing_color_profiles(monkeypatch):
    attempts, _ = fake_realsense(monkeypatch, [])
    with pytest.raises(SystemExit, match="not connected"):
        charuco.start_color_pipeline("missing_serial")
    with pytest.raises(SystemExit, match="no BGR8"):
        charuco.start_color_pipeline("side_serial")
    assert not attempts


def test_usb2_observation_and_final_check_use_actual_intrinsics(tmp_path, monkeypatch):
    attempts, stopped = fake_realsense(monkeypatch, [(1280, 720, 15), (640, 480, 30)])
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(10, 7, .037, .027, dictionary)
    ids = np.array([0, 1, 8, 9, 45, 53], dtype=np.int32).reshape(-1, 1)
    corners = np.array([[30 + i * 30, 100] for i in range(6)], dtype=np.float32).reshape(-1, 1, 2)
    monkeypatch.setattr(cv2.aruco, "detectMarkers", lambda *args, **kwargs: ([], ids, []))

    def interpolate(*args, cameraMatrix, distCoeffs):
        assert np.allclose(cameraMatrix, [[600, 0, 640], [0, 610, 360], [0, 0, 1]])
        assert len(distCoeffs) == 5
        return 6, corners, ids

    monkeypatch.setattr(cv2.aruco, "interpolateCornersCharuco", interpolate)
    monkeypatch.setattr(cv2.aruco, "drawDetectedCornersCharuco", lambda *args: None)
    monkeypatch.setattr(cv2.aruco, "estimatePoseCharucoBoard",
                        lambda *args: (True, np.zeros(3), np.array([0, 0, .5])))
    monkeypatch.setattr(cv2, "imshow", lambda *args: None)
    keys = iter([32, -1])
    monkeypatch.setattr(cv2, "waitKey", lambda *args: next(keys))
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda: None)
    args = SimpleNamespace(output=tmp_path / "side.yaml", camera_serial="side_serial", camera_frame="side",
                           camera_samples=1, recapture_camera=True, touch_corner_ids=None, squares_x=10)
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        observation = charuco.observe_camera(args, board, charuco._board_corners(board), dictionary,
                                             {"camera_serial": "side_serial"}, defer_touch_selection=True)
    assert observation["touch_corner_ids"] is None
    assert observation["color_fps"] == 15
    assert observation["intrinsics"]["width"] == 1280
    assert observation["intrinsics"]["height"] == 720
    checkpoint = yaml.safe_load(args.output.with_name("side_camera_observation.yaml").read_text())
    assert checkpoint["color_fps"] == 15
    motion = charuco.verify_board(args, board, dictionary, np.asarray(observation["camera_to_board"]))
    assert np.allclose(motion, [0, 0])
    assert attempts == stopped == [(1280, 720, 15), (1280, 720, 15)]
