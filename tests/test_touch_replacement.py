"""Touch replacements preserve known board geometry and resumable TCP readings."""
from types import SimpleNamespace
import sys

import cv2
import numpy as np
import pytest
import yaml

from quest3_robot_data import calibrate_camera_charuco as charuco


def setup_touch_session(tmp_path, monkeypatch, answers, sampled_ids):
    board = cv2.aruco.CharucoBoard_create(
        11, 8, .070, .052, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    )
    points = charuco._board_corners(board)
    args = SimpleNamespace(output=tmp_path / "right_front.yaml", robot_ip="robot",
                           squares_x=11, recapture_camera=False, retouch_corner_ids=[])
    reads, prompts, disconnected = [], [], []
    sample_ids = iter(sampled_ids)
    input_answers = iter(answers)

    def tcp():
        corner = next(sample_ids)
        reads.append(corner)
        return [*points[corner], 0, 0, 0]

    def answer(prompt):
        prompts.append(prompt)
        return next(input_answers)

    receiver = SimpleNamespace(getActualTCPPose=tcp, disconnect=lambda: disconnected.append(True))
    monkeypatch.setitem(sys.modules, "rtde_receive", SimpleNamespace(RTDEReceiveInterface=lambda ip: receiver))
    monkeypatch.setattr("builtins.input", answer)
    return args, points, reads, prompts, disconnected


def test_undetected_replacement_resumes_and_recovers_transform(tmp_path, monkeypatch, capsys):
    args, points, reads, prompts, disconnected = setup_touch_session(
        tmp_path, monkeypatch, ["id=46", "", ""], [46, 69]
    )
    signature = {"observation_ids": ["right-front", "right-side"]}
    checkpoint = args.output.with_name("right_front_touches.yaml")
    checkpoint.write_text(yaml.safe_dump({
        "signature": signature, "touch_corner_ids": [4, 9, 67, 69],
        "touches": {key: points[key].tolist() for key in [4, 9]},
    }))
    transform, rms, ids = charuco.touch_board(args, points, [3, 9, 67, 69], {3, 4, 9, 67, 69}, signature)
    assert ids == [4, 9, 46, 69]
    assert reads == [46, 69]
    assert rms < 1e-10 and np.allclose(transform, np.eye(4))
    assert any("第 5 行、第 7 列" in prompt for prompt in prompts)
    assert "未在两台锁定图中共同识别" in capsys.readouterr().out
    assert yaml.safe_load(checkpoint.read_text())["touch_corner_ids"] == ids
    # A later resume retains ID 46 even though neither image detected it.
    charuco.touch_board(args, points, [3, 9, 67, 69], {3, 4, 9, 67, 69}, signature)
    assert reads == [46, 69]
    assert len(disconnected) == 2


def test_invalid_replacements_reprompt_without_reading_tcp(tmp_path, monkeypatch, capsys):
    args, points, reads, prompts, disconnected = setup_touch_session(
        tmp_path, monkeypatch,
        ["id=bogus", "id=-1", "id=70", "id=9", "oops", "", "", "", ""], [4, 9, 67, 69]
    )
    _, rms, ids = charuco.touch_board(args, points, [4, 9, 67, 69], {4, 9, 67, 69}, {})
    assert ids == [4, 9, 67, 69]
    assert reads == ids and rms < 1e-10
    assert len(prompts) == 9 and disconnected == [True]
    output = capsys.readouterr().out
    assert "必须是整数" in output and "超出本板范围" in output and "已用于其他触点" in output


def test_collinear_replacement_is_rejected_before_sampling(tmp_path, monkeypatch, capsys):
    args, points, reads, _, _ = setup_touch_session(
        tmp_path, monkeypatch, ["", "", "", "id=3", ""], [0, 1, 2, 10]
    )
    _, rms, ids = charuco.touch_board(args, points, [0, 1, 2, 10], {0, 1, 2, 3, 10}, {})
    assert ids == [0, 1, 2, 10] and reads == ids and rms < 1e-10
    assert "四个触点共线" in capsys.readouterr().out


def test_cancelling_replacement_preserves_checkpoint_and_disconnects(tmp_path, monkeypatch):
    args, points, reads, _, disconnected = setup_touch_session(tmp_path, monkeypatch, ["id=46", "q"], [])
    checkpoint = args.output.with_name("right_front_touches.yaml")
    saved = yaml.safe_dump({"signature": {}, "touch_corner_ids": [4, 9, 67, 69],
                            "touches": {key: points[key].tolist() for key in [4, 9]}})
    checkpoint.write_text(saved)
    with pytest.raises(SystemExit, match="cancelled"):
        charuco.touch_board(args, points, [4, 9, 67, 69], {4, 9, 67, 69}, {})
    assert not reads and disconnected == [True]
    assert checkpoint.read_text() == saved
