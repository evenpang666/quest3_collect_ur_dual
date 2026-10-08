"""Exercise dual calibration with synthetic camera observations and a read-only TCP."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest
import yaml

from quest3_robot_data import calibrate_camera_charuco as charuco
from quest3_robot_data import calibrate_dual_camera_charuco as dual
from quest3_robot_data import preflight
from quest3_robot_data.transforms import load_transform, rotvec_to_matrix


BOARD_ARGS = [
    "--squares-x", "10", "--squares-y", "7",
    "--square-size-m", "0.037", "--marker-size-m", "0.027",
    "--dictionary", "DICT_5X5_100",
]
ROOT = Path(__file__).resolve().parents[1]


def make_config(tmp_path):
    config = yaml.safe_load((ROOT / "config/hardware_dual_teleop.yaml").read_text())
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(config))
    return path, config


def pose(translation, rotvec):
    value = np.eye(4)
    value[:3, :3] = rotvec_to_matrix(rotvec)
    value[:3, 3] = translation
    return value


class SyntheticRig:
    def __init__(self, monkeypatch):
        self.events = []
        self.receiver_count = 0
        self.side_motion_mm = 0.0
        self.side_spread_mm = 0.0
        self.side_ids = [0, 8, 9, 45, 53]
        self.touch_ids = []
        self.current_base = None
        self.boards = {
            "left_base": pose([.3, .1, .2], [.1, -.2, .4]),
            # The board has moved between arm sessions.
            "right_base": pose([-.2, -.3, .4], [-.3, .2, -.6]),
        }
        self.left_to_right = pose([-.6, .4, .2], [-.1, .2, .3])
        left_cameras = {
            "260922305390": pose([.7, .2, .8], [.2, -.1, .7]),
            "260922303231": pose([1.1, -.1, .9], [-.4, .5, -.3]),
        }
        self.cameras = {}
        for serial, value in left_cameras.items():
            self.cameras["left_base", serial] = value
            self.cameras["right_base", serial] = np.linalg.inv(self.left_to_right) @ value
        monkeypatch.setattr(charuco, "observe_camera", self.observe)
        monkeypatch.setattr(charuco, "verify_board", self.verify)
        real_touch = charuco.touch_board

        def touch(args, board_points, touch_ids, candidates, signature):
            self.touch_ids = list(touch_ids)
            self.current_base = args.base_frame
            self.board_points = board_points
            return real_touch(args, board_points, touch_ids, candidates, signature)

        monkeypatch.setattr(charuco, "touch_board", touch)
        monkeypatch.setitem(sys.modules, "rtde_receive", SimpleNamespace(RTDEReceiveInterface=self.receiver))
        monkeypatch.setattr("builtins.input", self.answer)

    def answer(self, prompt):
        self.events.append(("prompt", prompt))
        return ""

    def observe(self, args, board, points, dictionary, signature, **kwargs):
        self.events.append(("observe", args.base_frame, args.camera_serial, args.recapture_camera))
        if args.base_frame == "right_base":
            ids = ([2, 6, 12, 47, 51] if args.camera_serial == "260922303231" else [2, 6, 11, 47, 51])
        else:
            ids = self.side_ids if args.camera_serial == "260922303231" else [0, 1, 8, 45, 53]
        return {
            "signature": signature,
            "observation_id": f"{args.base_frame}-{args.camera_serial}",
            "camera_to_board": (np.linalg.inv(self.cameras[args.base_frame, args.camera_serial])
                                @ self.boards[args.base_frame]).tolist(),
            "detected_corner_ids": ids,
            "detected_corner_pixels": {key: [float(key % 9 * 30), float(key // 9 * 30)] for key in ids},
            "touch_corner_ids": [0, 8, 45, 53],
            "camera_translation_max_spread_mm": (
                self.side_spread_mm if args.camera_serial == "260922303231" else 0.1),
            "camera_rotation_max_spread_deg": 0.1,
            "camera_samples": 30,
            "intrinsics": {"fx": 900},
        }

    def verify(self, args, board, dictionary, camera_to_board):
        self.events.append(("verify", args.base_frame, args.camera_serial))
        assert np.allclose(camera_to_board, np.linalg.inv(self.cameras[args.base_frame, args.camera_serial])
                           @ self.boards[args.base_frame])
        return (self.side_motion_mm if args.camera_serial == "260922303231" else 0.1), 0.1

    def receiver(self, robot_ip):
        self.receiver_count += 1
        self.events.append(("connect", robot_ip))
        points = iter(self.touch_ids)

        def tcp():
            corner_id = next(points)
            board = self.boards[self.current_base]
            point = board[:3, :3] @ self.board_points[corner_id] + board[:3, 3]
            self.events.append(("tcp", self.current_base, corner_id))
            return [*point, 0, 0, 0]

        return SimpleNamespace(getActualTCPPose=tcp, disconnect=lambda: self.events.append(("disconnect", robot_ip)))


def test_both_arms_recover_four_extrinsics_with_board_moved(tmp_path, monkeypatch):
    config_path, config = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    dual.main(["--config", str(config_path), *BOARD_ARGS])
    for arm in config["arms"].values():
        for camera, serial in (("front", "260922305390"), ("side", "260922303231")):
            result = yaml.safe_load((tmp_path / arm[f"{camera}_calibration"]).read_text())
            assert result["calibrated"] is True
            assert result["camera_serial"] == serial
            assert result["touch_corner_ids"] == ([0, 8, 45, 53] if arm["role"] == "left" else [2, 6, 47, 51])
            assert np.allclose(load_transform(result), rig.cameras[arm["base_frame"], serial])
    observations = [event for event in rig.events if event[0] == "observe"]
    assert observations == [
        ("observe", "left_base", "260922305390", True),
        ("observe", "left_base", "260922303231", True),
        ("observe", "right_base", "260922305390", True),
        ("observe", "right_base", "260922303231", True),
    ]
    assert rig.receiver_count == 2
    assert len([event for event in rig.events if event[0] == "tcp"]) == 8
    left_verification = rig.events.index(("verify", "left_base", "260922303231"))
    right_observation = rig.events.index(observations[2])
    assert left_verification < right_observation
    prompts = [event[1] for event in rig.events[left_verification:right_observation] if event[0] == "prompt"]
    assert any("右臂" in prompt and "可移动标定板" in prompt for prompt in prompts)
    signatures = []
    for arm in config["arms"].values():
        path = tmp_path / arm["front_calibration"]
        signatures.append(yaml.safe_load(path.with_name(path.stem + "_touches.yaml").read_text())["signature"])
    assert signatures[0] != signatures[1]
    summary = yaml.safe_load((tmp_path / config["dual_calibration"]).read_text())
    assert summary["calibrated"] is True
    assert np.allclose(load_transform(summary["left_base_to_right_base"]), rig.left_to_right)
    expected_cameras = np.linalg.inv(rig.cameras["left_base", "260922305390"]) @ rig.cameras["left_base", "260922303231"]
    assert np.allclose(load_transform(summary["front_camera_to_side_camera"]), expected_cameras)


@pytest.mark.parametrize("failure", ["motion", "spread"])
def test_bad_side_camera_invalidates_pair_and_stops_before_right(tmp_path, monkeypatch, failure):
    path, config = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    if failure == "motion":
        rig.side_motion_mm = 50
    else:
        rig.side_spread_mm = 50
    with pytest.raises(SystemExit, match="quality gate failed"):
        dual.main(["--config", str(path), *BOARD_ARGS])
    for camera in ("front", "side"):
        result = yaml.safe_load((tmp_path / config["arms"]["left_arm"][f"{camera}_calibration"]).read_text())
        assert result["calibrated"] is False
        assert not (tmp_path / config["arms"]["right_arm"][f"{camera}_calibration"]).exists()
    assert not any(event[0] == "observe" and event[1] == "right_base" for event in rig.events)


def test_insufficient_shared_corners_never_connects_robot(tmp_path, monkeypatch):
    path, _ = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    rig.side_ids = [0, 1, 8, 9, 10, 11]
    with pytest.raises(SystemExit, match="四个不共线"):
        dual.main(["--config", str(path), *BOARD_ARGS])
    assert rig.receiver_count == 0


def test_single_camera_cli_remains_usable(tmp_path, monkeypatch):
    rig = SyntheticRig(monkeypatch)
    output = tmp_path / "left_front.yaml"
    charuco.main([
        "--robot-ip", "192.168.124.61", "--base-frame", "left_base",
        "--camera-serial", "260922305390", "--camera-frame", "front_camera_color_optical_frame",
        "--output", str(output), "--recapture-camera", *BOARD_ARGS,
    ])
    assert yaml.safe_load(output.read_text())["calibrated"] is True
    assert len([event for event in rig.events if event[0] == "tcp"]) == 4


def test_resume_requires_single_arm_and_does_not_offer_board_movement(tmp_path, monkeypatch):
    path, _ = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    with pytest.raises(SystemExit):
        dual.main(["--config", str(path), "--resume", *BOARD_ARGS])
    dual.main(["--config", str(path), "--arm", "right", "--resume", *BOARD_ARGS])
    observations = [event for event in rig.events if event[0] == "observe"]
    assert len(observations) == 2
    assert all(event[1] == "right_base" and not event[3] for event in observations)
    preparation = [event[1] for event in rig.events if event[0] == "prompt" and "准备右臂" in event[1]][0]
    assert "禁止移动" in preparation and "可移动标定板" not in preparation


def test_rerun_skips_successful_left_pair_and_recaptures_right(tmp_path, monkeypatch):
    path, config = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    dual.main(["--config", str(path), "--arm", "left", *BOARD_ARGS])
    saved_left = {camera: (tmp_path / config["arms"]["left_arm"][f"{camera}_calibration"]).read_bytes()
                  for camera in ("front", "side")}
    rig.events.clear()
    dual.main(["--config", str(path), *BOARD_ARGS])
    assert all(event[1] == "right_base" for event in rig.events if event[0] == "observe")
    for camera in ("front", "side"):
        assert (tmp_path / config["arms"]["left_arm"][f"{camera}_calibration"]).read_bytes() == saved_left[camera]


def test_new_observation_rejects_stale_touches_before_rtde_connect(tmp_path, monkeypatch):
    path, _ = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    rig.side_motion_mm = 50
    with pytest.raises(SystemExit):
        dual.main(["--config", str(path), "--arm", "left", *BOARD_ARGS])
    count = rig.receiver_count
    observe = rig.observe

    def recaptured(*args, **kwargs):
        observation = observe(*args, **kwargs)
        observation["observation_id"] += "-new"
        return observation

    monkeypatch.setattr(charuco, "observe_camera", recaptured)
    with pytest.raises(SystemExit, match="saved touch checkpoint does not match"):
        dual.main(["--config", str(path), "--arm", "left", "--resume", *BOARD_ARGS])
    assert rig.receiver_count == count


def test_fresh_session_ignores_old_touches(tmp_path, monkeypatch):
    path, _ = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    rig.side_motion_mm = 50
    with pytest.raises(SystemExit):
        dual.main(["--config", str(path), "--arm", "left", *BOARD_ARGS])
    rig.side_motion_mm = 0
    # New board placement after a failed session requires all four touches anew.
    rig.boards["left_base"][:3, 3] += [.1, .2, .3]
    dual.main(["--config", str(path), "--arm", "left", *BOARD_ARGS])
    assert len([event for event in rig.events if event[0] == "tcp"]) == 8


def test_preflight_requires_side_for_each_arm_only_when_configured(tmp_path, monkeypatch, capsys):
    path, config = make_config(tmp_path)
    # This test isolates per-camera preflight checks from the combined result.
    config.pop("dual_calibration")
    path.write_text(yaml.safe_dump(config))
    for arm in config["arms"].values():
        output = tmp_path / arm["front_calibration"]
        output.parent.mkdir(exist_ok=True)
        output.write_text("calibrated: true\n")
    monkeypatch.setattr(sys, "argv", ["preflight", "--config", str(path), "--skip-connectivity"])
    with pytest.raises(SystemExit):
        preflight.main()
    assert "missing side_calibration" in capsys.readouterr().out
    for arm in config["arms"].values():
        (tmp_path / arm["side_calibration"]).write_text("calibrated: true\n")
    preflight.main()
    assert "PREFLIGHT PASS" in capsys.readouterr().out
    # Front-only commissioning still passes without side calibration fields.
    config = deepcopy(config)
    config["cameras"].pop("side")
    for arm in config["arms"].values():
        arm.pop("side_calibration")
    path.write_text(yaml.safe_dump(config))
    preflight.main()
    assert "PREFLIGHT PASS" in capsys.readouterr().out


def test_duplicate_output_paths_are_rejected_before_observing(tmp_path, monkeypatch):
    path, config = make_config(tmp_path)
    config["arms"]["right_arm"]["side_calibration"] = config["arms"]["left_arm"]["side_calibration"]
    path.write_text(yaml.safe_dump(config))
    rig = SyntheticRig(monkeypatch)
    with pytest.raises(SystemExit, match="output paths must be different"):
        dual.main(["--config", str(path), *BOARD_ARGS])
    assert rig.events == []


def test_resume_camera_observation_without_selected_points(tmp_path, monkeypatch):
    # An interruption after front capture leaves no chosen common corners yet.
    args = SimpleNamespace(output=tmp_path / "front.yaml", recapture_camera=False, touch_corner_ids=None)
    observation = {"signature": {"camera_serial": "260922305390"},
                   "detected_corner_ids": [0, 8, 45, 53], "touch_corner_ids": None}
    args.output.with_name("front_camera_observation.yaml").write_text(yaml.safe_dump(observation))
    monkeypatch.setitem(sys.modules, "pyrealsense2", SimpleNamespace())
    assert charuco.observe_camera(args, None, None, None, observation["signature"],
                                  defer_touch_selection=True) == observation


def test_cross_arm_disagreement_retains_independent_results_but_blocks_summary(tmp_path, monkeypatch):
    path, config = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    # Simulate the side camera moving between left/right sessions.
    rig.cameras["right_base", "260922303231"][:3, 3] += [.1, 0, 0]
    with pytest.raises(SystemExit, match="cross-arm consistency failed"):
        dual.main(["--config", str(path), *BOARD_ARGS])
    summary = yaml.safe_load((tmp_path / config["dual_calibration"]).read_text())
    assert summary["calibrated"] is False
    for arm in config["arms"].values():
        for camera in ("front", "side"):
            assert yaml.safe_load((tmp_path / arm[f"{camera}_calibration"]).read_text())["calibrated"] is True
    monkeypatch.setattr(sys, "argv", ["preflight", "--config", str(path), "--skip-connectivity"])
    with pytest.raises(SystemExit):
        preflight.main()


def test_preflight_rejects_stale_combined_result(tmp_path, monkeypatch, capsys):
    path, config = make_config(tmp_path)
    SyntheticRig(monkeypatch)
    dual.main(["--config", str(path), *BOARD_ARGS])
    monkeypatch.setattr(sys, "argv", ["preflight", "--config", str(path), "--skip-connectivity"])
    preflight.main()
    assert "PREFLIGHT PASS" in capsys.readouterr().out
    source = tmp_path / config["arms"]["right_arm"]["side_calibration"]
    source.write_text(source.read_text() + "# Changed independently after the combined calculation.\n")
    with pytest.raises(SystemExit):
        preflight.main()
    assert "dual_calibration is stale" in capsys.readouterr().out


def test_recorder_embeds_all_independent_results_and_combined_transforms(tmp_path, monkeypatch):
    pytest.importorskip("rclpy")
    from quest3_robot_data.recorder import _embed_hardware_metadata

    path, config = make_config(tmp_path)
    rig = SyntheticRig(monkeypatch)
    dual.main(["--config", str(path), *BOARD_ARGS])
    metadata = {}
    _embed_hardware_metadata(metadata, path)
    embedded = metadata["calibration_yaml"]
    assert set(embedded) == {
        "left_arm/front_calibration", "left_arm/side_calibration",
        "right_arm/front_calibration", "right_arm/side_calibration", "dual_calibration",
    }
    summary = yaml.safe_load(embedded["dual_calibration"])
    assert np.allclose(load_transform(summary["left_base_to_right_base"]), rig.left_to_right)
    assert yaml.safe_load(metadata["hardware_config_yaml"]) == config
