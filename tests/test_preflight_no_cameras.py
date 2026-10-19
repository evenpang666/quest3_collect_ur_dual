"""The no-camera teleop path must leave an existing RealSense owner alone."""

from contextlib import nullcontext
from types import SimpleNamespace
import sys

import pytest
import yaml

from quest3_robot_data import preflight


@pytest.mark.parametrize(("adb_output", "expected"), [
    ("List of devices attached\n\n", "not detected by ADB"),
    ("List of devices attached\nquest\tunauthorized\n", "USB debugging is unauthorized"),
    ("List of devices attached\nquest\toffline\n", "offline in ADB"),
    ("List of devices attached\nquest\tno permissions (udev rules)\n", "USB access is denied"),
    ("List of devices attached\nquest\tdevice usb:1-1\n", None),
])
def test_quest_adb_failure_reports_device_state(adb_output, expected):
    failure = preflight.quest_adb_failure(adb_output)
    if expected is None:
        assert failure is None
    else:
        assert expected in failure


def test_preflight_skips_camera_query_but_checks_robot_and_quest(tmp_path, monkeypatch, capsys):
    for name in ("left_front", "left_side", "right_front", "right_side"):
        (tmp_path / f"{name}.yaml").write_text("calibrated: true\n")
    config = {
        "mode": "dual",
        "rtde": {"enable_motion": True, "gripper_port": 63352},
        "cameras": {"front": {"serial": "123456"}, "side": {"serial": "654321"}},
        "arms": {
            name: {
                "role": role, "robot_ip": f"192.0.2.{index}",
                "tcp_source": "robot_active_tcp",
                "front_calibration": f"{role}_front.yaml",
                "side_calibration": f"{role}_side.yaml",
            }
            for index, (name, role) in enumerate((("left_arm", "left"), ("right_arm", "right")), 1)
        },
    }
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(config))
    calls = []
    monkeypatch.setattr(preflight, "connected_realsense_serials", lambda: (_ for _ in ()).throw(
        AssertionError("camera query must not run")))
    monkeypatch.setattr(preflight.socket, "create_connection", lambda address, timeout: (
        calls.append(address), nullcontext())[1])
    monkeypatch.setattr(preflight.subprocess, "run", lambda command, **kwargs: SimpleNamespace(
        stdout="quest\tdevice\n" if command[-1] == "devices" else ""))
    monkeypatch.setattr(sys, "argv", ["preflight", "--config", str(path), "--skip-camera-check"])

    preflight.main()

    assert len(calls) == 4
    assert "camera enumeration skipped" in capsys.readouterr().out
