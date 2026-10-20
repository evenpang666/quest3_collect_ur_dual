"""Exercise the real servo loop with fake RTDE interfaces; no hardware access."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

pytest.importorskip("rclpy")

from quest3_robot_data import rtde_servo
from quest3_robot_data.target_limiter import limit_cartesian_target
from quest3_robot_data.transforms import matrix_to_ur_pose, rotvec_to_matrix


@pytest.mark.parametrize(("legacy_limits", "enabled", "stale"), [
    (False, True, False),
    (True, True, False),
    (False, False, False),
    (False, True, True),
])
def test_servo_loop_streams_full_dual_target_and_preserves_motion_gates(
    monkeypatch, legacy_limits, enabled, stale,
):
    path = Path(__file__).resolve().parents[1] / "config/hardware_dual_teleop.yaml"
    config = yaml.safe_load(path.read_text())
    common = dict(config["rtde"], control_hz=config["control_hz"])
    if legacy_limits:
        # Configurations without the new switch retain their old limit.
        common.pop("limit_cartesian_targets")
    node = SimpleNamespace(create_publisher=lambda *args: None)
    monkeypatch.setattr(rtde_servo.ArmWorker, "start", lambda self: None)
    monkeypatch.setattr(rtde_servo, "GripperWorker", lambda *args: SimpleNamespace(stop=lambda: None))
    faults = []
    worker = rtde_servo.ArmWorker(
        node, "left_arm", config["arms"]["left_arm"], common, path,
        safety_event_callback=lambda *args: faults.append(args),
    )
    goal = np.eye(4)
    goal[:3, 3] = [0.20, -0.10, 0.05]
    goal[:3, :3] = rotvec_to_matrix([0.2, 0.3, 0.4])
    worker.target = goal
    worker.enabled = enabled
    worker.target_time = time.monotonic() - (1.0 if stale else 0.0)
    worker._publish = lambda receive: None
    commands = []
    control = SimpleNamespace(
        initPeriod=lambda: 0,
        servoL=lambda *args: commands.append(args) or True,
        waitPeriod=lambda start: worker.stopping.set(),
        servoStop=lambda: None, stopScript=lambda: None, disconnect=lambda: None,
    )
    receive = SimpleNamespace(
        isEmergencyStopped=lambda: False, isProtectiveStopped=lambda: False,
        getActualTCPPose=lambda: [0.0] * 6, disconnect=lambda: None,
    )
    monkeypatch.setitem(sys.modules, "rtde_control", SimpleNamespace(RTDEControlInterface=lambda ip: control))
    monkeypatch.setitem(sys.modules, "rtde_receive", SimpleNamespace(RTDEReceiveInterface=lambda ip: receive))

    worker.run()

    assert not faults
    if not enabled or stale:
        assert not commands
        return
    assert len(commands) == 1
    expected = (limit_cartesian_target(np.eye(4), goal, common["max_target_jump_m"],
                                       common["max_target_jump_rad"])
                if legacy_limits else goal)
    np.testing.assert_allclose(commands[0][0], matrix_to_ur_pose(expected))
    assert commands[0][3] == 1.0 / config["control_hz"]
