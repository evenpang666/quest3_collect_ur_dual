"""Joint command rate bounds and motion gates, without connecting to a robot."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from quest3_robot_data.target_limiter import limit_joint_target


def test_joint_limit_preserves_direction_and_stops_at_goal():
    reference = np.array([0.0, -2.0, 1.0, 3.0, -4.0, 5.0])
    goal = reference + [1.0, -0.5, 0.25, -1.0, 0.0, 0.1]
    original = reference.copy()
    for _ in range(125):
        command = limit_joint_target(reference, goal, 1.0, 1 / 125)
        assert np.max(np.abs(command - reference)) <= 0.008 + 1e-12
        reference = command
    np.testing.assert_allclose(reference, goal)
    np.testing.assert_allclose(limit_joint_target(goal, goal, 1.0, 1 / 125), goal)
    np.testing.assert_allclose(
        limit_joint_target(original, goal, 1.0, 1 / 125) - original,
        (goal - original) * 0.008,
    )


@pytest.mark.parametrize(("speed", "period"), [(0, .008), (-1, .008),
    (float("nan"), .008), (float("inf"), .008), (1, 0), (1, float("nan"))])
def test_invalid_rate_parameters_are_rejected(speed, period):
    with pytest.raises(ValueError):
        limit_joint_target(np.zeros(6), np.ones(6), speed, period)


@pytest.mark.parametrize("target", [[], [0] * 5, [0] * 7, [float("nan")] * 6])
def test_invalid_ik_solution_is_rejected(target):
    with pytest.raises(ValueError):
        limit_joint_target(np.zeros(6), target, 1.0, .008)


@pytest.fixture
def servo_harness(monkeypatch):
    pytest.importorskip("rclpy")
    from quest3_robot_data import rtde_servo

    def build(arm="left_arm", cycles=3, hz=125.0):
        path = Path(__file__).resolve().parents[1] / "config/hardware_dual_teleop.yaml"
        config = yaml.safe_load(path.read_text())
        common = dict(config["rtde"], control_hz=hz)
        monkeypatch.setattr(rtde_servo.ArmWorker, "start", lambda self: None)
        monkeypatch.setattr(rtde_servo, "GripperWorker", lambda *args: SimpleNamespace(stop=lambda: None))
        faults = []
        def fault(*args):
            faults.append(args)
            worker.stopping.set()
        worker = rtde_servo.ArmWorker(
            SimpleNamespace(create_publisher=lambda *args: None), arm,
            config["arms"][arm], common, path, safety_event_callback=fault,
        )
        worker.enabled = True
        worker.target = np.eye(4)
        worker.target[0, 3] = 0.2
        worker.target_time = time.monotonic()
        worker._publish = lambda receive: None
        state = SimpleNamespace(
            tick=0, actual=np.array([0., -2., 1., 3., -4., 5.]),
            solution=np.array([1., -3., 1.5, 2., -4., 5.2]),
            commands=[], references=[], faults=faults, stops=[],
            on_tick=lambda: None, on_ik=lambda: None,
        )
        def ik(pose, near):
            state.references.append(np.asarray(near))
            state.on_ik()
            return state.solution.tolist()
        def send(*args):
            state.commands.append((state.tick, args))
            return True
        def wait(start):
            state.tick += 1
            state.on_tick()
            if state.tick >= cycles:
                worker.stopping.set()
        def unexpected_servo_l(*args):
            raise AssertionError("rate-limited teleop must use servoJ")
        control = SimpleNamespace(
            initPeriod=lambda: 0, getInverseKinematics=ik, servoJ=send,
            servoL=unexpected_servo_l, waitPeriod=wait,
            servoStop=lambda: state.stops.append(state.tick),
            stopScript=lambda: None, disconnect=lambda: None,
        )
        receive = SimpleNamespace(
            isEmergencyStopped=lambda: False, isProtectiveStopped=lambda: False,
            getActualQ=lambda: state.actual.tolist(),
            getActualTCPPose=lambda: [0.0] * 6, disconnect=lambda: None,
        )
        monkeypatch.setitem(sys.modules, "rtde_control", SimpleNamespace(RTDEControlInterface=lambda ip: control))
        monkeypatch.setitem(sys.modules, "rtde_receive", SimpleNamespace(RTDEReceiveInterface=lambda ip: receive))
        return worker, state
    return build


@pytest.mark.parametrize("arm", ["left_arm", "right_arm"])
@pytest.mark.parametrize("hz", [125.0, 250.0])
def test_ordinary_loop_limits_each_joint_from_previous_command(servo_harness, arm, hz):
    worker, state = servo_harness(arm=arm, hz=hz)
    previous = state.actual.copy()
    # Feedback moving ahead must not increase the command-rate allowance.
    state.on_tick = lambda: setattr(state, "actual", state.actual + 0.1)
    worker.run()
    assert not state.faults
    assert len(state.commands) == 3
    for (_, args), near in zip(state.commands, state.references):
        np.testing.assert_allclose(near, previous)
        q = np.array(args[0])
        assert np.max(np.abs(q - previous)) <= 1.0 / hz + 1e-12
        assert args[3] == 1.0 / hz
        previous = q


def test_ordinary_loop_keeps_small_targets_exact(servo_harness):
    worker, state = servo_harness(cycles=1)
    state.solution = state.actual + 0.002
    worker.run()
    assert not state.faults
    np.testing.assert_allclose(state.commands[0][1][0], state.solution)


@pytest.mark.parametrize("gate", ["disabled", "stale", "workspace"])
def test_ordinary_loop_preserves_motion_gates(servo_harness, gate):
    worker, state = servo_harness(cycles=1)
    if gate == "disabled":
        worker.set_enabled(False)
    elif gate == "stale":
        worker.target_time -= 1.0
    else:
        worker.workspace_min, worker.workspace_max = np.zeros(3), np.full(3, 0.1)
    worker.run()
    assert not state.commands
    assert not state.references
    assert not state.faults


@pytest.mark.parametrize("gate", ["release", "regrip", "timeout", "avoidance"])
def test_pending_ik_cannot_send_after_motion_gate_changes(servo_harness, monkeypatch, gate):
    worker, state = servo_harness(cycles=1)
    def cancel():
        if gate in ("release", "regrip"):
            worker.set_enabled(False)
            if gate == "regrip":
                worker.set_enabled(True)
        else:
            worker.avoidance_available = True
            worker.set_avoidance_mode(True)
    if gate == "timeout":
        # A controller call may take longer than the target timeout.
        original = time.monotonic
        state.on_ik = lambda: monkeypatch.setattr(time, "monotonic", lambda: original() + 1.0)
    else:
        state.on_ik = cancel
    worker.run()
    assert not state.commands
    assert not state.faults


def test_regrip_restarts_rate_reference_at_measured_joints(servo_harness):
    worker, state = servo_harness(cycles=3)
    goal = worker.target.copy()
    def tick():
        if state.tick == 1:
            worker.set_enabled(False)
            state.actual = np.full(6, -1.0)
        elif state.tick == 2:
            worker.target = goal
            worker.target_time = time.monotonic()
            worker.set_enabled(True)
    state.on_tick = tick
    worker.run()
    assert not state.faults
    assert [tick for tick, _ in state.commands] == [0, 2]
    np.testing.assert_allclose(state.references[1], np.full(6, -1.0))
    assert np.max(np.abs(np.asarray(state.commands[1][1][0]) + 1.0)) <= .008 + 1e-12


@pytest.mark.parametrize("solution", [[], [float("nan")] * 6])
def test_invalid_ik_stops_without_sending(servo_harness, solution):
    worker, state = servo_harness(cycles=1)
    state.solution = np.asarray(solution)
    worker.run()
    assert not state.commands
    assert state.faults
    assert worker._rearm_required


def test_avoidance_commands_keep_their_separate_step_bound(servo_harness):
    worker, state = servo_harness(cycles=1)
    worker.avoidance_available = True
    worker.set_avoidance_mode(True)
    worker.enabled = True
    worker.joint_target = state.actual + 0.02
    worker.target_time = time.monotonic()
    worker.run()
    assert not state.faults
    assert not state.references
    np.testing.assert_allclose(state.commands[0][1][0], state.actual + .02)
