"""A-key avoidance belongs only to the dedicated no-camera teleop mode."""

import json
from concurrent.futures import Future
from dataclasses import replace
from io import BytesIO
import time
from types import MethodType, SimpleNamespace
from urllib.error import HTTPError

import numpy as np
import pytest

pytest.importorskip("rclpy")
from std_msgs.msg import String

from quest3_robot_data.pose_mapper import PoseMapperNode
from quest3_robot_data.teleop_avoidance_client import SnapshotExpiredError, TeleopAvoidanceClient
from quest3_robot_data.teleop_core import ControllerGate, MappedTarget


def pose_message(*, toggle=False, squeeze=0.0, host_timestamp_ns=None):
    hand = {"position": [0, 0, 0], "orientation": [0, 0, 0, 1],
            "squeeze": squeeze, "trigger": 0.0}
    data = {"left": hand, "right": hand, "avoidance_toggle": toggle}
    if host_timestamp_ns is not None:
        data["host_timestamp_ns"] = host_timestamp_ns
    return String(data=json.dumps(data))


class Arm:
    def __init__(self, moving):
        self.moving = moving
        self.mapper = SimpleNamespace(deadman_threshold=0.55, gripper_close_threshold=0.15)
        self.joints = np.zeros(6)
        self.gripper_position = 0.0
        self.current_tcp = np.eye(4)
        self.joint_time = self.gripper_time = self.tcp_time = time.monotonic()
        self.published = []
        self.reset_count = 0
        self.enabled = False
        self.hold_count = 0
        self.pose_position = np.zeros(3)

    def reset(self):
        self.reset_count += 1

    def compute(self, hand, recenter):
        pose = np.eye(4)
        pose[:3, 3] = self.pose_position
        return MappedTarget(pose if self.moving else None, self.moving, 0.0)

    def publish(self, result):
        self.published.append(result)

    def publish_avoidance(self, result, target):
        self.published.append((result, target))

    def set_enabled(self, enabled):
        self.enabled = enabled

    def hold_avoidance(self):
        self.hold_count += 1
        self.enabled = False


class ImmediateExecutor:
    def submit(self, callback, *args):
        future = Future()
        try:
            future.set_result(callback(*args))
        except Exception as exc:
            future.set_exception(exc)
        return future


class DeferredExecutor:
    def __init__(self):
        self.calls = []
        self.future = Future()

    def submit(self, callback, *args):
        self.calls.append((callback, args))
        return self.future


def node(available=True, errors=None, warnings=None, infos=None):
    if errors is None:
        errors = []
    if warnings is None:
        warnings = []
    if infos is None:
        infos = []
    fake = SimpleNamespace(
        arms={"left": Arm(True), "right": Arm(False)},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        get_logger=lambda: SimpleNamespace(error=errors.append, warning=warnings.append, info=infos.append),
        controller_gate=ControllerGate(("left", "right"), 0.25),
        rearm_required=False, collision_stop=False,
        config={"rtde": {"max_target_jump_m": 0.0036, "max_target_jump_rad": 0.024}},
        avoidance_available=available, avoidance_enabled=False, avoidance=None,
        avoidance_error="", avoidance_failure_kind="", avoidance_failure_last_log=0.0,
        avoidance_event="", avoidance_event_seq=0,
        avoidance_stale_since=None,
        avoidance_generation=0, avoidance_sequence=0, avoidance_last_attempt_sequence=-1,
        avoidance_last_dispatch_at=0.0,
        avoidance_latest=None, avoidance_future=None, avoidance_inflight=None,
        avoidance_reset_pending=False, avoidance_executor=ImmediateExecutor(),
        avoidance_mode_pub=SimpleNamespace(publish=lambda message: None) if available else None,
    )
    fake._avoidance_failure = MethodType(PoseMapperNode._avoidance_failure, fake)
    fake._avoidance_feedback = MethodType(PoseMapperNode._avoidance_feedback, fake)
    fake._submit_avoidance = MethodType(PoseMapperNode._submit_avoidance, fake)
    return fake


def test_a_toggles_only_in_dedicated_mode(monkeypatch):
    class Client:
        def __init__(self, config):
            self.config = config

    monkeypatch.setattr("quest3_robot_data.teleop_avoidance_client.TeleopAvoidanceClient", Client)
    dedicated = node()
    PoseMapperNode._controllers(dedicated, pose_message(toggle=True))
    assert dedicated.avoidance_enabled
    assert dedicated.avoidance_event == "enabled"
    PoseMapperNode._controllers(dedicated, pose_message(toggle=True))
    assert not dedicated.avoidance_enabled
    assert dedicated.avoidance_event == "disabled"

    ordinary = node(available=False)
    PoseMapperNode._controllers(ordinary, pose_message(toggle=True))
    assert not ordinary.avoidance_enabled
    assert ordinary.avoidance_event_seq == 0


def test_client_uses_port_selected_by_dedicated_script(monkeypatch):
    monkeypatch.setenv("QUEST3_AVOIDANCE_RUNTIME_URL", "http://127.0.0.1:8769")
    calls = []
    monkeypatch.setattr(TeleopAvoidanceClient, "_request",
                        lambda self, path, payload: calls.append((self.url, path)))
    client = TeleopAvoidanceClient({"obstacle_avoidance": {"runtime_url": "http://127.0.0.1:8767"}})
    assert client.url == "http://127.0.0.1:8769"
    assert calls == [(client.url, "/health"), (client.url, "/reset")]


def test_snapshot_expired_response_has_a_distinct_error_type(monkeypatch):
    body = BytesIO(json.dumps({"error": "Obstacle snapshot expired", "error_type": "SnapshotExpired"}).encode())
    def fail(_request, timeout):
        raise HTTPError("http://127.0.0.1:8768/project", 409, "Conflict", {}, body)
    monkeypatch.setattr("quest3_robot_data.teleop_avoidance_client.urlopen", fail)
    client = TeleopAvoidanceClient.__new__(TeleopAvoidanceClient)
    client.url = "http://127.0.0.1:8768"
    with pytest.raises(SnapshotExpiredError, match="Obstacle snapshot expired"):
        client._request("/project", {})


def test_enable_failure_is_logged_and_exits_mode(monkeypatch):
    class FailedClient:
        def __init__(self, config):
            raise RuntimeError("avoidance worker unavailable")

    monkeypatch.setattr("quest3_robot_data.teleop_avoidance_client.TeleopAvoidanceClient", FailedClient)
    errors = []
    dedicated = node(errors=errors)
    PoseMapperNode._controllers(dedicated, pose_message(toggle=True))
    assert not dedicated.avoidance_enabled
    assert dedicated.avoidance_event == "failed"
    assert "避障启用失败" in errors[0]
    assert "RuntimeError: avoidance worker unavailable" in errors[0]


def test_projection_failure_holds_action_and_next_hand_input_retries_avoidance():
    errors, infos = [], []
    dedicated = node(errors=errors, infos=infos)
    dedicated.avoidance_enabled = True
    attempts = []
    def compute(*args, **kwargs):
        attempts.append(1)
        if len(attempts) == 1:
            raise RuntimeError("CBF projection failed: recovery_future_or_sweep_unsafe")
        return np.zeros(12), {"needs_projection": False}
    dedicated.avoidance = SimpleNamespace(
        last_step=0.0, period=0.05,
        reset=lambda: None,
        compute=compute,
    )
    client = dedicated.avoidance
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    PoseMapperNode._avoidance_tick(dedicated)
    assert dedicated.avoidance_enabled
    assert dedicated.avoidance is client
    assert dedicated.avoidance_event == "blocked"
    assert "recovery_future_or_sweep_unsafe" in dedicated.avoidance_error
    assert "保持避障模式" in errors[0]
    assert "RuntimeError: CBF projection failed" in errors[0]
    assert dedicated.arms["left"].hold_count >= 1
    assert len(attempts) == 1
    assert all(isinstance(item, tuple) for item in dedicated.arms["left"].published)
    PoseMapperNode._avoidance_tick(dedicated)
    assert len(attempts) == 1  # A failed action is never replayed.
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    dedicated.avoidance_last_dispatch_at -= 0.05
    PoseMapperNode._avoidance_tick(dedicated)
    PoseMapperNode._avoidance_tick(dedicated)
    assert len(attempts) == 2
    assert dedicated.avoidance_enabled
    assert dedicated.avoidance_error == ""
    assert dedicated.avoidance_event == "resumed"
    assert infos == ["避障已恢复，继续发送经检查的动作"]
    assert any(isinstance(item, tuple) and item[1] is not None
               for item in dedicated.arms["left"].published)


def test_failure_preserves_newer_hand_input_received_during_solver():
    dedicated = node()
    dedicated.avoidance_enabled = True
    dedicated.avoidance = SimpleNamespace(last_step=0.0, period=0.05, reset=lambda: None)
    executor = DeferredExecutor()
    dedicated.avoidance_executor = executor
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    first_sequence = dedicated.avoidance_last_attempt_sequence
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    assert dedicated.avoidance_latest.sequence > first_sequence
    executor.future.set_exception(RuntimeError("CBF projection failed"))
    PoseMapperNode._avoidance_tick(dedicated)
    assert dedicated.avoidance_enabled
    dedicated.avoidance_last_dispatch_at -= 0.05
    PoseMapperNode._avoidance_tick(dedicated)
    assert len(executor.calls) == 2
    assert dedicated.avoidance_last_attempt_sequence == dedicated.avoidance_latest.sequence


def test_brief_stale_snapshot_holds_motion_then_recovers():
    errors, infos = [], []
    dedicated = node(errors=errors, infos=infos)
    dedicated.avoidance_enabled = True
    attempts = []
    def compute(*args, **kwargs):
        attempts.append(1)
        if len(attempts) == 1:
            raise SnapshotExpiredError("Obstacle snapshot expired: 1.6s")
        return np.zeros(12), {}
    dedicated.avoidance = SimpleNamespace(last_step=0.0, period=0.05,
                                          reset=lambda: None, compute=compute)
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    PoseMapperNode._avoidance_tick(dedicated)
    assert dedicated.avoidance_enabled
    assert dedicated.avoidance_stale_since is not None
    assert all(not arm.enabled for arm in dedicated.arms.values())
    assert len(errors) == 1
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    dedicated.avoidance_last_dispatch_at -= 0.05
    PoseMapperNode._avoidance_tick(dedicated)
    PoseMapperNode._avoidance_tick(dedicated)
    assert dedicated.avoidance_enabled
    assert dedicated.avoidance_stale_since is None
    assert infos == ["避障已恢复，继续发送经检查的动作"]
    assert len(attempts) == 2


def test_persistent_stale_snapshot_stays_in_avoidance_mode():
    errors = []
    dedicated = node(errors=errors)
    dedicated.avoidance_enabled = True
    dedicated.avoidance_stale_since = time.monotonic() - 0.6
    dedicated.avoidance = SimpleNamespace(
        last_step=0.0, period=0.05,
        reset=lambda: None,
        compute=lambda *args, **kwargs: (_ for _ in ()).throw(SnapshotExpiredError("Obstacle snapshot expired")),
    )
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    PoseMapperNode._avoidance_tick(dedicated)
    assert dedicated.avoidance_enabled
    assert dedicated.avoidance_event == "blocked"
    assert "SnapshotExpiredError" in errors[0]
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    PoseMapperNode._avoidance_tick(dedicated)
    assert dedicated.avoidance_enabled
    assert len(errors) == 1


def test_projection_samples_latest_hand_and_drops_reversed_command():
    dedicated = node()
    dedicated.avoidance_enabled = True
    dedicated.avoidance = SimpleNamespace(last_step=0.0, period=0.05, reset=lambda: None)
    executor = DeferredExecutor()
    dedicated.avoidance_executor = executor
    dedicated.arms["left"].pose_position[0] = 0.01
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    dedicated.arms["left"].pose_position[0] = -0.01
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    assert not executor.calls  # Controller messages only replace the pending intent.
    PoseMapperNode._avoidance_tick(dedicated)
    assert len(executor.calls) == 1
    assert executor.calls[0][1][4]["left"][0, 3] == pytest.approx(-0.01)
    assert executor.calls[0][1][-1] is True
    assert dedicated.arms["left"].hold_count == 0

    dedicated.arms["left"].pose_position[0] = 0.01
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    executor.future.set_result((np.zeros(12), {}))
    PoseMapperNode._avoidance_tick(dedicated)
    assert not any(isinstance(item, tuple) and item[1] is not None
                   for item in dedicated.arms["left"].published)
    assert dedicated.arms["left"].hold_count >= 1


def test_only_intruding_command_holds_and_starts_qp():
    dedicated = node()
    dedicated.avoidance_enabled = True
    inspected = []
    def compute(*args, inspect=False):
        inspected.append(inspect)
        if inspect:
            return None, {"needs_projection": True}
        return np.zeros(12), {"success": True}
    dedicated.avoidance = SimpleNamespace(last_step=0.0, period=0.05,
                                          reset=lambda: None, compute=compute)
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    assert dedicated.arms["left"].hold_count == 0
    PoseMapperNode._avoidance_tick(dedicated)
    assert inspected == [True, False]
    assert dedicated.arms["left"].hold_count == 1
    PoseMapperNode._avoidance_tick(dedicated)
    assert any(isinstance(item, tuple) and item[1] is not None
               for item in dedicated.arms["left"].published)


def test_clear_inspection_robot_progress_does_not_pause_motion():
    dedicated = node()
    dedicated.avoidance_enabled = True
    dedicated.avoidance = SimpleNamespace(last_step=0.0, period=0.05, reset=lambda: None)
    executor = DeferredExecutor()
    dedicated.avoidance_executor = executor
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    dedicated.arms["left"].joints[0] = 0.02
    executor.future.set_result((np.zeros(12), {"needs_projection": False}))
    PoseMapperNode._avoidance_tick(dedicated)
    assert dedicated.arms["left"].hold_count == 0
    assert not any(isinstance(item, tuple) and item[1] is not None
                   for item in dedicated.arms["left"].published)


def test_clear_action_period_counts_inspection_time():
    dedicated = node()
    dedicated.avoidance_enabled = True
    attempts = []
    client = SimpleNamespace(last_step=0.0, period=0.05, reset=lambda: None)
    def compute(*args, **kwargs):
        attempts.append(1)
        client.last_step = time.monotonic()
        return np.zeros(12), {"needs_projection": False}
    client.compute = compute
    dedicated.avoidance = client
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    PoseMapperNode._avoidance_tick(dedicated)
    assert len(attempts) == 1
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    dedicated.avoidance_last_dispatch_at = time.monotonic() - 0.051
    client.last_step = time.monotonic()  # Completion was recent; dispatch was not.
    PoseMapperNode._avoidance_tick(dedicated)
    assert len(attempts) == 2


def test_slower_projection_can_send_if_latest_imminent_command_is_unchanged():
    dedicated = node()
    dedicated.avoidance_enabled = True
    dedicated.avoidance = SimpleNamespace(last_step=0.0, period=0.05, reset=lambda: None)
    executor = DeferredExecutor()
    dedicated.avoidance_executor = executor
    dedicated.arms["left"].pose_position[0] = 0.01
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    PoseMapperNode._avoidance_tick(dedicated)
    dedicated.avoidance_inflight = replace(dedicated.avoidance_inflight,
                                          started_at=time.monotonic() - 0.26)
    PoseMapperNode._controllers(dedicated, pose_message(squeeze=1.0))
    executor.future.set_result((np.zeros(12), {}))
    PoseMapperNode._avoidance_tick(dedicated)
    assert any(isinstance(item, tuple) and item[1] is not None
               for item in dedicated.arms["left"].published)


def test_delayed_controller_message_is_ignored_in_avoidance_mode():
    dedicated = node()
    dedicated.avoidance_enabled = True
    PoseMapperNode._controllers(
        dedicated, pose_message(squeeze=1.0, host_timestamp_ns=700_000_000))
    assert dedicated.avoidance_latest is None
    assert dedicated.arms["left"].hold_count == 1
