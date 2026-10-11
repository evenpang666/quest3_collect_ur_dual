"""Avoidance scales servoL motion limits to its own control period."""

import numpy as np
from types import SimpleNamespace

from quest3_robot_data.target_limiter import avoidance_cartesian_limits, limit_cartesian_target
from quest3_robot_data.teleop_avoidance import TeleopAvoidance
from quest3_robot_data.transforms import matrix_to_rotvec, rotvec_to_matrix


def test_limits_imminent_translation_and_rotation_without_changing_full_goal():
    actual = np.eye(4)
    goal = np.eye(4)
    goal[:3, 3] = [0.2, 0.0, 0.0]
    goal[:3, :3] = rotvec_to_matrix([0.0, 0.0, 0.5])
    command = limit_cartesian_target(actual, goal, 0.0036, 0.024)
    assert np.allclose(command[:3, 3], [0.0036, 0.0, 0.0])
    np.testing.assert_allclose(np.linalg.norm(matrix_to_rotvec(command[:3, :3])), 0.024)
    assert goal[0, 3] == 0.2


def test_avoidance_tick_preserves_servo_cartesian_speed_limit():
    config = {"control_hz": 125.0, "rtde": {
        "max_target_jump_m": 0.0036, "max_target_jump_rad": 0.024}}
    translation, rotation = avoidance_cartesian_limits(config, 0.05)
    np.testing.assert_allclose([translation, rotation], [0.0225, 0.15])


def test_cbf_receives_the_bounded_next_rtde_command():
    adapter = TeleopAvoidance.__new__(TeleopAvoidance)
    adapter._tcp = lambda q, index: np.eye(4)
    bounded = {}
    def nominal(q, goals, moving):
        bounded.update(goals)
        return np.zeros(12)
    adapter._nominal = nominal
    adapter.sampler = SimpleNamespace(calibration_digest="valid")
    adapter.safety = {"collision_margin_m": 0.01, "inter_arm_margin_m": 0.035}
    adapter.CollisionChecker = lambda *args, **kwargs: SimpleNamespace(state_safe=lambda q: False)
    projection_args = {}
    def project(*args, **kwargs):
        projection_args.update(kwargs)
        return np.zeros(12), {"success": True}
    adapter.project = SimpleNamespace(project=project)
    adapter.motion_safe = lambda *args: True
    adapter.boxes_from_json = lambda snapshot: []
    adapter.read_snapshot = lambda url, age: {}
    adapter.validate_calibration_identity = lambda snapshot, digest: None
    adapter.url, adapter.age = "http://localhost/snapshot.json", 1.5
    adapter.ignore_snapshot_age = False
    adapter.period, adapter.speed, adapter.acceleration = 0.05, 0.5, 1.2
    adapter.max_translation, adapter.max_rotation = 0.0225, 0.15
    adapter.previous_delta = np.zeros(12)
    goal = np.eye(4)
    goal[0, 3] = 0.2
    target, info = adapter.compute(
        np.zeros(12), np.zeros(2), {"left": goal}, {"left": True, "right": False},
        {"left": np.eye(4), "right": np.eye(4)},
    )
    assert bounded["left"][0, 3] == 0.0225
    assert np.allclose(projection_args["lower"][6:], np.zeros(6))
    assert np.allclose(projection_args["upper"][6:], np.zeros(6))
    assert np.allclose(target, np.zeros(12))
    assert info["success"]


def test_clear_next_rtde_command_skips_qp():
    adapter = TeleopAvoidance.__new__(TeleopAvoidance)
    adapter._tcp = lambda q, index: np.eye(4)
    delta = np.full(12, 0.001)
    delta[6:] = 0
    adapter._nominal = lambda q, goals, moving: delta.copy()
    adapter.sampler = SimpleNamespace(calibration_digest="valid")
    adapter.safety = {"collision_margin_m": 0.01, "inter_arm_margin_m": 0.035}
    adapter.CollisionChecker = lambda *args, **kwargs: SimpleNamespace(state_safe=lambda q: True)
    adapter.project = SimpleNamespace(project=lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError("safe action must not enter QP")))
    adapter.motion_safe = lambda *args: True
    adapter.boxes_from_json = lambda snapshot: []
    adapter.read_snapshot = lambda url, age: {}
    adapter.validate_calibration_identity = lambda snapshot, digest: None
    adapter.url, adapter.age = "http://localhost/snapshot.json", 1.5
    adapter.ignore_snapshot_age = False
    adapter.period, adapter.speed, adapter.acceleration = 0.05, 0.5, 1.2
    adapter.max_translation, adapter.max_rotation = 0.0225, 0.15
    adapter.previous_delta = np.zeros(12)
    args = (np.zeros(12), np.zeros(2), {"left": np.eye(4)},
            {"left": True, "right": False}, {"left": np.eye(4), "right": np.eye(4)})
    target, info = adapter.compute(*args, optimize=False)
    np.testing.assert_allclose(target, delta)
    assert info["needs_projection"] is False
    np.testing.assert_allclose(adapter.previous_delta, delta)


def test_intruding_inspection_defers_qp_until_robot_is_held():
    adapter = TeleopAvoidance.__new__(TeleopAvoidance)
    adapter._tcp = lambda q, index: np.eye(4)
    adapter._nominal = lambda q, goals, moving: np.zeros(12)
    adapter.sampler = SimpleNamespace(calibration_digest="valid")
    adapter.safety = {"collision_margin_m": 0.01, "inter_arm_margin_m": 0.035}
    adapter.CollisionChecker = lambda *args, **kwargs: SimpleNamespace(state_safe=lambda q: False)
    adapter.project = SimpleNamespace(project=lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError("inspect must not run QP")))
    adapter.motion_safe = lambda *args: False
    adapter.boxes_from_json = lambda snapshot: []
    adapter.read_snapshot = lambda url, age: {}
    adapter.validate_calibration_identity = lambda snapshot, digest: None
    adapter.url, adapter.age = "http://localhost/snapshot.json", 1.5
    adapter.ignore_snapshot_age = False
    adapter.period, adapter.speed, adapter.acceleration = 0.05, 0.5, 1.2
    adapter.max_translation, adapter.max_rotation = 0.0036, 0.024
    adapter.previous_delta = np.zeros(12)
    target, info = adapter.compute(
        np.zeros(12), np.zeros(2), {"left": np.eye(4)},
        {"left": True, "right": False},
        {"left": np.eye(4), "right": np.eye(4)}, optimize=False)
    assert target is None
    assert info["needs_projection"] is True


def test_dedicated_teleop_reads_snapshot_without_age_cutoff():
    adapter = TeleopAvoidance.__new__(TeleopAvoidance)
    adapter.url = "http://localhost/snapshot.json"
    adapter.age = 2.0
    adapter.ignore_snapshot_age = True
    calls = []
    adapter.read_snapshot = lambda *args, **kwargs: (
        calls.append((args, kwargs)) or {"frame": 7})
    assert adapter._snapshot() == {"frame": 7}
    assert calls == [((adapter.url, float("inf")), {"http_timeout_s": 0.5})]
