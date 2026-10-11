import threading

import numpy as np
import pytest

pytest.importorskip("rclpy")
from std_msgs.msg import Float64MultiArray
from quest3_robot_data.rtde_servo import ArmWorker


def test_reconnect_requires_release_then_new_clutch_press():
    """A held clutch cannot revive a target created before an RTDE failure."""
    worker = ArmWorker.__new__(ArmWorker)
    worker._lock = threading.Lock()
    worker.enabled = True
    worker.target = np.eye(4)
    worker.target_time = 123.0
    worker._rearm_required = False
    worker._released_since_reconnect = False
    worker.status = "enabled"

    worker._require_rearm()
    assert not worker.enabled
    assert worker.target is None

    worker.set_enabled(True)  # Clutch was held through the recovery.
    assert not worker.enabled
    worker.set_enabled(False)  # Operator releases it.
    worker.set_enabled(True)  # A new press is now accepted.
    assert worker.enabled
    assert not worker._rearm_required


def test_joint_targets_are_accepted_only_in_dedicated_avoidance_mode():
    worker = ArmWorker.__new__(ArmWorker)
    worker._lock = threading.Lock()
    worker.target = np.eye(4)
    worker.joint_target = None
    worker.target_time = 0.0
    worker.avoidance_available = False
    worker.avoidance_mode = False
    target = Float64MultiArray(data=[0.01] * 6)

    worker.set_joint_target(target)
    assert worker.joint_target is None
    assert worker.target is not None

    worker.avoidance_available = True
    worker.set_avoidance_mode(True)
    worker.set_joint_target(target)
    assert np.allclose(worker.joint_target, [0.01] * 6)
    assert worker.target is None


def test_avoidance_hold_clears_target_without_satisfying_rearm():
    worker = ArmWorker.__new__(ArmWorker)
    worker._lock = threading.Lock()
    worker.avoidance_available = True
    worker.avoidance_mode = True
    worker.target = None
    worker.joint_target = np.ones(6)
    worker.target_time = 123.0
    worker.enabled = True
    worker._rearm_required = True
    worker._released_since_reconnect = False
    worker.status = "stopped"

    worker.hold_avoidance()
    assert worker.joint_target is None
    assert not worker.enabled
    assert not worker._released_since_reconnect
    worker.set_enabled(True)
    assert not worker.enabled


def test_avoidance_mode_rejects_late_joint_target_after_toggle_off():
    worker = ArmWorker.__new__(ArmWorker)
    worker._lock = threading.Lock()
    worker.avoidance_available = True
    worker.avoidance_mode = True
    worker.enabled = True
    worker.target = None
    worker.joint_target = np.ones(6)
    worker.target_time = 123.0
    worker.set_avoidance_mode(False)
    worker.set_joint_target(Float64MultiArray(data=[0.02] * 6))
    assert worker.joint_target is None
    assert not worker.enabled
