import threading

import numpy as np
import pytest

pytest.importorskip("rclpy")
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
