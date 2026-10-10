import json
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")

from std_msgs.msg import String

from quest3_robot_data.pose_mapper import PoseMapperNode
from quest3_robot_data.recorder import DatasetRecorder
from quest3_robot_data.teleop_core import ControllerGate, MappedTarget


def payload(left=True, right=True, squeeze=0.0):
    hand = {
        "position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0],
        "squeeze": squeeze, "trigger": 0.0,
    }
    return String(data=json.dumps({"left": hand if left else None, "right": hand if right else None}))


class FakeArm:
    def __init__(self):
        self.mapper = SimpleNamespace(deadman_threshold=0.55, gripper_close_threshold=0.15)
        self.reset_count = 0
        self.compute_count = 0
        self.published = []

    def reset(self):
        self.reset_count += 1

    def compute(self, hand, recenter):
        self.compute_count += 1
        return MappedTarget(None, False, 0.0)

    def publish(self, result):
        self.published.append(result)


def test_dual_mapper_blocks_both_arms_until_both_tracked_and_clutches_released():
    arms = {role: FakeArm() for role in ("left", "right")}
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
    node = SimpleNamespace(
        arms=arms, get_clock=lambda: clock,
        controller_gate=ControllerGate(("left", "right"), 0.25),
        rearm_required=True, collision_stop=False,
    )

    PoseMapperNode._controllers(node, payload(left=True, right=False))
    assert all(arm.reset_count == 1 and arm.compute_count == 0 for arm in arms.values())

    PoseMapperNode._controllers(node, payload(squeeze=1.0))
    assert node.rearm_required
    assert all(arm.compute_count == 0 for arm in arms.values())

    PoseMapperNode._controllers(node, payload(squeeze=0.0))
    assert not node.rearm_required
    PoseMapperNode._controllers(node, payload(squeeze=1.0))
    assert all(arm.compute_count == 1 for arm in arms.values())


def test_recorder_rejects_start_without_both_controllers():
    errors = []
    published = []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
    gate = ControllerGate(("left", "right"), 0.25)
    gate.observe({"left": object(), "right": None}, 1_000_000_000)
    node = SimpleNamespace(
        recording=False, controller_gate=gate, recording_interrupted_reason=None,
        get_clock=lambda: clock, get_logger=lambda: SimpleNamespace(error=errors.append),
        _publish_status=lambda: published.append(True),
    )

    DatasetRecorder._set_recording(node, True, origin="test")
    assert not node.recording
    assert "right" in node.recording_interrupted_reason
    assert errors and published
