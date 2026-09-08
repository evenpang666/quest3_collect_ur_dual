import numpy as np

from quest3_robot_data.teleop_core import AnchoredPoseMapper, ControllerState


def hand(position, *, squeeze=1.0, trigger=0.0, orientation=(0, 0, 0, 1)):
    return ControllerState(np.array(position, float), np.array(orientation, float), squeeze, trigger)


def test_release_and_first_grip_do_not_move_tcp():
    mapper = AnchoredPoseMapper(np.eye(4))
    tcp = np.eye(4)
    assert not mapper.update(hand([0, 0, 0], squeeze=0), tcp).enabled
    anchored = mapper.update(hand([1, 2, 3]), tcp)
    assert anchored.enabled and anchored.just_anchored and np.allclose(anchored.pose, tcp)


def test_webxr_motion_is_mapped_through_front_camera():
    mapper = AnchoredPoseMapper(np.eye(4), position_gain=2.0)
    tcp = np.eye(4)
    mapper.update(hand([0, 0, 0]), tcp)
    target = mapper.update(hand([0.1, 0.2, -0.3]), tcp).pose
    # x right, y up, -z forward -> optical x right, -y down, +z forward.
    assert np.allclose(target[:3, 3], [0.2, -0.4, 0.6])


def test_left_and_right_mapper_state_is_independent():
    left = AnchoredPoseMapper(np.eye(4))
    right = AnchoredPoseMapper(np.eye(4))
    tcp = np.eye(4)
    left.update(hand([0, 0, 0]), tcp)
    assert right.update(hand([10, 0, 0]), tcp).just_anchored
    assert np.allclose(left.update(hand([0.1, 0, 0]), tcp).pose[:3, 3], [0.1, 0, 0])


def test_gripper_trigger_is_independent_of_cartesian_deadman():
    mapper = AnchoredPoseMapper(np.eye(4), gripper_close_threshold=0.15)
    tcp = np.eye(4)
    # Side grip released: arm remains disabled, but trigger still closes.
    closing = mapper.update(hand([0, 0, 0], squeeze=0.0, trigger=1.0), tcp)
    assert not closing.enabled
    assert closing.gripper == 1.0
    opening = mapper.update(hand([0, 0, 0], squeeze=0.0, trigger=0.0), tcp)
    assert opening.gripper == 0.0
    # A tracking loss must not turn into an unwanted open command.
    assert mapper.update(None, tcp).gripper is None


def test_controller_rotation_maps_to_tcp_when_enabled():
    mapper = AnchoredPoseMapper(np.eye(4), rotation_gain=1.0, enable_rotation=True)
    tcp = np.eye(4)
    mapper.update(hand([0, 0, 0]), tcp)
    # 90 degrees around WebXR X maps to a non-identity TCP orientation.
    rotated = mapper.update(hand([0, 0, 0], orientation=(np.sqrt(0.5), 0, 0, np.sqrt(0.5))), tcp)
    assert rotated.enabled
    assert not np.allclose(rotated.pose[:3, :3], np.eye(3))


def test_translation_uses_controller_offset_not_absolute_position():
    tcp = np.eye(4)
    first = AnchoredPoseMapper(np.eye(4))
    second = AnchoredPoseMapper(np.eye(4))
    # The two grips begin one metre apart in the XR tracking space, but each
    # then makes exactly the same relative hand motion.
    first.update(hand([0.0, 0.0, 0.0]), tcp)
    second.update(hand([1.0, -2.0, 3.0]), tcp)
    first_target = first.update(hand([0.10, -0.20, 0.30]), tcp).pose
    second_target = second.update(hand([1.10, -2.20, 3.30]), tcp).pose
    assert np.allclose(first_target, second_target)
