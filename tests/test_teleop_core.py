import numpy as np

from quest3_robot_data.teleop_core import AnchoredPoseMapper, ControllerState, GRIP_TO_TOOL_AXES
from quest3_robot_data.transforms import matrix_to_quaternion, rotvec_to_matrix


def hand(position, *, squeeze=1.0, trigger=0.0, orientation=(0, 0, 0, 1)):
    return ControllerState(np.array(position, float), np.array(orientation, float), squeeze, trigger)


def test_release_and_first_grip_do_not_move_tcp():
    mapper = AnchoredPoseMapper(np.eye(4))
    tcp = np.eye(4)
    assert not mapper.update(hand([0, 0, 0], squeeze=0), tcp).enabled
    anchored = mapper.update(hand([1, 2, 3]), tcp)
    assert anchored.enabled and anchored.just_anchored and np.allclose(anchored.pose, tcp)


def test_grip_x_is_same_and_y_z_are_reversed_in_anchored_tcp_frame():
    camera = np.eye(4)
    camera[:3, :3] = rotvec_to_matrix([0.3, -0.2, 0.5])
    mapper = AnchoredPoseMapper(camera, position_gain=2.0)
    hand_rotation = rotvec_to_matrix([0, np.pi / 2, 0])
    tcp = np.eye(4)
    tcp[:3, :3] = rotvec_to_matrix([0, 0, np.pi / 2])
    tcp[:3, 3] = [0.4, -0.2, 0.3]
    q = matrix_to_quaternion(hand_rotation)
    mapper.update(hand([1, 2, 3], orientation=q), tcp)

    # Physical check: X is unchanged, Y and Z are reversed for both hands.
    # Camera tilt cannot change the tool-relative target direction.
    assert np.allclose(GRIP_TO_TOOL_AXES, np.diag([1, -1, -1]))
    assert np.isclose(np.linalg.det(GRIP_TO_TOOL_AXES), 1.0)
    for local_delta in (np.array([0.1, 0, 0]), np.array([0, 0.1, 0]), np.array([0, 0, 0.1])):
        position = np.array([1, 2, 3]) + hand_rotation @ local_delta
        target = mapper.update(hand(position, orientation=q), tcp).pose
        expected_offset = tcp[:3, :3] @ GRIP_TO_TOOL_AXES @ (2 * local_delta)
        assert np.allclose(target[:3, 3], tcp[:3, 3] + expected_offset)
        assert np.allclose(target[:3, :3], tcp[:3, :3])


def test_left_and_right_mapper_state_is_independent():
    left = AnchoredPoseMapper(np.eye(4))
    right = AnchoredPoseMapper(np.eye(4))
    tcp = np.eye(4)
    left.update(hand([0, 0, 0]), tcp)
    assert right.update(hand([10, 0, 0]), tcp).just_anchored
    assert np.allclose(left.update(hand([0.1, 0, 0]), tcp).pose[:3, 3], [0.1, 0, 0])


def test_each_arm_uses_its_own_tool_orientation():
    left = AnchoredPoseMapper(np.eye(4))
    right = AnchoredPoseMapper(np.eye(4))
    left_tcp = np.eye(4)
    right_tcp = np.eye(4)
    right_tcp[:3, :3] = rotvec_to_matrix([0, 0, np.pi / 2])
    left.update(hand([0, 0, 0]), left_tcp)
    right.update(hand([0, 0, 0]), right_tcp)
    for axis in np.eye(3):
        displacement = 0.1 * axis
        left_target = left.update(hand(displacement), left_tcp).pose
        right_target = right.update(hand(displacement), right_tcp).pose
        assert np.allclose(left_target[:3, 3], left_tcp[:3, :3] @ GRIP_TO_TOOL_AXES @ displacement)
        assert np.allclose(right_target[:3, 3], right_tcp[:3, :3] @ GRIP_TO_TOOL_AXES @ displacement)


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


def test_relative_hand_rotation_uses_same_grip_to_tool_axis_alignment():
    mapper = AnchoredPoseMapper(np.eye(4))
    hand_anchor_rotation = rotvec_to_matrix([0, np.pi / 2, 0])
    tcp = np.eye(4)
    tcp[:3, :3] = rotvec_to_matrix([0, 0, np.pi / 2])
    mapper.update(hand([0, 0, 0], orientation=matrix_to_quaternion(hand_anchor_rotation)), tcp)
    local_rotation = rotvec_to_matrix([np.pi / 2, 0, 0])
    target = mapper.update(
        hand([0, 0, 0], orientation=matrix_to_quaternion(hand_anchor_rotation @ local_rotation)), tcp
    ).pose
    expected_rotation = tcp[:3, :3] @ GRIP_TO_TOOL_AXES @ local_rotation @ GRIP_TO_TOOL_AXES.T
    assert np.allclose(target[:3, :3], expected_rotation)


def test_regrip_reanchors_hand_to_current_tcp_without_target_jump():
    mapper = AnchoredPoseMapper(np.eye(4))
    first_tcp = np.eye(4)
    mapper.update(hand([0, 0, 0]), first_tcp)
    mapper.update(hand([0.1, 0, 0]), first_tcp)
    mapper.update(hand([0.1, 0, 0], squeeze=0), first_tcp)
    current_tcp = np.eye(4)
    current_tcp[:3, 3] = [0.3, 0.4, 0.5]
    anchored = mapper.update(hand([4, 5, 6]), current_tcp)
    assert anchored.just_anchored
    assert np.allclose(anchored.pose, current_tcp)


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
