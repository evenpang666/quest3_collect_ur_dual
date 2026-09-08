import numpy as np
import cv2

from quest3_robot_data.calibrate_camera_charuco import _board_corners, select_spread_corners
from quest3_robot_data.calibrate_tcp import solve_pivot
from quest3_robot_data.transforms import rotvec_to_matrix


def test_pivot_solver_recovers_tcp_offset():
    tcp = np.array([0.02, -0.01, 0.18])
    pivot = np.array([0.4, 0.1, 0.3])
    poses = []
    for rotvec in ([0, 0, 0], [.3, 0, 0], [0, -.4, 0], [0, 0, .5], [.2, -.3, .4], [-.3, .2, -.2]):
        pose = np.eye(4)
        pose[:3, :3] = rotvec_to_matrix(rotvec)
        pose[:3, 3] = pivot - pose[:3, :3] @ tcp
        poses.append(pose)
    solved_tcp, solved_pivot, rms, condition = solve_pivot(poses)
    assert np.allclose(solved_tcp, tcp)
    assert np.allclose(solved_pivot, pivot)
    assert rms < 1e-10 and condition < 100


def test_spread_selector_uses_four_extreme_detected_corners():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(10, 7, 0.037, 0.027, dictionary)
    points = _board_corners(board)
    assert select_spread_corners(list(range(len(points))), points) == [0, 8, 45, 53]


def test_spread_selector_never_invents_undetected_corner():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(10, 7, 0.037, 0.027, dictionary)
    points = _board_corners(board)
    detected = [0, 1, 8, 9, 18, 26, 36, 45]
    selected = select_spread_corners(detected, points)
    assert len(selected) == 4
    assert set(selected) <= set(detected)
