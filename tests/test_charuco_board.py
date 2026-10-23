"""Physical corner locations and poses must survive OpenCV layout differences."""
import cv2
import numpy as np
import pytest

from quest3_robot_data.charuco_board import create_board, estimate_pose, interpolate_corners
from quest3_robot_data.calibrate_camera_charuco import corner_grid_position, image_point_label


def printed_board(pattern, rows=8):
    # Render from the physical square pattern, not the model's object points.
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    image = np.full((rows * 100 + 80, 1180), 255, np.uint8)
    marker_id = 0
    parity = 0 if pattern == "legacy" and rows % 2 == 0 else 1
    for row in range(rows):
        for column in range(11):
            x, y = 40 + column * 100, 40 + row * 100
            if (row + column) % 2 != parity:
                image[y:y + 100, x:x + 100] = 0
            else:
                marker = cv2.aruco.drawMarker(dictionary, marker_id, 74)
                image[y + 13:y + 87, x + 13:x + 87] = marker
                marker_id += 1
    return image, dictionary


@pytest.mark.parametrize("pattern,rows,distorted", [
    ("legacy", 8, False), ("opencv", 8, False), ("legacy", 7, False), ("legacy", 8, True),
])
def test_printed_board_corners_match_physical_grid_and_pose(pattern, rows, distorted):
    image, dictionary = printed_board(pattern, rows)
    # Exercise perspective as well as the front-on grid.
    source = np.float32([[40, 40], [1140, 40], [1140, 40 + rows * 100], [40, 40 + rows * 100]])
    matrix = np.array([[1200., 0, 590], [0, 1200., 440], [0, 0, 1]])
    rotation = np.array([.25, -.15, .07])
    translation = np.array([-.38, -.25, 1.3])
    physical = np.float32([[0, 0, 0], [.77, 0, 0], [.77, rows * .07, 0], [0, rows * .07, 0]])
    destination, _ = cv2.projectPoints(physical, rotation, translation, matrix, None)
    homography = cv2.getPerspectiveTransform(source, destination.reshape(-1, 2))
    image = cv2.warpPerspective(image, homography, (1180, 880), borderValue=255)
    distortion = np.array([-.16, .04, .001, -.002, 0.]) if distorted else None
    if distorted:
        y, x = np.indices(image.shape, dtype=np.float32)
        undistorted = cv2.undistortPoints(np.stack([x, y], axis=-1).reshape(-1, 1, 2),
                                        matrix, distortion, P=matrix).reshape(*image.shape, 2)
        image = cv2.remap(image, undistorted[:, :, 0], undistorted[:, :, 1],
                          cv2.INTER_LINEAR, borderValue=255)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    board = create_board(11, rows, .07, .052, pattern)
    count, corners, ids = interpolate_corners(marker_corners, marker_ids, image, board, matrix, distortion)
    assert count == 10 * (rows - 1)
    by_id = dict(zip(ids.reshape(-1), corners.reshape(-1, 2)))
    for corner_id in (0, 8, 9, (rows - 1) * 10 - 1):
        row, column = divmod(corner_id, 10)
        expected, _ = cv2.projectPoints(np.float32([[.07 * (column + 1), .07 * (row + 1), 0]]),
                                        rotation, translation, matrix, distortion)
        assert np.linalg.norm(by_id[corner_id] - expected.reshape(2)) < 1.2
    ok, rvec, tvec = estimate_pose(corners, ids, board, matrix, distortion)
    assert ok
    assert np.linalg.norm(tvec.reshape(3) - translation) < .003
    assert np.linalg.norm(rvec.reshape(3) - rotation) < .01


@pytest.mark.parametrize("printed,configured", [("legacy", "opencv"), ("opencv", "legacy")])
@pytest.mark.parametrize("scale", [1., .4])
def test_wrong_even_row_pattern_is_rejected_instead_of_pruning_rows(printed, configured, scale):
    image, dictionary = printed_board(printed)
    image = cv2.resize(image, None, fx=scale, fy=scale)
    markers, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    matrix = np.array([[1000., 0, 590], [0, 1000., 440], [0, 0, 1]])
    with pytest.raises(SystemExit, match="--board-pattern"):
        interpolate_corners(markers, ids, image, create_board(11, 8, .07, .052, configured), matrix, None)


def test_occlusion_does_not_invent_unsupported_corners():
    image, dictionary = printed_board("legacy")
    # Occlude the board's upper-left corner, including its first marker.
    image[40:240, 40:240] = 180
    markers, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    board = create_board(11, 8, .07, .052)
    matrix = np.array([[1000., 0, 590], [0, 1000., 440], [0, 0, 1]])
    count, corners, corner_ids = interpolate_corners(markers, ids, image, board, matrix, None)
    assert count > 6 and len(corners) == count
    assert 0 not in corner_ids
    assert 9 in corner_ids


def test_one_marker_row_parity_cannot_validate_pattern():
    image, dictionary = printed_board("legacy")
    markers, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    board = create_board(11, 8, .07, .052)
    selected = [i for i, value in enumerate(ids.reshape(-1)) if board.marker_rows[value] % 2 == 0]
    count, _, _ = interpolate_corners([markers[i] for i in selected], ids[selected], image, board,
                                      np.eye(3), None)
    assert count == 0


def test_misplaced_marker_is_rejected_instead_of_silently_dropped():
    image, dictionary = printed_board("legacy")
    markers, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    markers = [value.copy() for value in markers]
    markers[0] += [80, 0]
    matrix = np.array([[1000., 0, 590], [0, 1000., 440], [0, 0, 1]])
    with pytest.raises(SystemExit, match="marker geometry"):
        interpolate_corners(markers, ids, image, create_board(11, 8, .07, .052), matrix, None)


def test_corner_id_and_display_label_have_explicit_origin():
    assert corner_grid_position(0, 11) == (1, 1)
    assert corner_grid_position(9, 11) == (1, 10)
    assert image_point_label(9, 11) == "ID9 R1C10"


def test_collinear_corners_do_not_produce_a_pose():
    board = create_board(11, 8, .07, .052)
    ok, _, _ = estimate_pose(np.zeros((6, 1, 2), np.float32), np.arange(6), board,
                             np.eye(3), None)
    assert not ok
