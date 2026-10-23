"""Explicit ChArUco geometry, independent of OpenCV's even-row layout changes.

OpenCV 4.6 cannot switch to the pre-4.6 (legacy) printed pattern. Estimate
the board pose from its marker corners, project the adjacent chess corners,
and refine those projections against the image, as in ChArUco interpolation.
All coordinates use the printed board's upper-left corner as the origin.
"""
from dataclasses import dataclass

import cv2
import numpy as np


BOARD_MODEL_VERSION = 2


@dataclass
class CharucoBoardModel:
    chessboardCorners: np.ndarray
    objPoints: list[np.ndarray]
    ids: np.ndarray
    adjacent_markers: list[tuple[int, ...]]
    marker_rows: list[int]


def create_board(squares_x, squares_y, square_size, marker_size, pattern="legacy"):
    """Legacy even-row boards have a marker in the upper-left square."""
    if pattern not in ("legacy", "opencv"):
        raise ValueError(f"unknown ChArUco pattern: {pattern}")
    marker_parity = 0 if pattern == "legacy" and squares_y % 2 == 0 else 1
    inset = (square_size - marker_size) / 2
    markers, cells, marker_rows = [], {}, []
    for row in range(squares_y):
        for column in range(squares_x):
            if (row + column) % 2 != marker_parity:
                continue
            cells[row, column] = len(markers)
            marker_rows.append(row)
            x, y = column * square_size + inset, row * square_size + inset
            markers.append(np.array([
                [x, y, 0], [x + marker_size, y, 0],
                [x + marker_size, y + marker_size, 0], [x, y + marker_size, 0],
            ], dtype=np.float32))
    corners, adjacent = [], []
    for row in range(1, squares_y):
        for column in range(1, squares_x):
            corners.append([column * square_size, row * square_size, 0])
            adjacent.append(tuple(cells[r, c] for r in (row - 1, row)
                                  for c in (column - 1, column) if (r, c) in cells))
    return CharucoBoardModel(np.asarray(corners, dtype=np.float32), markers,
                            np.arange(len(markers), dtype=np.int32).reshape(-1, 1),
                            adjacent, marker_rows)


def interpolate_corners(marker_corners, marker_ids, image, board, camera_matrix, distortion):
    """Refine only inner corners supported by both adjacent decoded markers.

    Validate *all* known markers. RANSAC pruning can otherwise hide an incorrect
    pattern by retaining alternating rows with a common one-square offset.
    """
    if marker_ids is None:
        return 0, None, None
    detected = {int(value): np.asarray(corners, dtype=np.float32).reshape(4, 2)
                for value, corners in zip(np.asarray(marker_ids).reshape(-1), marker_corners)
                if 0 <= int(value) < len(board.ids)}
    # A single row parity cannot distinguish the two even-row layouts.
    if len(detected) < 6 or len({board.marker_rows[value] % 2 for value in detected}) < 2:
        return 0, None, None
    objects = np.concatenate([board.objPoints[value] for value in detected])
    pixels = np.concatenate(list(detected.values()))
    ok, rvec, tvec = cv2.solvePnP(objects, pixels, camera_matrix, distortion)
    if not ok:
        return 0, None, None
    projected, _ = cv2.projectPoints(objects, rvec, tvec, camera_matrix, distortion)
    errors = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1).reshape(-1, 4)
    marker_edges = [np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)
                    for corners in detected.values()]
    tolerance = max(1.5, min(8.0, .2 * float(np.median(marker_edges))))
    if np.max(np.sqrt(np.mean(errors ** 2, axis=1))) > tolerance:
        raise SystemExit("ChArUco marker geometry does not match the board; check "
                         "--board-pattern legacy/opencv, dimensions and camera intrinsics; "
                         "do not reuse the old observation")
    projected, _ = cv2.projectPoints(board.chessboardCorners, rvec, tvec,
                                    camera_matrix, distortion)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    height, width = gray.shape
    corners, ids = [], []
    for corner_id, neighbors in enumerate(board.adjacent_markers):
        if not all(value in detected for value in neighbors):
            continue
        initial = projected[corner_id].astype(np.float32).reshape(1, 1, 2)
        distance = min(float(np.linalg.norm(pixel - initial.reshape(2)))
                       for value in neighbors for pixel in detected[value])
        window = max(2, min(10, int(distance / 2)))
        u, v = initial.reshape(2)
        if not (window + 1 <= u < width - window - 1 and
                window + 1 <= v < height - window - 1):
            continue
        refined = cv2.cornerSubPix(gray, initial.copy(), (window, window), (-1, -1),
                                  (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        if np.linalg.norm(refined - initial) > window:
            continue
        corners.append(refined.reshape(2))
        ids.append(corner_id)
    if not ids:
        return 0, None, None
    return (len(ids), np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2),
            np.asarray(ids, dtype=np.int32).reshape(-1, 1))


def estimate_pose(corners, ids, board, camera_matrix, distortion):
    objects = board.chessboardCorners[np.asarray(ids).reshape(-1)]
    if len(objects) < 4 or np.linalg.matrix_rank(objects - objects.mean(axis=0)) < 2:
        return False, None, None
    return cv2.solvePnP(objects, corners, camera_matrix, distortion)
