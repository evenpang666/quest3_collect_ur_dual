"""Calibrate a fixed camera to a UR base from a table-fixed ChArUco board.

The calibrated TCP touches known ChArUco corners to establish base_T_board.
The camera observes the same stationary board to establish camera_T_board.
"""
from __future__ import annotations

import argparse
from copy import copy
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path
from uuid import uuid4

import cv2
import numpy as np
import yaml

from .calibrate_camera import average_transforms, rotation_error_deg
from .transforms import dump_transform


DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in ("DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000")
}


def rigid_transform(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float]:
    """Return target_T_source and point-fit RMS."""
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3 or len(source) < 3:
        raise ValueError("source/target must be matching Nx3 arrays with N >= 3")
    source_center, target_center = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_center).T @ (target - target_center))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation, translation
    residual = (rotation @ source.T).T + translation - target
    rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    return result, rms


def _board_corners(board: object) -> np.ndarray:
    values = board.chessboardCorners if hasattr(board, "chessboardCorners") else board.getChessboardCorners()
    return np.asarray(values, dtype=float).reshape(-1, 3)


def filter_inconsistent_markers(marker_corners, marker_ids, board):
    """Discard marker IDs whose physical positions disagree with this board.

    Some printed ChArUco boards use the opposite alternating marker layout
    from the installed OpenCV version. Those markers can be decoded correctly
    while having the wrong board coordinates, corrupting corner interpolation.
    """
    if marker_ids is None or len(marker_ids) < 6 or len(marker_corners) != len(marker_ids):
        return marker_corners, marker_ids
    board_centers = {
        int(marker_id): np.asarray(points, dtype=np.float32).reshape(4, 3)[:, :2].mean(axis=0)
        for marker_id, points in zip(np.asarray(board.ids).reshape(-1), board.objPoints)
    }
    indices = [index for index, marker_id in enumerate(marker_ids.reshape(-1))
               if int(marker_id) in board_centers]
    if len(indices) < 6:
        raise SystemExit("fewer than six known ChArUco markers; check board parameters and visibility")
    expected = np.asarray([board_centers[int(marker_ids[index, 0])] for index in indices], dtype=np.float32)
    observed = np.asarray([np.asarray(marker_corners[index]).reshape(4, 2).mean(axis=0)
                           for index in indices], dtype=np.float32)
    _, inliers = cv2.findHomography(expected, observed, cv2.RANSAC, 8.0)
    if inliers is None or int(inliers.sum()) < max(6, int(np.ceil(0.6 * len(indices)))):
        raise SystemExit("ChArUco marker geometry does not match the configured board")
    keep = np.zeros(len(marker_ids), dtype=bool)
    keep[np.asarray(indices)[inliers.reshape(-1).astype(bool)]] = True
    return [corners for corners, selected in zip(marker_corners, keep) if selected], marker_ids[keep]


def select_spread_corners(detected_ids: list[int], board_points: np.ndarray) -> list[int]:
    """Select four detected corners with the largest mutual separation."""
    ids = sorted(set(int(value) for value in detected_ids))
    if len(ids) < 4:
        raise ValueError("at least four detected ChArUco corners are required")
    best_ids: tuple[int, ...] | None = None
    best_score: tuple[float, float] | None = None
    for candidate in combinations(ids, 4):
        points = board_points[list(candidate), :2]
        if np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
            continue
        distances = [float(np.linalg.norm(points[a] - points[b])) for a, b in combinations(range(4), 2)]
        # Total pairwise distance gives the broadest board coverage; minimum
        # distance breaks ties in favour of an evenly spread quadrilateral.
        score = (sum(distances), min(distances))
        if best_score is None or score > best_score:
            best_ids, best_score = candidate, score
    if best_ids is None:
        raise ValueError("detected corners do not contain four non-collinear points")
    return list(best_ids)


def corner_grid_position(corner_id: int, squares_x: int) -> tuple[int, int]:
    """Return one-based inner-corner row/column measured from the board origin."""
    row, column = divmod(corner_id, squares_x - 1)
    return row + 1, column + 1


def touch_point_label(corner_id: int, squares_x: int) -> str:
    row, column = corner_grid_position(corner_id, squares_x)
    return f"板坐标原点起第 {row} 行、第 {column} 列内角（corner ID {corner_id}）"


def describe_touch_points(
    touch_ids: list[int], pixel_by_id: dict[int, list[float]], squares_x: int
) -> list[dict[str, object]]:
    """Build human-readable board-grid and image locations for selected points."""
    if not touch_ids:
        return []
    center_u = float(np.mean([pixel_by_id[value][0] for value in touch_ids]))
    center_v = float(np.mean([pixel_by_id[value][1] for value in touch_ids]))
    result = []
    for corner_id in touch_ids:
        row, column = corner_grid_position(corner_id, squares_x)
        u, v = (float(value) for value in pixel_by_id[corner_id])
        result.append({
            "id": corner_id,
            "board_row_from_origin": row,
            "board_column_from_origin": column,
            "image_u_px": round(u, 1),
            "image_v_px": round(v, 1),
            "image_position": ("左" if u < center_u else "右") + ("上" if v < center_v else "下"),
        })
    return result


def start_color_pipeline(camera_serial: str):
    """Open an advertised RGB profile, including the restricted USB 2 profiles."""
    import pyrealsense2 as rs

    context = rs.context()
    device = next((device for device in context.query_devices()
                   if device.get_info(rs.camera_info.serial_number) == camera_serial), None)
    if device is None:
        raise SystemExit(f"RealSense camera not connected: {camera_serial}")
    available = set()
    for sensor in device.query_sensors():
        for profile in sensor.get_stream_profiles():
            if profile.stream_type() == rs.stream.color and profile.format() == rs.format.bgr8:
                video = profile.as_video_stream_profile()
                available.add((video.width(), video.height(), profile.fps()))
    # Preserve high-resolution board corners on USB 2 by reducing frame rate
    # before resolution. Calibration uses a sample count, not a fixed duration.
    preferred = [(1280, 720, 30), (1280, 720, 15), (640, 480, 30), (640, 480, 15)]
    candidates = [value for value in preferred if value in available]
    candidates.extend(sorted(available - set(candidates), key=lambda value: (-value[0] * value[1], -value[2])))
    if not candidates:
        raise SystemExit(f"RealSense {camera_serial} has no BGR8 color profiles")
    errors = []
    for width, height, fps in candidates:
        pipeline, cfg = rs.pipeline(context), rs.config()
        cfg.enable_device(camera_serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            profile = pipeline.start(cfg)
        except RuntimeError as exc:
            errors.append(f"{width}x{height}@{fps}: {exc}")
            # A failed start can leave partial resources behind.
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            if "busy" in str(exc).lower():
                raise SystemExit(f"RealSense {camera_serial} is busy; stop camera-owning processes: {exc}") from exc
            print(f"RGB 格式 {width}×{height}@{fps} 启动失败，尝试其他支持格式: {exc}")
            continue
        stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        print(f"RGB ready: {camera_serial}, {stream.width()}×{stream.height()}@{stream.fps()} BGR8")
        return pipeline, profile
    raise SystemExit(f"cannot start RealSense {camera_serial} RGB stream: {'; '.join(errors)}")


def add_board_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--squares-x", type=int, required=True)
    parser.add_argument("--squares-y", type=int, required=True)
    parser.add_argument("--square-size-m", type=float, required=True)
    parser.add_argument("--marker-size-m", type=float, required=True)
    parser.add_argument("--dictionary", choices=sorted(DICTIONARIES), required=True)
    parser.add_argument(
        "--touch-corner-ids", type=int, nargs="+",
        help="optional override; otherwise selected from corners detected in the locked frame",
    )
    parser.add_argument("--camera-samples", type=int, default=30)
    parser.add_argument("--recapture-camera", action="store_true", help="ignore the saved board observation")
    parser.add_argument("--retouch-corner-ids", type=int, nargs="+", default=[])
    parser.add_argument("--max-touch-rms-mm", type=float, default=2.0)
    parser.add_argument("--max-camera-spread-mm", type=float, default=3.0)
    parser.add_argument("--max-board-motion-mm", type=float, default=5.0)
    parser.add_argument("--max-board-motion-deg", type=float, default=2.0)


def observe_camera(args, board, board_points, dictionary, signature, *, defer_touch_selection=False):
    """Lock a board observation independently for this camera and arm."""
    observation_path = args.output.with_name(args.output.stem + "_camera_observation.yaml")
    touch_ids = list(args.touch_corner_ids) if args.touch_corner_ids else None
    if observation_path.exists() and not args.recapture_camera:
        observation = yaml.safe_load(observation_path.read_text(encoding="utf-8"))
        if observation.get("signature") != signature:
            raise SystemExit(f"saved camera observation does not match current board/camera: {observation_path}")
        if touch_ids is None and not defer_touch_selection:
            if not observation.get("touch_corner_ids"):
                raise SystemExit("saved observation has no auto-selected touch points; rerun with --recapture-camera")
            touch_ids = [int(value) for value in observation["touch_corner_ids"]]
        if "detected_corner_ids" not in observation:
            observation["detected_corner_ids"] = touch_ids or []
        print(f"复用已保存的相机/标定板观测: {observation_path}")
    else:
        import pyrealsense2 as rs

        print(f"正在独占打开 {args.camera_frame} ({args.camera_serial})；请先停止占用它的 RealSense 节点。")
        pipeline, profile = start_color_pipeline(args.camera_serial)
        stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = stream.get_intrinsics()
        camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], float)
        distortion = np.asarray(intr.coeffs, float)
        intrinsics = {
            "width": intr.width, "height": intr.height, "fx": intr.fx, "fy": intr.fy,
            "ppx": intr.ppx, "ppy": intr.ppy, "distortion": intr.coeffs,
        }
        parameters = cv2.aruco.DetectorParameters_create()
        camera_to_board_samples: list[np.ndarray] = []
        capturing = False
        locked_image: np.ndarray | None = None
        point_descriptions: list[dict[str, object]] = []
        detected_touch_candidates: set[int] = set()
        print("保持板和相机固定；检测稳定后只按一次空格，程序将自动完成观测。q 取消。")
        try:
            while len(camera_to_board_samples) < args.camera_samples:
                frame = pipeline.wait_for_frames().get_color_frame()
                if not frame:
                    continue
                image = np.asanyarray(frame.get_data())
                marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
                marker_corners, marker_ids = filter_inconsistent_markers(marker_corners, marker_ids, board)
                charuco_count, pose = 0, None
                if marker_ids is not None:
                    count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        marker_corners, marker_ids, image, board,
                        cameraMatrix=camera_matrix, distCoeffs=distortion,
                    )
                    charuco_count = int(count or 0)
                    if charuco_count >= 6:
                        cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids)
                        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                            charuco_corners, charuco_ids, board, camera_matrix, distortion, None, None
                        )
                        if ok:
                            pose = np.eye(4)
                            pose[:3, :3] = cv2.Rodrigues(rvec)[0]
                            pose[:3, 3] = np.asarray(tvec).reshape(3)
                pixel_by_id = (
                    {int(value): point.tolist() for value, point in zip(charuco_ids.reshape(-1), charuco_corners.reshape(-1, 2))}
                    if charuco_count >= 6 else {}
                )
                if capturing and pose is not None:
                    camera_to_board_samples.append(pose)
                if touch_ids is not None:
                    for corner_id in touch_ids:
                        if corner_id not in pixel_by_id:
                            continue
                        u, v = np.rint(pixel_by_id[corner_id]).astype(int)
                        cv2.circle(image, (u, v), 14, (0, 0, 255), 3)
                        cv2.putText(image, f"P{corner_id}", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(
                    image,
                    f"corners={charuco_count} auto={len(camera_to_board_samples)}/{args.camera_samples}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2,
                )
                cv2.imshow(f"{args.camera_frame} ChArUco calibration", image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    raise SystemExit("calibration cancelled")
                if key == 32 and pose is not None and not capturing:
                    if touch_ids is None and not defer_touch_selection:
                        touch_ids = select_spread_corners(list(pixel_by_id), board_points)
                    detected_touch_candidates = set(pixel_by_id)
                    locked_pixels = dict(pixel_by_id)
                    missing = [value for value in (touch_ids or []) if value not in pixel_by_id]
                    if missing:
                        print(f"当前帧未识别指定角点 {missing}，请等待后重新按空格")
                        continue
                    point_descriptions = describe_touch_points(touch_ids or [], pixel_by_id, args.squares_x)
                    locked_image = image.copy()
                    for item in point_descriptions:
                        u, v = int(round(item["image_u_px"])), int(round(item["image_v_px"]))
                        cv2.circle(locked_image, (u, v), 16, (0, 0, 255), 4)
                        cv2.putText(locked_image, f"P{item['id']}", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
                    capturing = True
                    print("已锁定标定板；本臂完成前请保持板和两台相机固定。")
                    for item in point_descriptions:
                        print(
                            f"  P{item['id']}: 图像{item['image_position']}，"
                            f"像素(u={item['image_u_px']}, v={item['image_v_px']})，"
                            f"板坐标原点起第{item['board_row_from_origin']}行"
                            f"第{item['board_column_from_origin']}列"
                        )
                    print("正在自动采集稳定帧……")
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
        camera_to_board = average_transforms(camera_to_board_samples)
        camera_sample_count = len(camera_to_board_samples)
        camera_spread = float(max(
            np.linalg.norm(value[:3, 3] - camera_to_board[:3, 3]) * 1000
            for value in camera_to_board_samples
        ))
        rotation_spread = float(max(
            rotation_error_deg(value, camera_to_board) for value in camera_to_board_samples
        ))
        selected_image_path = args.output.with_name(args.output.stem + "_selected_points.png")
        if locked_image is not None:
            cv2.imwrite(str(selected_image_path), locked_image)
            print(f"推荐触点标注图已保存: {selected_image_path}")
        observation = {
            "signature": signature,
            "observation_id": uuid4().hex,
            "detected_corner_pixels": {int(k): v for k, v in locked_pixels.items()},
            "camera_to_board": camera_to_board.tolist(),
            "camera_samples": camera_sample_count,
            "color_fps": stream.fps(),
            "touch_corner_ids": touch_ids,
            "detected_corner_ids": sorted(detected_touch_candidates),
            "touch_point_descriptions": point_descriptions,
            "camera_translation_max_spread_mm": camera_spread,
            "camera_rotation_max_spread_deg": rotation_spread,
            "intrinsics": intrinsics,
        }
        observation_path.write_text(yaml.safe_dump(observation, sort_keys=False), encoding="utf-8")
        print(f"相机观测检查点已保存: {observation_path}")

    return observation


def touch_board(args, board_points, touch_ids, detected_touch_candidates, signature):
    """Read this arm's active TCP at four corners; never command robot motion."""
    from rtde_receive import RTDEReceiveInterface

    touch_path = args.output.with_name(args.output.stem + "_touches.yaml")
    touch_data = (
        yaml.safe_load(touch_path.read_text(encoding="utf-8"))
        if touch_path.exists() and not args.recapture_camera else {}
    ) or {}
    if touch_data and touch_data.get("signature") != signature:
        raise SystemExit(f"saved touch checkpoint does not match current board/camera: {touch_path}")
    if touch_data.get("touch_corner_ids"):
        touch_ids = [int(value) for value in touch_data["touch_corner_ids"]]
        if (len(touch_ids) != 4 or len(set(touch_ids)) != 4
                or any(value < 0 or value >= len(board_points) for value in touch_ids)
                or np.linalg.matrix_rank(board_points[touch_ids] - board_points[touch_ids].mean(axis=0)) < 2):
            raise SystemExit("saved touch corners must be four valid, non-collinear board corners")
    touched_by_id = (
        {} if args.recapture_camera else
        {int(key): np.asarray(value, dtype=float) for key, value in touch_data.get("touches", {}).items()}
    )
    for corner_id in args.retouch_corner_ids:
        touched_by_id.pop(corner_id, None)
    print("确认示教器已激活刚标定的 TCP。依次将 TCP 尖端精确触碰指定 ChArUco 内角点。")
    print(f"行列从 1 开始，仅数棋盘格内部交点；本板每行 {args.squares_x - 1} 个内角。位置可对照已保存的 P<角点ID> 标注图。")
    receiver = RTDEReceiveInterface(args.robot_ip)
    try:
        for index in range(len(touch_ids)):
            corner_id = touch_ids[index]
            label = touch_point_label(corner_id, args.squares_x)
            if corner_id in touched_by_id:
                print(f"[{index + 1}/{len(touch_ids)}] 复用 {label} 检查点")
                continue
            while True:
                answer = input(
                    f"[{index + 1}/{len(touch_ids)}] 触碰{label}后按 Enter；"
                    f"输入 id=N 可替换为可达内角（ID 0–{len(board_points) - 1}），q 取消: "
                ).strip().lower()
                if answer == "q":
                    raise SystemExit("calibration cancelled")
                if not answer:
                    break
                if not answer.startswith("id="):
                    print("请输入 Enter、id=N 或 q；尚未记录触点。")
                    continue
                try:
                    replacement = int(answer.split("=", 1)[1])
                except ValueError:
                    print("角点 ID 必须是整数；请重新输入。")
                    continue
                if not 0 <= replacement < len(board_points):
                    print(f"角点 ID 超出本板范围 0–{len(board_points) - 1}；请重新输入。")
                    continue
                if replacement in touch_ids[:index] + touch_ids[index + 1:]:
                    print(f"corner ID {replacement} 已用于其他触点；请选择不同内角。")
                    continue
                candidate_ids = list(touch_ids)
                candidate_ids[index] = replacement
                candidate_points = board_points[candidate_ids]
                if np.linalg.matrix_rank(candidate_points - candidate_points.mean(axis=0)) < 2:
                    print("替换后四个触点共线，无法标定；请选择其他内角。")
                    continue
                replacement_label = touch_point_label(replacement, args.squares_x)
                if replacement not in detected_touch_candidates:
                    # All corners have known board coordinates, even when not
                    # detected in the locked images. Camera pose comes from the
                    # other observed corners on the same stationary board.
                    print(f"{replacement_label}未在两台锁定图中共同识别；"
                          "请按行列确认实物上的正确内角，保持板固定。")
                confirmation = input(f"改为{replacement_label}；触碰后按 Enter，q 取消: ").strip().lower()
                if confirmation == "q":
                    raise SystemExit("calibration cancelled")
                if confirmation:
                    print("尚未记录触点；请重新选择并在触碰后按 Enter。")
                    continue
                touch_ids[index] = replacement
                corner_id, label = replacement, replacement_label
                break
            touched_by_id[corner_id] = np.asarray(receiver.getActualTCPPose()[:3], dtype=float)
            touch_path.write_text(yaml.safe_dump({
                "signature": signature,
                "touch_corner_ids": touch_ids,
                "touches": {int(k): v.tolist() for k, v in touched_by_id.items()},
            }, sort_keys=False), encoding="utf-8")
            print(f"{label}已保存检查点")
    finally:
        receiver.disconnect()
    selected = board_points[touch_ids]
    if np.linalg.matrix_rank(selected - selected.mean(axis=0)) < 2:
        raise SystemExit("final touch corners are collinear")
    touched = np.asarray([touched_by_id[value] for value in touch_ids])
    base_to_board, touch_rms = rigid_transform(selected, touched)
    return base_to_board, touch_rms, touch_ids


def verify_board(args, board, dictionary, camera_to_board):
    """Check each camera against its own observation before allowing board movement."""
    import pyrealsense2 as rs

    pipeline, profile = start_color_pipeline(args.camera_serial)
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    verify_intr = stream.get_intrinsics()
    verify_matrix = np.array([
        [verify_intr.fx, 0, verify_intr.ppx],
        [0, verify_intr.fy, verify_intr.ppy],
        [0, 0, 1],
    ], float)
    verify_distortion = np.asarray(verify_intr.coeffs, float)
    parameters = cv2.aruco.DetectorParameters_create()
    verification_samples: list[np.ndarray] = []
    try:
        for _ in range(300):
            frame = pipeline.wait_for_frames().get_color_frame()
            if not frame:
                continue
            image = np.asanyarray(frame.get_data())
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
            marker_corners, marker_ids = filter_inconsistent_markers(marker_corners, marker_ids, board)
            if marker_ids is None:
                continue
            count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, image, board,
                cameraMatrix=verify_matrix, distCoeffs=verify_distortion,
            )
            if int(count or 0) < 6:
                continue
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, board, verify_matrix, verify_distortion, None, None
            )
            if ok:
                pose = np.eye(4)
                pose[:3, :3] = cv2.Rodrigues(rvec)[0]
                pose[:3, 3] = np.asarray(tvec).reshape(3)
                verification_samples.append(pose)
                if len(verification_samples) >= 10:
                    break
    finally:
        pipeline.stop()
    if len(verification_samples) < 10:
        raise SystemExit("final board-motion check failed: board is occluded or insufficient corners were detected")
    verified_camera_to_board = average_transforms(verification_samples)
    board_motion_mm = float(np.linalg.norm(
        verified_camera_to_board[:3, 3] - camera_to_board[:3, 3]
    ) * 1000)
    board_motion_deg = float(rotation_error_deg(verified_camera_to_board, camera_to_board))
    return board_motion_mm, board_motion_deg


def calibration_result(args, base_to_camera, touch_ids, touch_rms, observation,
                       board_motion_mm, board_motion_deg, calibrated):
    output = dump_transform(base_to_camera, args.base_frame, args.camera_frame)
    output.update({
        "calibrated": bool(calibrated),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "method": "table-fixed ChArUco observed by camera and touched by calibrated TCP",
        "camera_serial": args.camera_serial,
        "board": {
            "squares_x": args.squares_x, "squares_y": args.squares_y,
            "square_size_m": args.square_size_m, "marker_size_m": args.marker_size_m,
            "dictionary": args.dictionary,
        },
        "touch_corner_ids": touch_ids,
        "touch_rms_mm": touch_rms * 1000,
        "quality_limits": {
            "max_touch_rms_mm": args.max_touch_rms_mm,
            "max_camera_spread_mm": args.max_camera_spread_mm,
            "max_board_motion_mm": args.max_board_motion_mm,
            "max_board_motion_deg": args.max_board_motion_deg,
        },
        "final_board_motion_mm": board_motion_mm,
        "final_board_motion_deg": board_motion_deg,
        "camera_samples": observation["camera_samples"],
        "color_fps": observation.get("color_fps"),
        "camera_translation_max_spread_mm": observation["camera_translation_max_spread_mm"],
        "camera_rotation_max_spread_deg": observation["camera_rotation_max_spread_deg"],
        "intrinsics": observation["intrinsics"],
    })
    return output


def calibrate_arm(args) -> None:
    """Observe one or two fixed cameras, touch once, then verify every camera."""
    cameras = [args]
    if args.side_camera_serial:
        side = copy(args)
        side.camera_serial = args.side_camera_serial
        side.camera_frame = args.side_camera_frame
        side.output = args.side_output
        cameras.append(side)
    if len({camera.camera_serial for camera in cameras}) != len(cameras):
        raise SystemExit("front/side camera serials must be different")
    if len({camera.output.resolve() for camera in cameras}) != len(cameras):
        raise SystemExit("front/side output paths must be different")
    for camera in cameras:
        if camera.output.exists():
            previous = yaml.safe_load(camera.output.read_text(encoding="utf-8")) or {}
            if previous.get("calibrated", False):
                raise SystemExit(f"refusing to overwrite calibrated result {camera.output}")
            print(f"将覆盖上一份未通过质量门的结果: {camera.output}")
        camera.output.parent.mkdir(parents=True, exist_ok=True)
    if args.squares_x < 3 or args.squares_y < 3:
        raise SystemExit("board must contain at least 3x3 squares")
    if not 0 < args.marker_size_m < args.square_size_m:
        raise SystemExit("marker size must be positive and smaller than square size")
    if args.camera_samples < 1:
        raise SystemExit("--camera-samples must be at least 1")
    limits = (args.max_touch_rms_mm, args.max_camera_spread_mm,
              args.max_board_motion_mm, args.max_board_motion_deg)
    if any(not np.isfinite(value) or value <= 0 for value in limits):
        raise SystemExit("quality limits must be finite and positive")
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES[args.dictionary])
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x, args.squares_y, args.square_size_m, args.marker_size_m, dictionary
    )
    board_points = _board_corners(board)
    if args.touch_corner_ids:
        ids = args.touch_corner_ids
        if len(ids) != 4 or len(set(ids)) != 4 or any(value < 0 or value >= len(board_points) for value in ids):
            raise SystemExit(f"provide exactly four unique touch IDs in [0, {len(board_points) - 1}]")
        if np.linalg.matrix_rank(board_points[ids] - board_points[ids].mean(axis=0)) < 2:
            raise SystemExit("touch corners must not be collinear")
    observations = []
    for camera in cameras:
        signature = {
            "camera_serial": camera.camera_serial,
            "squares_x": args.squares_x, "squares_y": args.squares_y,
            "square_size_m": args.square_size_m, "marker_size_m": args.marker_size_m,
            "dictionary": args.dictionary,
        }
        observations.append(observe_camera(
            camera, board, board_points, dictionary, signature,
            defer_touch_selection=len(cameras) > 1,
        ))
    common_ids = set.intersection(*(
        set(int(value) for value in observation["detected_corner_ids"])
        for observation in observations
    ))
    if args.touch_corner_ids:
        touch_ids = list(args.touch_corner_ids)
    elif len(cameras) == 1:
        touch_ids = [int(value) for value in observations[0]["touch_corner_ids"]]
    else:
        try:
            touch_ids = select_spread_corners(list(common_ids), board_points)
        except ValueError as exc:
            raise SystemExit(f"两台相机需要看见至少四个不共线的共同内角；调整板后用 --recapture-camera 重试: {exc}") from exc
    if not set(touch_ids) <= common_ids:
        raise SystemExit("selected touch points must be visible in every locked camera frame")
    for camera, observation in zip(cameras, observations):
        pixels = {int(key): value for key, value in observation.get("detected_corner_pixels", {}).items()}
        descriptions = (describe_touch_points(touch_ids, pixels, args.squares_x)
                        if pixels else observation.get("touch_point_descriptions", []))
        print(f"{camera.camera_frame} ({camera.camera_serial}) 本臂共同触点：")
        for item in descriptions:
            print(f"  P{item['id']}: 图像{item['image_position']}，"
                  f"像素(u={item['image_u_px']}, v={item['image_v_px']})，"
                  f"板坐标原点起第{item['board_row_from_origin']}行第{item['board_column_from_origin']}列")
        if len(cameras) > 1:
            observation["touch_corner_ids"] = touch_ids
            observation["touch_point_descriptions"] = descriptions
            observation_path = camera.output.with_name(camera.output.stem + "_camera_observation.yaml")
            observation_path.write_text(yaml.safe_dump(observation, sort_keys=False), encoding="utf-8")
            image_path = camera.output.with_name(camera.output.stem + "_selected_points.png")
            image = cv2.imread(str(image_path)) if image_path.exists() else None
            if image is not None:
                for item in descriptions:
                    u, v = int(round(item["image_u_px"])), int(round(item["image_v_px"]))
                    cv2.circle(image, (u, v), 16, (0, 0, 255), 4)
                    cv2.putText(image, f"P{item['id']}", (u + 10, v - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
                cv2.imwrite(str(image_path), image)
                print(f"共同触点标注图已保存: {image_path}")
    touch_signature = {
        "robot_ip": args.robot_ip, "base_frame": args.base_frame,
        "observations": [observation["signature"] for observation in observations],
        "observation_ids": [observation.get("observation_id") for observation in observations],
    }
    base_to_board, touch_rms, touch_ids = touch_board(
        args, board_points, touch_ids, common_ids, touch_signature
    )
    input("将机械臂移出两台相机的标定板视野，保持板固定，按 Enter 检查本臂标定板位移: ")
    results = []
    for camera, observation in zip(cameras, observations):
        camera_to_board = np.asarray(observation["camera_to_board"], dtype=float)
        motion_mm, motion_deg = verify_board(camera, board, dictionary, camera_to_board)
        passed = (
            touch_rms * 1000 <= args.max_touch_rms_mm
            and observation["camera_translation_max_spread_mm"] <= args.max_camera_spread_mm
            and motion_mm <= args.max_board_motion_mm
            and motion_deg <= args.max_board_motion_deg
        )
        results.append(calibration_result(
            camera, base_to_board @ np.linalg.inv(camera_to_board), touch_ids,
            touch_rms, observation, motion_mm, motion_deg, passed,
        ))
    # A failed camera invalidates this arm's pair; keep both retryable.
    all_passed = all(result["calibrated"] for result in results)
    for camera, result in zip(cameras, results):
        result["camera_quality_passed"] = result["calibrated"]
        result["calibrated"] = all_passed
        camera.output.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        print(yaml.safe_dump(result, sort_keys=False))
    if not all_passed:
        raise SystemExit("quality gate failed; retained calibrated:false results; keep board fixed for retry")
    for camera in cameras:
        print(f"PASS: wrote {camera.output}")
    print(f"{args.base_frame} 标定完成；现在允许移动标定板。相机和机器人底座须保持固定。")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--camera-serial", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--base-frame", required=True)
    parser.add_argument("--camera-frame", required=True)
    parser.add_argument("--side-camera-serial")
    parser.add_argument("--side-camera-frame", default="side_camera_color_optical_frame")
    parser.add_argument("--side-output", type=Path)
    add_board_arguments(parser)
    args = parser.parse_args(argv)
    if bool(args.side_camera_serial) != bool(args.side_output):
        parser.error("--side-camera-serial and --side-output must be provided together")
    calibrate_arm(args)


if __name__ == "__main__":
    main()
