"""Calibrate a fixed camera to a UR base from a table-fixed ChArUco board.

The calibrated TCP touches known ChArUco corners to establish base_T_board.
The camera observes the same stationary board to establish camera_T_board.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path

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


def describe_touch_points(
    touch_ids: list[int], pixel_by_id: dict[int, list[float]], squares_x: int
) -> list[dict[str, object]]:
    """Build human-readable board-grid and image locations for selected points."""
    inner_x = squares_x - 1
    center_u = float(np.mean([pixel_by_id[value][0] for value in touch_ids]))
    center_v = float(np.mean([pixel_by_id[value][1] for value in touch_ids]))
    result = []
    for corner_id in touch_ids:
        u, v = (float(value) for value in pixel_by_id[corner_id])
        result.append({
            "id": corner_id,
            "board_row_from_origin": corner_id // inner_x + 1,
            "board_column_from_origin": corner_id % inner_x + 1,
            "image_u_px": round(u, 1),
            "image_v_px": round(v, 1),
            "image_position": ("左" if u < center_u else "右") + ("上" if v < center_v else "下"),
        })
    return result


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--camera-serial", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--base-frame", required=True)
    parser.add_argument("--camera-frame", required=True)
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
    args = parser.parse_args(argv)
    if args.output.exists():
        previous = yaml.safe_load(args.output.read_text(encoding="utf-8"))
        if previous.get("calibrated", False):
            raise SystemExit(f"refusing to overwrite calibrated result {args.output}")
        print(f"将覆盖上一份未通过质量门的结果: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.squares_x < 3 or args.squares_y < 3:
        raise SystemExit("board must contain at least 3x3 squares")
    if not 0 < args.marker_size_m < args.square_size_m:
        raise SystemExit("marker size must be positive and smaller than square size")
    if args.camera_samples < 1:
        raise SystemExit("--camera-samples must be at least 1")

    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES[args.dictionary])
    signature = {
        "camera_serial": args.camera_serial,
        "squares_x": args.squares_x,
        "squares_y": args.squares_y,
        "square_size_m": args.square_size_m,
        "marker_size_m": args.marker_size_m,
        "dictionary": args.dictionary,
    }
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x, args.squares_y, args.square_size_m, args.marker_size_m, dictionary
    )
    board_points = _board_corners(board)
    touch_ids = list(args.touch_corner_ids) if args.touch_corner_ids else None
    if touch_ids is not None:
        if len(set(touch_ids)) != 4 or any(value < 0 or value >= len(board_points) for value in touch_ids):
            raise SystemExit(f"provide exactly four unique touch IDs in [0, {len(board_points) - 1}]")
        if np.linalg.matrix_rank(board_points[touch_ids] - board_points[touch_ids].mean(axis=0)) < 2:
            raise SystemExit("touch corners must not be collinear")

    import pyrealsense2 as rs
    from rtde_receive import RTDEReceiveInterface

    observation_path = args.output.with_name(args.output.stem + "_camera_observation.yaml")
    if observation_path.exists() and not args.recapture_camera:
        observation = yaml.safe_load(observation_path.read_text(encoding="utf-8"))
        if observation.get("signature") != signature:
            raise SystemExit(f"saved camera observation does not match current board/camera: {observation_path}")
        camera_to_board = np.asarray(observation["camera_to_board"], dtype=float)
        camera_spread = float(observation["camera_translation_max_spread_mm"])
        rotation_spread = float(observation["camera_rotation_max_spread_deg"])
        intrinsics = observation["intrinsics"]
        camera_sample_count = int(observation.get("camera_samples", args.camera_samples))
        if touch_ids is None:
            if "touch_corner_ids" not in observation:
                raise SystemExit("saved observation has no auto-selected touch points; rerun with --recapture-camera")
            touch_ids = [int(value) for value in observation["touch_corner_ids"]]
        detected_touch_candidates = {
            int(value) for value in observation.get("detected_corner_ids", touch_ids)
        }
        point_descriptions = observation.get("touch_point_descriptions", [])
        print(f"复用已保存的相机/标定板观测: {observation_path}")
    else:
        pipeline, cfg = rs.pipeline(), rs.config()
        cfg.enable_device(args.camera_serial)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        print("正在独占打开 front_cam；若提示 device busy，请先停止 ROS RealSense 节点。")
        profile = pipeline.start(cfg)
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
                cv2.imshow("front ChArUco calibration", image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    raise SystemExit("calibration cancelled")
                if key == 32 and pose is not None:
                    if touch_ids is None:
                        touch_ids = select_spread_corners(list(pixel_by_id), board_points)
                    detected_touch_candidates = set(pixel_by_id)
                    missing = [value for value in touch_ids if value not in pixel_by_id]
                    if missing:
                        print(f"当前帧未识别指定角点 {missing}，请等待后重新按空格")
                        continue
                    point_descriptions = describe_touch_points(touch_ids, pixel_by_id, args.squares_x)
                    locked_image = image.copy()
                    for item in point_descriptions:
                        u, v = int(round(item["image_u_px"])), int(round(item["image_v_px"]))
                        cv2.circle(locked_image, (u, v), 16, (0, 0, 255), 4)
                        cv2.putText(locked_image, f"P{item['id']}", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
                    capturing = True
                    print("已锁定标定板并自动选择四个相距最远的已识别内角：")
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
            "camera_to_board": camera_to_board.tolist(),
            "camera_samples": camera_sample_count,
            "touch_corner_ids": touch_ids,
            "detected_corner_ids": sorted(detected_touch_candidates),
            "touch_point_descriptions": point_descriptions,
            "camera_translation_max_spread_mm": camera_spread,
            "camera_rotation_max_spread_deg": rotation_spread,
            "intrinsics": intrinsics,
        }
        observation_path.write_text(yaml.safe_dump(observation, sort_keys=False), encoding="utf-8")
        print(f"相机观测检查点已保存: {observation_path}")

    assert touch_ids is not None
    if any(value not in detected_touch_candidates for value in touch_ids):
        raise SystemExit("selected touch point is not present in the locked camera frame")
    print("本次推荐触点：")
    for item in point_descriptions:
        print(
            f"  P{item['id']}: 图像{item['image_position']}，"
            f"像素(u={item['image_u_px']}, v={item['image_v_px']})，"
            f"板坐标原点起第{item['board_row_from_origin']}行"
            f"第{item['board_column_from_origin']}列"
        )

    receiver = RTDEReceiveInterface(args.robot_ip)
    touch_path = args.output.with_name(args.output.stem + "_touches.yaml")
    touch_data = yaml.safe_load(touch_path.read_text(encoding="utf-8")) if touch_path.exists() else {}
    if touch_data and touch_data.get("signature") != signature:
        raise SystemExit(f"saved touch checkpoint does not match current board/camera: {touch_path}")
    touched_by_id = (
        {} if args.recapture_camera else
        {int(key): np.asarray(value, dtype=float) for key, value in touch_data.get("touches", {}).items()}
    )
    for corner_id in args.retouch_corner_ids:
        touched_by_id.pop(corner_id, None)
    print("确认示教器已激活刚标定的 TCP。依次将 TCP 尖端精确触碰指定 ChArUco 内角点。")
    try:
        for index in range(len(touch_ids)):
            corner_id = touch_ids[index]
            if corner_id in touched_by_id:
                print(f"[{index + 1}/{len(touch_ids)}] 复用 corner ID {corner_id} 检查点")
                continue
            answer = input(
                f"[{index + 1}/{len(touch_ids)}] 触碰 corner ID {corner_id} 后按 Enter；"
                "输入 id=N 可替换为可达内角，q 取消: "
            ).strip().lower()
            if answer == "q":
                raise SystemExit("calibration cancelled")
            if answer.startswith("id="):
                replacement = int(answer.split("=", 1)[1])
                if replacement not in detected_touch_candidates or replacement in touch_ids:
                    raise SystemExit(f"invalid replacement corner ID {replacement}")
                touch_ids[index] = replacement
                corner_id = replacement
                input(f"改为 corner ID {corner_id}；触碰后按 Enter: ")
            touched_by_id[corner_id] = np.asarray(receiver.getActualTCPPose()[:3], dtype=float)
            touch_path.write_text(yaml.safe_dump({
                "signature": signature,
                "touches": {int(k): v.tolist() for k, v in touched_by_id.items()},
            }, sort_keys=False), encoding="utf-8")
            print(f"corner ID {corner_id} 已保存检查点")
    finally:
        receiver.disconnect()
    selected = board_points[touch_ids]
    if np.linalg.matrix_rank(selected - selected.mean(axis=0)) < 2:
        raise SystemExit("final touch corners are collinear")
    touched = np.asarray([touched_by_id[value] for value in touch_ids])
    base_to_board, touch_rms = rigid_transform(selected, touched)
    base_to_camera = base_to_board @ np.linalg.inv(camera_to_board)
    input("将机械臂移出标定板视野，确认标定板仍固定在原位，然后按 Enter 做最终防移动检查: ")
    pipeline, cfg = rs.pipeline(), rs.config()
    cfg.enable_device(args.camera_serial)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    profile = pipeline.start(cfg)
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
    calibrated = (
        touch_rms * 1000 <= args.max_touch_rms_mm
        and camera_spread <= args.max_camera_spread_mm
        and board_motion_mm <= args.max_board_motion_mm
        and board_motion_deg <= args.max_board_motion_deg
    )
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
        "camera_samples": camera_sample_count,
        "camera_translation_max_spread_mm": camera_spread,
        "camera_rotation_max_spread_deg": rotation_spread,
        "intrinsics": intrinsics,
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(output, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(output, sort_keys=False))
    if not calibrated:
        raise SystemExit(f"quality gate failed; retained calibrated:false file: {args.output}")
    print(f"PASS: wrote {args.output}")


if __name__ == "__main__":
    main()
