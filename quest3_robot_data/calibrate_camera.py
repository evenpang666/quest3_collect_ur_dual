"""Calibrate a fixed D435i/D455 optical frame to one UR base using an ArUco tag."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml

from .transforms import dump_transform, load_transform, matrix_to_quaternion, ur_pose_to_matrix


def average_transforms(values: list[np.ndarray]) -> np.ndarray:
    translations = np.array([value[:3, 3] for value in values])
    quaternions = np.array([matrix_to_quaternion(value[:3, :3]) for value in values])
    reference = quaternions[0]
    quaternions = np.array([q if np.dot(q, reference) >= 0 else -q for q in quaternions])
    eigenvalues, eigenvectors = np.linalg.eigh(quaternions.T @ quaternions)
    q = eigenvectors[:, int(np.argmax(eigenvalues))]
    if np.dot(q, reference) < 0:
        q = -q
    from .transforms import transform
    return transform(np.median(translations, axis=0), q)


def rotation_error_deg(lhs: np.ndarray, rhs: np.ndarray) -> float:
    cosine = np.clip((np.trace(lhs[:3, :3] @ rhs[:3, :3].T) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(cosine)))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--camera-serial", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--base-frame", required=True)
    parser.add_argument("--camera-frame", required=True)
    parser.add_argument("--tcp-to-tag", required=True, type=Path, help="YAML transform from calibrated TCP to tag")
    parser.add_argument("--marker-id", type=int, required=True)
    parser.add_argument("--marker-size-m", type=float, required=True)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--max-translation-error-mm", type=float, default=10.0)
    parser.add_argument("--max-rotation-error-deg", type=float, default=3.0)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite {args.output}")
    fixture = yaml.safe_load(args.tcp_to_tag.read_text(encoding="utf-8"))
    tcp_to_tag = load_transform(fixture)
    import pyrealsense2 as rs
    from rtde_receive import RTDEReceiveInterface

    receiver = RTDEReceiveInterface(args.robot_ip)
    pipeline, cfg = rs.pipeline(), rs.config()
    cfg.enable_device(args.camera_serial)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    profile = pipeline.start(cfg)
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = stream.get_intrinsics()
    camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], float)
    distortion = np.asarray(intr.coeffs, float)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        detect = detector.detectMarkers
    else:  # Ubuntu 22.04 / OpenCV 4.5 compatibility.
        parameters = cv2.aruco.DetectorParameters_create()

        def detect(image):
            return cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    half = args.marker_size_m / 2
    object_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)
    estimates: list[np.ndarray] = []
    print("移动机械臂让标记以不同位置/姿态覆盖画面。检测稳定时按空格采样，q 退出。")
    try:
        while len(estimates) < args.samples:
            frame = pipeline.wait_for_frames().get_color_frame()
            if not frame:
                continue
            image = np.asanyarray(frame.get_data())
            corners, ids, _ = detect(image)
            selected = None
            if ids is not None:
                matches = np.where(ids.reshape(-1) == args.marker_id)[0]
                if len(matches):
                    selected = corners[int(matches[0])].reshape(4, 2)
                    cv2.polylines(image, [selected.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.putText(image, f"samples {len(estimates)}/{args.samples}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("camera calibration", image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                raise SystemExit("calibration cancelled")
            if key == 32 and selected is not None:
                ok, rvec, tvec = cv2.solvePnP(object_points, selected, camera_matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                camera_to_tag = np.eye(4)
                camera_to_tag[:3, :3] = cv2.Rodrigues(rvec)[0]
                camera_to_tag[:3, 3] = tvec.reshape(3)
                base_to_tcp = ur_pose_to_matrix(receiver.getActualTCPPose())
                estimates.append(base_to_tcp @ tcp_to_tag @ np.linalg.inv(camera_to_tag))
                print(f"captured {len(estimates)}/{args.samples}")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    initial = average_transforms(estimates)
    good = [value for value in estimates if np.linalg.norm(value[:3, 3] - initial[:3, 3]) * 1000 <= args.max_translation_error_mm and rotation_error_deg(value, initial) <= args.max_rotation_error_deg]
    if len(good) < max(6, len(estimates) // 2):
        calibrated, result = False, initial
    else:
        calibrated, result = True, average_transforms(good)
    translation_errors = [np.linalg.norm(v[:3, 3] - result[:3, 3]) * 1000 for v in good]
    rotation_errors = [rotation_error_deg(v, result) for v in good]
    output = dump_transform(result, args.base_frame, args.camera_frame)
    output.update({
        "calibrated": calibrated,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "method": "fixed camera, ArUco tag rigidly attached to calibrated TCP",
        "camera_serial": args.camera_serial,
        "marker_id": args.marker_id,
        "marker_size_m": args.marker_size_m,
        "samples_total": len(estimates),
        "samples_used": len(good),
        "translation_rms_mm": float(np.sqrt(np.mean(np.square(translation_errors)))) if good else None,
        "rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotation_errors)))) if good else None,
        "intrinsics": {"width": intr.width, "height": intr.height, "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy, "distortion": intr.coeffs},
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(output, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(output, sort_keys=False))
    if not calibrated:
        raise SystemExit(f"calibration quality failed; retained calibrated:false file: {args.output}")


if __name__ == "__main__":
    main()
