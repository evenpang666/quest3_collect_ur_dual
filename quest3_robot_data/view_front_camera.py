"""Show RGB images from every connected RealSense camera in one OpenCV window.

The viewer opens cameras directly, independent of ROS and all robot-control
nodes. Stop VR teleoperation, RealSense ROS launches and other camera tools
first: each physical camera can have only one direct owner at a time.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass
class CameraPreview:
    serial: str
    label: str
    pipeline: Any | None
    image: np.ndarray | None = None
    error: str | None = None


def _device_label(device: Any, rs: Any) -> tuple[str, str]:
    serial = device.get_info(rs.camera_info.serial_number)
    name = device.get_info(rs.camera_info.name)
    return serial, f"{name}  [{serial}]"


def _status_tile(preview: CameraPreview, width: int, height: int) -> np.ndarray:
    if preview.image is not None:
        tile = preview.image.copy()
    else:
        tile = np.zeros((height, width, 3), dtype=np.uint8)
        message = preview.error or "waiting for RGB frames..."
        color = (40, 70, 230) if preview.error else (220, 220, 220)
        cv2.putText(tile, message[:70], (14, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    cv2.rectangle(tile, (0, 0), (width, 31), (20, 20, 20), thickness=-1)
    cv2.putText(tile, preview.label, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return tile


def _grid(previews: list[CameraPreview], width: int, height: int) -> np.ndarray:
    columns = math.ceil(math.sqrt(len(previews)))
    rows = math.ceil(len(previews) / columns)
    tiles = [_status_tile(preview, width, height) for preview in previews]
    blank = np.zeros((height, width, 3), dtype=np.uint8)
    tiles.extend(blank.copy() for _ in range(rows * columns - len(tiles)))
    return np.vstack([
        np.hstack(tiles[row * columns:(row + 1) * columns])
        for row in range(rows)
    ])


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--serial",
        action="append",
        default=[],
        help="optional serial filter; repeat to select specific cameras (default: all connected cameras)",
    )
    # 640x480 is supported by D435i/D455 and keeps a multi-camera USB preview
    # practical. This viewer is diagnostic only and never changes recording resolution.
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args(argv)
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise SystemExit("--width, --height and --fps must be positive")
    requested = {serial.strip() for serial in args.serial if serial.strip()}
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise SystemExit(
            "pyrealsense2 is required for the standalone viewer. "
            "Install librealsense Python bindings, then retry."
        ) from exc

    context = rs.context()
    devices = list(context.query_devices())
    if requested:
        devices = [device for device in devices if _device_label(device, rs)[0] in requested]
        missing = requested - {_device_label(device, rs)[0] for device in devices}
        if missing:
            raise SystemExit(f"requested RealSense serial(s) not connected: {', '.join(sorted(missing))}")
    if not devices:
        raise SystemExit("no RealSense cameras are connected")

    previews: list[CameraPreview] = []
    for device in devices:
        serial, label = _device_label(device, rs)
        pipeline = rs.pipeline(context)
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
        try:
            profile = pipeline.start(config)
            color = profile.get_stream(rs.stream.color).as_video_stream_profile()
            print(f"RGB ready: {label}, {color.width()}x{color.height()}@{color.fps()} BGR8", flush=True)
            previews.append(CameraPreview(serial, label, pipeline))
        except RuntimeError as exc:
            # Keep a visible tile for every detected device, including one
            # owned by another process or connected through a bad USB link.
            previews.append(CameraPreview(serial, label, None, error=f"unavailable: {exc}"))
            print(f"RGB unavailable: {label}: {exc}", flush=True)

    if not any(preview.pipeline is not None for preview in previews):
        raise SystemExit(
            "no connected RealSense RGB stream could be opened. Stop camera-owning processes first."
        )

    window = "RealSense RGB preview (q / Esc to exit)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print("Press q or Esc in the image window to exit.", flush=True)
    try:
        while True:
            for preview in previews:
                if preview.pipeline is None:
                    continue
                try:
                    frames = preview.pipeline.poll_for_frames()
                    if not frames:
                        continue
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        preview.image = np.asanyarray(color_frame.get_data())
                except RuntimeError as exc:
                    preview.error = f"stream lost: {exc}"
                    try:
                        preview.pipeline.stop()
                    except RuntimeError:
                        pass
                    preview.pipeline = None
            cv2.imshow(window, _grid(previews, args.width, args.height))
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        cv2.destroyAllWindows()
        for preview in previews:
            if preview.pipeline is not None:
                try:
                    preview.pipeline.stop()
                except RuntimeError:
                    pass


if __name__ == "__main__":
    main()
