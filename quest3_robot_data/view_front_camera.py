"""Show the configured front RealSense color stream in an OpenCV window.

This utility opens the camera directly; it is intentionally independent of ROS
and of all robot-control nodes.  Do not run it while a RealSense ROS launch or
the VR teleoperation script is already using the same D455.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def _front_serial(path: Path) -> str:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        serial = str(data["cameras"]["front"]["serial"]).strip()
    except (KeyError, TypeError, OSError, yaml.YAMLError) as exc:
        raise SystemExit(f"cannot read front camera serial from {path}: {exc}") from exc
    if not serial:
        raise SystemExit(f"front camera serial is empty in {path}")
    return serial


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware-config",
        required=True,
        type=Path,
        help="hardware YAML containing cameras.front.serial",
    )
    parser.add_argument("--serial", help="override the front D455 serial from the YAML")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args(argv)
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise SystemExit("--width, --height and --fps must be positive")

    serial = args.serial.strip() if args.serial else _front_serial(args.hardware_config.expanduser().resolve())
    if not serial:
        raise SystemExit("--serial must not be empty")
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise SystemExit(
            "pyrealsense2 is required for the standalone viewer. "
            "Install librealsense Python bindings, then retry."
        ) from exc

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    started = False
    try:
        profile = pipeline.start(config)
        started = True
        color = profile.get_stream(rs.stream.color).as_video_stream_profile()
        print(
            f"front_cam ready: serial={serial}, "
            f"{color.width()}x{color.height()}@{color.fps()} BGR8\n"
            "Press q or Esc in the image window to exit.",
            flush=True,
        )
        window = "front_cam (q / Esc to exit)"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            frame = frames.get_color_frame()
            if not frame:
                continue
            cv2.imshow(window, np.asanyarray(frame.get_data()))
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    except RuntimeError as exc:
        raise SystemExit(
            f"cannot start front_cam {serial}: {exc}\n"
            "If it is busy, stop the VR teleoperation/RealSense launch first; "
            "only one direct owner may open the camera."
        ) from exc
    finally:
        cv2.destroyAllWindows()
        if started:
            pipeline.stop()


if __name__ == "__main__":
    main()
