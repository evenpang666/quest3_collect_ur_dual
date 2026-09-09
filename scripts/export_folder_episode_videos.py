#!/usr/bin/env python3
"""Export a folder_npy episode to RGB and depth review MP4 videos."""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


RGB_CAMERAS = (
    ("rgb_front_cam", "front_cam.mp4"),
    ("rgb_side_cam", "side_cam.mp4"),
    ("rgb_wrist_left_cam", "wrist_left_cam.mp4"),
    ("rgb_wrist_right_cam", "wrist_right_cam.mp4"),
)

DEPTH_CAMERAS = (
    ("depth_front_cam", "front_depth.mp4"),
    ("depth_side_cam", "side_depth.mp4"),
    ("depth_wrist_left_cam", "wrist_left_depth.mp4"),
    ("depth_wrist_right_cam", "wrist_right_depth.mp4"),
)


@dataclass(frozen=True)
class VideoSpec:
    directory: str
    filename: str
    width: int
    height: int


def frame_count(episode: Path) -> int:
    state_path = episode / "robot_state.jsonl"
    if state_path.is_file():
        return sum(1 for line in state_path.read_text(encoding="utf-8").splitlines() if line.strip())
    highest = -1
    for directory, _ in RGB_CAMERAS + DEPTH_CAMERAS:
        for path in (episode / directory).glob("*"):
            try:
                highest = max(highest, int(path.stem))
            except ValueError:
                pass
    return highest + 1


def load_rgb(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".jpg":
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"cannot decode {path}")
        return frame
    value = np.load(path, allow_pickle=False)
    if value.ndim != 3 or value.shape[2] != 3:
        raise ValueError(f"unsupported RGB NPY shape {value.shape}: {path}")
    return np.ascontiguousarray(value[..., ::-1])  # saved NPY is RGB; OpenCV is BGR.


def rgb_frame_path(episode: Path, directory: str, index: int) -> Path | None:
    base = episode / directory / f"{index:06d}"
    for suffix in (".jpg", ".npy"):
        candidate = base.with_suffix(suffix)
        if candidate.is_file():
            return candidate
    return None


def depth_frame_path(episode: Path, directory: str, index: int) -> Path | None:
    candidate = episode / directory / f"{index:06d}.npy"
    return candidate if candidate.is_file() else None


def rgb_dimensions(episode: Path, directory: str) -> tuple[int, int] | None:
    folder = episode / directory
    for path in sorted((*folder.glob("*.jpg"), *folder.glob("*.npy"))):
        frame = load_rgb(path)
        return frame.shape[1], frame.shape[0]
    return None


def depth_dimensions(episode: Path, directory: str) -> tuple[int, int] | None:
    for path in sorted((episode / directory).glob("*.npy")):
        value = np.load(path, allow_pickle=False)
        if value.ndim == 3 and value.shape[2] == 1:
            value = value[..., 0]
        if value.ndim != 2:
            raise ValueError(f"unsupported depth NPY shape {value.shape}: {path}")
        return value.shape[1], value.shape[0]
    return None


def colorize_depth(path: Path, min_mm: int, max_mm: int) -> np.ndarray:
    depth = np.load(path, allow_pickle=False)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[..., 0]
    if depth.ndim != 2:
        raise ValueError(f"unsupported depth NPY shape {depth.shape}: {path}")
    # D4xx ROS depth images are uint16 millimetres. Keep invalid (zero) pixels black.
    scaled = np.clip((depth.astype(np.float32) - min_mm) * 255.0 / (max_mm - min_mm), 0, 255).astype(np.uint8)
    frame = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    frame[depth == 0] = 0
    return frame


def export(
    episode: Path,
    output_dir: Path,
    fps: float,
    codec: str,
    overwrite: bool,
    depth_min_mm: int,
    depth_max_mm: int,
) -> list[Path]:
    episode, output_dir = episode.expanduser().resolve(), output_dir.expanduser().resolve()
    if not episode.is_dir():
        raise FileNotFoundError(f"folder episode not found: {episode}")
    if fps <= 0 or len(codec) != 4:
        raise ValueError("--fps must be positive and --codec must have four characters")
    if depth_min_mm < 0 or depth_max_mm <= depth_min_mm:
        raise ValueError("--depth-max-mm must be greater than --depth-min-mm >= 0")
    frames = frame_count(episode)
    if frames <= 0:
        raise ValueError(f"no frame records found in {episode}")
    sizes = {directory: rgb_dimensions(episode, directory) for directory, _ in RGB_CAMERAS}
    sizes.update({directory: depth_dimensions(episode, directory) for directory, _ in DEPTH_CAMERAS})
    reference = next((size for size in sizes.values() if size), (640, 480))
    specs = [
        VideoSpec(directory, filename, *(sizes[directory] or reference))
        for directory, filename in RGB_CAMERAS + DEPTH_CAMERAS
    ]
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = [output_dir / spec.filename for spec in specs]
    if not overwrite and any(path.exists() for path in paths):
        raise FileExistsError("video exists; add --overwrite")
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writers = [cv2.VideoWriter(str(path), fourcc, fps, (spec.width, spec.height)) for spec, path in zip(specs, paths)]
    if not all(writer.isOpened() for writer in writers):
        for writer in writers:
            writer.release()
        raise RuntimeError(f"cannot open MP4 writer using {codec}")
    try:
        for index in range(frames):
            for spec, writer in zip(specs, writers):
                is_depth = spec.directory.startswith("depth_")
                source = (depth_frame_path if is_depth else rgb_frame_path)(episode, spec.directory, index)
                if source is None:
                    frame = np.zeros((spec.height, spec.width, 3), np.uint8)
                elif is_depth:
                    frame = colorize_depth(source, depth_min_mm, depth_max_mm)
                else:
                    frame = load_rgb(source)
                if frame.shape[:2] != (spec.height, spec.width):
                    frame = cv2.resize(frame, (spec.width, spec.height), interpolation=cv2.INTER_AREA)
                writer.write(frame)
    finally:
        for writer in writers:
            writer.release()
    print(f"Exported {frames} synchronized samples at {fps:g} FPS to {output_dir}")
    for spec, path in zip(specs, paths):
        state = "black (missing directory)" if sizes[spec.directory] is None else f"{spec.width}x{spec.height}"
        print(f"  {path.name}: {state}")
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path, help="folder episode_00001")
    parser.add_argument("--output-dir", type=Path, help="default: <episode>_videos beside it")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--codec", default="mp4v")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--depth-min-mm", type=int, default=200, help="depth visualization lower limit (default: 200)")
    parser.add_argument("--depth-max-mm", type=int, default=2000, help="depth visualization upper limit (default: 2000)")
    args = parser.parse_args()
    output = args.output_dir or args.episode.with_name(f"{args.episode.name}_videos")
    export(args.episode, output, args.fps, args.codec, args.overwrite, args.depth_min_mm, args.depth_max_mm)


if __name__ == "__main__":
    main()
