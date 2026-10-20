#!/usr/bin/env python3
"""Export one recorded HDF5 episode as four RGB MP4 videos.

The recorder names cameras ``front``, ``side``, ``left_wrist`` and
``right_wrist``.  The exported filenames deliberately use stable names for
downstream review: front_cam.mp4, side_cam.mp4, wrist_left_cam.mp4 and
wrist_right_cam.mp4.  Missing camera datasets become all-black videos with
the same number of frames as the episode.
The default H.264 (avc1) encoding supports VS Code's built-in video preview.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import h5py
import numpy as np


DEFAULT_CODEC = "avc1"


CAMERAS = (
    ("front", "front_cam.mp4"),
    ("side", "side_cam.mp4"),
    ("left_wrist", "wrist_left_cam.mp4"),
    ("right_wrist", "wrist_right_cam.mp4"),
)


@dataclass(frozen=True)
class VideoSpec:
    camera: str
    filename: str
    dataset: h5py.Dataset | None
    width: int
    height: int


def episode_frame_count(file: h5py.File) -> int:
    if "timestamps_ns" in file:
        return int(file["timestamps_ns"].shape[0])
    images = file.get("observations/images")
    if images is None:
        return 0
    lengths = [int(group["rgb"].shape[0]) for group in images.values() if "rgb" in group]
    return max(lengths, default=0)


def output_fps(requested: float | None) -> float:
    """Use a constant review rate; one HDF5 sample is exactly one video frame."""
    if requested is not None:
        if requested <= 0:
            raise ValueError("--fps must be positive")
        return requested
    return 30.0


def rgb_dataset(file: h5py.File, camera: str) -> h5py.Dataset | None:
    node = file.get(f"observations/images/{camera}/rgb")
    return node if isinstance(node, h5py.Dataset) else None


def dimensions(dataset: h5py.Dataset | None) -> tuple[int, int] | None:
    if dataset is None or len(dataset.shape) < 3:
        return None
    # Raw RGB dataset layout is N,H,W,3.  JPEG datasets retain original_shape.
    shape = dataset.attrs.get("original_shape", dataset.shape[1:])
    if len(shape) < 2:
        return None
    return int(shape[1]), int(shape[0])


def decode_frame(dataset: h5py.Dataset, index: int) -> np.ndarray:
    value = dataset[index]
    if dataset.attrs.get("storage", "raw_lzf") == "jpeg":
        frame = cv2.imdecode(np.asarray(value, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"cannot decode JPEG frame {index}")
        return frame  # OpenCV decode already returns BGR for VideoWriter.
    frame = np.asarray(value)
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(f"unsupported raw RGB frame shape {frame.shape}")
    return np.ascontiguousarray(frame[..., ::-1])  # RGB HDF5 -> BGR OpenCV.


def export(episode: Path, output_dir: Path, fps_override: float | None, codec: str, overwrite: bool) -> list[Path]:
    if len(codec) != 4:
        raise ValueError("--codec must be exactly four characters, e.g. mp4v")
    episode = episode.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    if not episode.is_file():
        raise FileNotFoundError(episode)
    output_dir.mkdir(parents=True, exist_ok=True)

    with h5py.File(episode, "r") as file:
        frames = episode_frame_count(file)
        if frames <= 0:
            raise ValueError(f"{episode} contains no recorded frames")
        fps = output_fps(fps_override)
        datasets = {camera: rgb_dataset(file, camera) for camera, _ in CAMERAS}
        reference = next((dimensions(dataset) for dataset in datasets.values() if dimensions(dataset)), (640, 480))
        specs = [
            VideoSpec(camera, filename, datasets[camera], *(dimensions(datasets[camera]) or reference))
            for camera, filename in CAMERAS
        ]
        paths = [output_dir / spec.filename for spec in specs]
        existing = [path for path in paths if path.exists()]
        if existing and not overwrite:
            raise FileExistsError(f"output already exists: {existing[0]}; use --overwrite")

        fourcc = cv2.VideoWriter_fourcc(*codec)
        writers = [cv2.VideoWriter(str(path), fourcc, fps, (spec.width, spec.height)) for spec, path in zip(specs, paths)]
        if not all(writer.isOpened() for writer in writers):
            for writer in writers:
                writer.release()
            raise RuntimeError(
                f"cannot open MP4 writer with codec {codec!r}; H.264 export requires an encoder-enabled "
                "OpenCV build (on Ubuntu use /usr/bin/python3 with python3-opencv)"
            )
        try:
            for index in range(frames):
                for spec, writer in zip(specs, writers):
                    if spec.dataset is None or index >= spec.dataset.shape[0]:
                        frame = np.zeros((spec.height, spec.width, 3), dtype=np.uint8)
                    else:
                        frame = decode_frame(spec.dataset, index)
                        if frame.shape[:2] != (spec.height, spec.width):
                            frame = cv2.resize(frame, (spec.width, spec.height), interpolation=cv2.INTER_AREA)
                    writer.write(frame)
        finally:
            for writer in writers:
                writer.release()

    print(f"Exported {frames} synchronized samples at {fps:g} FPS to {output_dir}")
    for spec, path in zip(specs, paths):
        state = "black (missing RGB dataset)" if spec.dataset is None else f"{spec.width}x{spec.height}"
        print(f"  {path.name}: {state}")
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path, help="episode_XXXXX.hdf5")
    parser.add_argument("--output-dir", type=Path, help="default: <episode_stem>_videos beside episode")
    parser.add_argument("--fps", type=float, help="video FPS; default: 30")
    parser.add_argument("--codec", default=DEFAULT_CODEC, help="fourcc codec; default: avc1 (H.264, VS Code compatible)")
    parser.add_argument("--overwrite", action="store_true", help="replace existing MP4 files")
    args = parser.parse_args()
    output = args.output_dir or args.episode.with_name(f"{args.episode.stem}_videos")
    export(args.episode, output, args.fps, args.codec, args.overwrite)


if __name__ == "__main__":
    main()
