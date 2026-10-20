#!/usr/bin/env python3
"""Export every HDF5 episode in one task directory to four MP4 videos."""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import h5py

if __package__:
    from scripts.export_episode_videos import CAMERAS, DEFAULT_CODEC, episode_frame_count, export
else:
    from export_episode_videos import CAMERAS, DEFAULT_CODEC, episode_frame_count, export


def videos_complete(episode: Path, paths: list[Path], codec: str = DEFAULT_CODEC) -> bool:
    if not all(path.is_file() and path.stat().st_size > 0 for path in paths):
        return False
    with h5py.File(episode, "r") as file:
        expected_frames = episode_frame_count(file)
    if expected_frames <= 0:
        return False
    for path in paths:
        capture = cv2.VideoCapture(str(path))
        try:
            if not capture.isOpened() or int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) != expected_frames:
                return False
            fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
            actual_codec = "".join(chr((fourcc >> (8 * index)) & 255) for index in range(4)).lower()
            h264_tags = {"avc1", "h264", "x264"}
            if codec.lower() in h264_tags:
                if actual_codec not in h264_tags:
                    return False
            elif actual_codec != codec.lower():
                return False
        finally:
            capture.release()
    return True


def export_task(task_dir: Path, fps: float | None = None, codec: str = DEFAULT_CODEC, overwrite: bool = False) -> tuple[int, int, int]:
    task_dir = task_dir.expanduser().resolve()
    if not task_dir.is_dir():
        raise NotADirectoryError(f"task directory does not exist: {task_dir}")
    if fps is not None and fps <= 0:
        raise ValueError("--fps must be positive")
    if len(codec) != 4:
        raise ValueError("--codec must be exactly four characters, e.g. mp4v")

    episodes = sorted(path for path in task_dir.glob("*.hdf5") if path.is_file())
    if not episodes:
        raise FileNotFoundError(f"no .hdf5 files in task directory: {task_dir}")

    succeeded = skipped = failed = 0
    print(f"Found {len(episodes)} HDF5 episodes in {task_dir}", flush=True)
    for episode in episodes:
        output_dir = episode.with_name(f"{episode.stem}_videos")
        expected = [output_dir / filename for _, filename in CAMERAS]
        try:
            if not overwrite and videos_complete(episode, expected, codec):
                print(f"[skip] {episode.name}: all four videos have the expected codec and frame count", flush=True)
                skipped += 1
                continue
            print(f"[export] {episode.name}", flush=True)
            # A previous run may have left incomplete MP4 files; replace those.
            export(episode, output_dir, fps, codec, overwrite or any(path.exists() for path in expected))
        except Exception as exc:
            print(f"[failed] {episode.name}: {exc}", flush=True)
            failed += 1
        else:
            succeeded += 1

    print(f"Done: {succeeded} exported, {skipped} skipped, {failed} failed", flush=True)
    return succeeded, skipped, failed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task_dir", type=Path, help="directory containing this task's .hdf5 episodes")
    parser.add_argument("--fps", type=float, help="video FPS; default: 30")
    parser.add_argument("--codec", default=DEFAULT_CODEC, help="fourcc codec; default: avc1 (H.264, VS Code compatible)")
    parser.add_argument("--overwrite", action="store_true", help="replace existing videos")
    args = parser.parse_args()
    try:
        _, _, failed = export_task(args.task_dir, args.fps, args.codec, args.overwrite)
    except (FileNotFoundError, NotADirectoryError, ValueError) as exc:
        parser.error(str(exc))
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
