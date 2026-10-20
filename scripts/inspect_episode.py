#!/usr/bin/env python3
"""Print an episode's layout and simple timestamp diagnostics."""
import argparse
from pathlib import Path

import h5py
import numpy as np


def timestamp_diagnostics(stamps, expected_fps=30.0):
    """Estimate missing reference frames, separately from observed recorder drops."""
    if not np.isfinite(expected_fps) or expected_fps <= 0:
        raise ValueError("expected FPS must be finite and positive")
    stamps = np.asarray(stamps, dtype=np.int64)
    delta = np.diff(stamps)
    period = 1e9 / expected_fps
    duration = (int(stamps[-1]) - int(stamps[0])) / 1e9 if len(stamps) > 1 else 0.0
    return {
        "fps": (len(stamps) - 1) / duration if duration > 0 else 0.0,
        "long_gaps": int(np.count_nonzero(delta > 1.5 * period)),
        "estimated_missing": int(np.maximum(np.rint(delta / period).astype(np.int64) - 1, 0).sum()),
        "non_monotonic": int(np.count_nonzero(delta <= 0)),
    }


def visit(name, obj):
    if isinstance(obj, h5py.Dataset):
        storage = obj.attrs.get("storage")
        suffix = f", storage={storage}" if storage else ""
        print(f"{name}: shape={obj.shape}, dtype={obj.dtype}{suffix}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--expected-fps", type=float, default=30.0,
                        help="reference camera rate for gap estimates (default: 30)")
    args = parser.parse_args()
    if not np.isfinite(args.expected_fps) or args.expected_fps <= 0:
        parser.error("--expected-fps must be finite and positive")
    with h5py.File(args.episode, "r") as file:
        print(f"file: {args.episode}")
        for key, value in file.attrs.items():
            print(f"attr/{key}: {value}")
        file.visititems(visit)
        stamps = file["timestamps_ns"][:]
        diagnostics = timestamp_diagnostics(stamps, args.expected_fps)
        print(f"sample rate: {diagnostics['fps']:.2f} FPS (expected {args.expected_fps:g})")
        print(f"long frame gaps (>1.5 periods): {diagnostics['long_gaps']}; "
              f"estimated missing reference frames: {diagnostics['estimated_missing']}; "
              f"non-monotonic timestamps: {diagnostics['non_monotonic']}")
        if "action_valid" in file:
            print(f"valid actions: {int(np.count_nonzero(file['action_valid'][:]))}/{len(stamps)}")
        if len(stamps) > 1:
            delta_ms = np.diff(stamps.astype(np.int64)) / 1e6
            print("frame interval (ms): "
                  f"min={delta_ms.min():.2f}, median={np.median(delta_ms):.2f}, max={delta_ms.max():.2f}")
        source_group = file.get("source_timestamps_ns")
        if source_group and len(stamps):
            max_skew_ms = max(
                float(np.max(np.abs(source_group[name][:].astype(np.int64) - stamps.astype(np.int64))) / 1e6)
                for name in source_group
            )
            print(f"maximum source skew: {max_skew_ms:.2f} ms")


if __name__ == "__main__":
    main()
