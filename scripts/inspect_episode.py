#!/usr/bin/env python3
"""Print an episode's layout and simple timestamp diagnostics."""
import argparse
from pathlib import Path

import h5py
import numpy as np


def visit(name, obj):
    if isinstance(obj, h5py.Dataset):
        storage = obj.attrs.get("storage")
        suffix = f", storage={storage}" if storage else ""
        print(f"{name}: shape={obj.shape}, dtype={obj.dtype}{suffix}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    with h5py.File(args.episode, "r") as file:
        print(f"file: {args.episode}")
        for key, value in file.attrs.items():
            print(f"attr/{key}: {value}")
        file.visititems(visit)
        stamps = file["timestamps_ns"][:]
        if "action_valid" in file:
            print(f"valid actions: {int(np.count_nonzero(file['action_valid'][:]))}/{len(stamps)}")
        if len(stamps) > 1:
            delta_ms = np.diff(stamps) / 1e6
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
