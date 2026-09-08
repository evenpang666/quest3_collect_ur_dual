"""Interactive pivot calibration for a UR TCP translation.

Before running, set the active TCP on the pendant to [0,0,0,0,0,0], so
getActualTCPPose reports the tool flange. Keep the physical tool tip fixed in a
small pivot fixture and capture varied flange orientations.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from .transforms import ur_pose_to_matrix


def solve_pivot(flange_poses: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray, float, float]:
    if len(flange_poses) < 4:
        raise ValueError("at least four poses are required")
    rows, rhs = [], []
    for pose in flange_poses:
        rows.append(np.hstack((pose[:3, :3], -np.eye(3))))
        rhs.append(-pose[:3, 3])
    a, b = np.vstack(rows), np.concatenate(rhs)
    solution, _, _, singular = np.linalg.lstsq(a, b, rcond=None)
    tcp, pivot = solution[:3], solution[3:]
    residuals = np.array([pose[:3, :3] @ tcp + pose[:3, 3] - pivot for pose in flange_poses])
    rms = float(np.sqrt(np.mean(np.sum(residuals**2, axis=1))))
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else float("inf")
    return tcp, pivot, rms, condition


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--base-frame", required=True)
    parser.add_argument("--flange-frame", default="tool0")
    parser.add_argument("--tcp-frame", required=True)
    parser.add_argument("--samples", type=int, default=12)
    parser.add_argument("--max-rms-mm", type=float, default=1.5)
    parser.add_argument(
        "--tcp-rotation-rotvec", type=float, nargs=3, default=(0.0, 0.0, 0.0),
        metavar=("RX", "RY", "RZ"),
        help="known TCP orientation offset from CAD/fixture; pivot calibration only solves translation",
    )
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite {args.output}")
    from rtde_receive import RTDEReceiveInterface

    receiver = RTDEReceiveInterface(args.robot_ip)
    print("确认示教器当前 TCP 为全零。将工具尖端固定在同一点，每次改变法兰姿态后按 Enter；输入 q 退出。")
    poses: list[np.ndarray] = []
    while len(poses) < args.samples:
        answer = input(f"[{len(poses)+1}/{args.samples}] 就位后按 Enter: ").strip().lower()
        if answer == "q":
            raise SystemExit("calibration cancelled")
        pose = ur_pose_to_matrix(receiver.getActualTCPPose())
        poses.append(pose)
        print(f"captured flange xyz: {pose[:3, 3]}")
    tcp, pivot, rms, condition = solve_pivot(poses)
    calibrated = rms * 1000 <= args.max_rms_mm and condition < 1e6
    result = {
        "calibrated": bool(calibrated),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "method": "pivot calibration; active robot TCP was zero",
        "base_frame": args.base_frame,
        "parent_frame": args.flange_frame,
        "child_frame": args.tcp_frame,
        "tcp_offset_xyz_rotvec": [*tcp.tolist(), *args.tcp_rotation_rotvec],
        "pivot_point_in_base": pivot.tolist(),
        "samples": len(poses),
        "rms_error_mm": rms * 1000,
        "condition_number": condition,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(result, sort_keys=False))
    if not calibrated:
        raise SystemExit(f"calibration quality failed; file retained with calibrated:false: {args.output}")
    print(f"PASS: write this offset to the UR TCP configuration: {args.output}")


if __name__ == "__main__":
    main()
