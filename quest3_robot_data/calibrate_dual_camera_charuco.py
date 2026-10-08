"""Calibrate front/side cameras to the left UR base, then the right UR base."""
from __future__ import annotations

import argparse
from copy import copy
from datetime import datetime, timezone
from hashlib import sha256
from pathlib import Path

import numpy as np
import yaml

from .calibrate_camera import average_transforms, rotation_error_deg
from .calibrate_camera_charuco import add_board_arguments, calibrate_arm
from .preflight import resolve
from .transforms import dump_transform, load_transform


def arm_arguments(args, config: dict, config_path: Path):
    """Resolve a separate camera pair and output/checkpoint set for each arm."""
    cameras = config.get("cameras", {})
    serials = [str(cameras.get(name, {}).get("serial", "")) for name in ("front", "side")]
    if any(not serial or "CHANGE_ME" in serial for serial in serials) or serials[0] == serials[1]:
        raise SystemExit("configure different front/side serial numbers in the hardware config")
    arms = config.get("arms", {})
    plan = []
    for role in ("left", "right"):
        matches = [arm for arm in arms.values() if arm.get("role") == role]
        if len(matches) != 1:
            raise SystemExit(f"hardware config must contain exactly one {role} arm")
        arm = matches[0]
        required = ("robot_ip", "base_frame", "front_calibration", "side_calibration")
        if any(not arm.get(field) or "CHANGE_ME" in str(arm[field]) for field in required):
            raise SystemExit(f"{role} arm requires {', '.join(required)}")
        run = copy(args)
        run.robot_ip, run.base_frame = arm["robot_ip"], arm["base_frame"]
        run.camera_serial, run.side_camera_serial = serials
        run.camera_frame = "front_camera_color_optical_frame"
        run.side_camera_frame = "side_camera_color_optical_frame"
        run.output = resolve(config_path, arm["front_calibration"])
        run.side_output = resolve(config_path, arm["side_calibration"])
        # A new arm session always captures its own board pose and TCP touches.
        # Resume is explicit and restricted to a single, still-fixed arm session.
        run.recapture_camera = not args.resume or args.recapture_camera
        plan.append((role, run))
    paths = [path for _, run in plan for path in (run.output, run.side_output)]
    if len(set(paths)) != 4:
        raise SystemExit("all four arm/camera calibration output paths must be different")
    return plan


def pair_complete(run) -> bool:
    """Skip only a matching, successful pair; never mix a result from another device."""
    for path, serial, frame in (
        (run.output, run.camera_serial, run.camera_frame),
        (run.side_output, run.side_camera_serial, run.side_camera_frame),
    ):
        if not path.exists():
            return False
        result = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not result.get("calibrated", False):
            return False
        if (result.get("camera_serial") != serial
                or result.get("parent_frame") != run.base_frame
                or result.get("child_frame") != frame):
            raise SystemExit(f"existing calibrated result does not match the hardware config: {path}")
    return True


def write_combined_transforms(plan, output_path: Path, max_translation_mm: float, max_rotation_deg: float):
    """Link independently placed boards through the two stationary cameras.

    left_T_right = left_T_camera @ inverse(right_T_camera).
    Each arm's board pose cancels within its own base-to-camera calibration;
    the left and right board poses and touch corner IDs need not be equal.
    """
    transforms, sources = {}, {}
    for role, run in plan:
        for name, path in (("front", run.output), ("side", run.side_output)):
            payload = path.read_bytes()
            result = yaml.safe_load(payload)
            transforms[role, name] = load_transform(result)
            sources[f"{role}/{name}"] = {"path": str(path), "sha256": sha256(payload).hexdigest()}
    base_estimates = [
        transforms["left", camera] @ np.linalg.inv(transforms["right", camera])
        for camera in ("front", "side")
    ]
    camera_estimates = [
        np.linalg.inv(transforms[role, "front"]) @ transforms[role, "side"]
        for role in ("left", "right")
    ]
    checks = {}
    for name, estimates in (("base_via_front_vs_side", base_estimates),
                            ("cameras_via_left_vs_right", camera_estimates)):
        translation_mm = float(np.linalg.norm(estimates[0][:3, 3] - estimates[1][:3, 3]) * 1000)
        rotation_deg = float(rotation_error_deg(*estimates))
        checks[name] = {
            "translation_error_mm": translation_mm, "rotation_error_deg": rotation_deg,
            "passed": bool(translation_mm <= max_translation_mm and rotation_deg <= max_rotation_deg),
        }
    left, right = plan[0][1], plan[1][1]
    output = {
        "calibrated": all(check["passed"] for check in checks.values()),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "method": "independent arm/board sessions linked through fixed front/side cameras",
        "transform_convention": "parent_T_child maps child coordinates into parent coordinates",
        "left_base_to_right_base": dump_transform(average_transforms(base_estimates), left.base_frame, right.base_frame),
        "front_camera_to_side_camera": dump_transform(average_transforms(camera_estimates), left.camera_frame, left.side_camera_frame),
        "left_base_to_right_base_via_front": dump_transform(base_estimates[0], left.base_frame, right.base_frame),
        "left_base_to_right_base_via_side": dump_transform(base_estimates[1], left.base_frame, right.base_frame),
        "consistency": checks,
        "quality_limits": {"max_translation_mm": max_translation_mm, "max_rotation_deg": max_rotation_deg},
        "source_calibrations": sources,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(output, sort_keys=False), encoding="utf-8")
    for name, check in checks.items():
        print(f"{name}: {check['translation_error_mm']:.2f} mm, {check['rotation_error_deg']:.2f} deg")
    if not output["calibrated"]:
        raise SystemExit(f"cross-arm consistency failed; retained calibrated:false summary: {output_path}")
    print(f"PASS: 双臂独立结果换算完成: {output_path}")
    return output


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path, help="dual-arm hardware YAML")
    parser.add_argument("--arm", choices=("both", "left", "right"), default="both")
    parser.add_argument("--resume", action="store_true", help="resume a selected arm with the board still fixed")
    parser.add_argument("--max-cross-translation-mm", type=float, default=10.0)
    parser.add_argument("--max-cross-rotation-deg", type=float, default=3.0)
    add_board_arguments(parser)
    args = parser.parse_args(argv)
    if args.resume and args.arm == "both":
        parser.error("--resume requires --arm left or --arm right; keep that arm's board fixed")
    if any(not np.isfinite(value) or value <= 0 for value in
           (args.max_cross_translation_mm, args.max_cross_rotation_deg)):
        parser.error("cross-arm quality limits must be finite and positive")
    config_path = args.config.expanduser().resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    plan = arm_arguments(args, config, config_path)
    combined_path = resolve(config_path, config.get("dual_calibration", "calibration/dual_arm_camera_transforms.yaml"))
    if combined_path in {path for _, run in plan for path in (run.output, run.side_output)}:
        parser.error("dual_calibration must be different from the four camera output paths")
    print("标定只读取活动 TCP，不发送机械臂运动命令。请先停止占用 front/side 的相机节点。")
    for role, run in plan:
        if args.arm != "both" and role != args.arm:
            continue
        if pair_complete(run):
            print(f"{role} 臂 front/side 已通过，保留两份结果并跳过。")
            continue
        board_instruction = (
            "续做时标定板必须仍在上次位置，禁止移动；"
            if args.resume and not run.recapture_camera else
            "可移动标定板至本臂可达且两台相机均可见的位置；"
        )
        answer = input(
            f"准备{'左' if role == 'left' else '右'}臂 ({run.robot_ip})："
            + board_instruction +
            "固定后按 Enter 开始，q 退出: "
        ).strip().lower()
        if answer == "q":
            raise SystemExit("calibration cancelled")
        print(f"开始 {role} 臂：front 观测 → side 观测 → 触碰四个共同内角 → 双相机位移检查。")
        calibrate_arm(run)
    print("PASS: 所选机械臂的 front/side 外参标定完成。")
    if all(pair_complete(run) for _, run in plan):
        write_combined_transforms(plan, combined_path, args.max_cross_translation_mm, args.max_cross_rotation_deg)
    else:
        print("另一臂尚未完成；保留本臂独立结果，待两臂均完成后自动换算基座和相机坐标。")


if __name__ == "__main__":
    main()
