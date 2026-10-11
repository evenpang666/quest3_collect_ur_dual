"""Read-only hardware/configuration checks that must pass before teleoperation."""
from __future__ import annotations

import argparse
from hashlib import sha256
import re
import shutil
import socket
import subprocess
from pathlib import Path

import numpy as np
import yaml


def connected_realsense_serials() -> set[str]:
    """Enumerate cameras through Python bindings, or the installed librealsense CLI."""
    try:
        import pyrealsense2 as rs

        return {device.get_info(rs.camera_info.serial_number) for device in rs.context().query_devices()}
    except ImportError:
        executable = shutil.which("rs-enumerate-devices")
        if executable is None:
            raise RuntimeError("neither pyrealsense2 nor rs-enumerate-devices is available")
        result = subprocess.run(
            [executable, "-s"], check=True, text=True, capture_output=True, timeout=10
        )
        return set(re.findall(r"\b[0-9]{6,}\b", result.stdout))


def resolve(config_path: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else (config_path.parent / path).resolve()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--skip-connectivity", action="store_true")
    parser.add_argument("--skip-camera-check", action="store_true",
                        help="Do not query RealSense devices; use when another process owns them")
    args = parser.parse_args()
    path = args.config.expanduser().resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    failures: list[str] = []
    warnings: list[str] = []
    motion_enabled = bool(config.get("rtde", {}).get("enable_motion", False))
    debug_bypass = bool(config.get("debug", {}).get("allow_uncalibrated_receive_only", False))
    if debug_bypass and motion_enabled:
        failures.append("uncalibrated debug mode is forbidden when enable_motion=true")
    allow_uncalibrated = debug_bypass and not motion_enabled

    roles = {arm.get("role") for arm in config.get("arms", {}).values()}
    expected = {"left"} if config.get("mode") == "single" else {"left", "right"}
    if roles != expected:
        failures.append(f"arm roles {roles} do not match mode={config.get('mode')}")
    for name, arm in config.get("arms", {}).items():
        if "CHANGE_ME" in str(arm.get("robot_ip")):
            failures.append(f"{name}: robot_ip is not configured")
        # Every configured scene camera needs an extrinsic for each arm.
        # Front-only commissioning configurations remain supported.
        required_fields = ["front_calibration"]
        if "side" in config.get("cameras", {}):
            required_fields.append("side_calibration")
        if arm.get("tcp_source", "calibration_file") != "robot_active_tcp":
            required_fields.append("tcp_calibration")
        for field in required_fields:
            if allow_uncalibrated:
                warnings.append(f"{name}: {field} skipped in receive-only debug mode")
                continue
            if not arm.get(field):
                failures.append(f"{name}: {field} is not configured")
                continue
            calibration = resolve(path, arm[field])
            if not calibration.is_file():
                failures.append(f"{name}: missing {field}: {calibration}")
                continue
            value = yaml.safe_load(calibration.read_text(encoding="utf-8"))
            if not value.get("calibrated", False):
                failures.append(f"{name}: {field} is calibrated:false")
        if not args.skip_connectivity and "CHANGE_ME" not in str(arm.get("robot_ip")):
            for port, service in ((30004, "RTDE"), (int(config["rtde"]["gripper_port"]), "Robotiq")):
                try:
                    with socket.create_connection((arm["robot_ip"], port), timeout=0.5):
                        pass
                except OSError as exc:
                    failures.append(f"{name}: {service} port {port} unreachable: {exc}")
            # A virtual Cartesian workspace is optional.  When configured,
            # verify the initial TCP so the first clutch anchor is usable.
            has_workspace = "workspace_min" in arm or "workspace_max" in arm
            if has_workspace and ("workspace_min" not in arm or "workspace_max" not in arm):
                failures.append(f"{name}: workspace_min and workspace_max must be set together")
            elif motion_enabled and has_workspace:
                try:
                    from rtde_receive import RTDEReceiveInterface

                    receive = RTDEReceiveInterface(arm["robot_ip"])
                    tcp = np.asarray(receive.getActualTCPPose(), dtype=float)
                    lower = np.asarray(arm["workspace_min"], dtype=float)
                    upper = np.asarray(arm["workspace_max"], dtype=float)
                    if tcp.shape != (6,) or np.any(tcp[:3] < lower) or np.any(tcp[:3] > upper):
                        failures.append(
                            f"{name}: current TCP {tcp[:3].round(4).tolist()} is outside "
                            f"workspace [{lower.tolist()}, {upper.tolist()}]"
                        )
                except Exception as exc:
                    failures.append(f"{name}: cannot read current TCP for workspace check: {exc}")

    if config.get("dual_calibration") and not allow_uncalibrated:
        summary_path = resolve(path, config["dual_calibration"])
        if not summary_path.is_file():
            failures.append(f"missing dual_calibration: {summary_path}")
        else:
            summary = yaml.safe_load(summary_path.read_text(encoding="utf-8")) or {}
            if not summary.get("calibrated", False):
                failures.append("dual_calibration is calibrated:false; cross-arm consistency must pass")
            sources = summary.get("source_calibrations", {})
            for arm in config.get("arms", {}).values():
                for camera in ("front", "side"):
                    source = sources.get(f"{arm.get('role')}/{camera}", {})
                    expected_path = arm.get(f"{camera}_calibration")
                    source_path = Path(source.get("path", ""))
                    if (not expected_path or not source_path.is_file()
                            or source_path.resolve() != resolve(path, expected_path)
                            or sha256(source_path.read_bytes()).hexdigest() != source.get("sha256")):
                        failures.append("dual_calibration is stale; rerun dual calibration to recompute transforms")
                        break

    configured_serials = {str(camera.get("serial")) for camera in config.get("cameras", {}).values()}
    if any("CHANGE_ME" in serial for serial in configured_serials):
        failures.append("one or more Realsense serial numbers are not configured")
    if len(configured_serials) != len(config.get("cameras", {})):
        failures.append("Realsense serial numbers must be unique")
    if not args.skip_connectivity and not args.skip_camera_check:
        try:
            connected = connected_realsense_serials()
            missing = configured_serials - connected
            if missing:
                failures.append(f"Realsense devices not connected: {sorted(missing)}")
        except Exception as exc:
            failures.append(f"cannot enumerate Realsense devices: {exc}")
    if not args.skip_connectivity:
        try:
            # The first invocation may need to launch the local ADB daemon;
            # Quest USB enumeration commonly exceeds three seconds then.  Do
            # it explicitly and allow a bounded startup window before reading
            # the authorized-device list.
            subprocess.run(
                ["adb", "start-server"], check=True, text=True, capture_output=True, timeout=12
            )
            adb = subprocess.run(
                ["adb", "devices"], check=True, text=True, capture_output=True, timeout=8
            ).stdout
            if "\tdevice" not in adb:
                failures.append("Quest is not authorized in `adb devices`")
        except Exception as exc:
            failures.append(f"cannot check Quest ADB: {exc}")

    if failures:
        print("PREFLIGHT FAILED")
        for failure in failures:
            print(f"  - {failure}")
        raise SystemExit(1)
    motion = bool(config.get("rtde", {}).get("enable_motion", False))
    if warnings:
        print("PREFLIGHT WARNINGS")
        for warning in warnings:
            print(f"  - {warning}")
    if args.skip_camera_check:
        print("PREFLIGHT PASS: robot/Quest connectivity valid; camera enumeration skipped")
    else:
        print("PREFLIGHT PASS: handedness, serial numbers and connectivity are valid")
    print(f"MOTION GATE: {'ENABLED — real robot can move' if motion else 'receive-only; robot cannot move'}")


if __name__ == "__main__":
    main()
