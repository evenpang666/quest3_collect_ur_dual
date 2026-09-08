"""Conservative single-joint RTDE motion probe for commissioning.

The probe is dry-run by default. Execution requires an explicit confirmation
token and always commands a return to the captured starting joint position.
It uses moveJ, so camera/TCP calibration is not involved.
"""
from __future__ import annotations

import argparse
import math
import time


CONFIRM_TOKEN = "ROBOT_AREA_CLEAR"


def _validate(args: argparse.Namespace) -> None:
    if not 0 <= args.joint_index < 6:
        raise SystemExit("--joint-index must be in [0, 5]")
    if not 0 < abs(args.delta_deg) <= 1.0:
        raise SystemExit("absolute --delta-deg must be in (0, 1.0]")
    if not 0 < args.speed <= 0.10:
        raise SystemExit("--speed must be in (0, 0.10] rad/s")
    if not 0 < args.acceleration <= 0.20:
        raise SystemExit("--acceleration must be in (0, 0.20] rad/s^2")
    if not 0 <= args.hold_sec <= 2.0:
        raise SystemExit("--hold-sec must be in [0, 2.0]")
    if args.execute and args.confirm != CONFIRM_TOKEN:
        raise SystemExit(f"execution requires --confirm {CONFIRM_TOKEN}")


def _assert_safe(receive: object) -> None:
    if receive.isEmergencyStopped():
        raise RuntimeError("robot is emergency-stopped")
    if receive.isProtectiveStopped():
        raise RuntimeError("robot is protective-stopped")
    safety_mode = int(receive.getSafetyMode())
    if safety_mode != 1:
        raise RuntimeError(f"robot safety mode is {safety_mode}, expected NORMAL (1)")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True)
    parser.add_argument("--joint-index", type=int, default=5, help="0..5; default is wrist_3")
    parser.add_argument("--delta-deg", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=0.05, help="rad/s; hard limit 0.10")
    parser.add_argument("--acceleration", type=float, default=0.10, help="rad/s^2; hard limit 0.20")
    parser.add_argument("--hold-sec", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args(argv)
    _validate(args)

    from rtde_receive import RTDEReceiveInterface

    receive = RTDEReceiveInterface(args.ip)
    try:
        _assert_safe(receive)
        start = list(receive.getActualQ())
        target = start.copy()
        target[args.joint_index] += math.radians(args.delta_deg)
        print(f"robot={args.ip} safety_mode={int(receive.getSafetyMode())}")
        print("start_rad=" + ",".join(f"{value:.6f}" for value in start))
        print(
            f"probe: joint[{args.joint_index}] {args.delta_deg:+.3f} deg, "
            f"speed={args.speed:.3f} rad/s, acceleration={args.acceleration:.3f} rad/s^2"
        )
        if not args.execute:
            print(f"DRY RUN: no command sent; add --execute --confirm {CONFIRM_TOKEN} after clearing the area")
            return

        from rtde_control import RTDEControlInterface

        control = RTDEControlInterface(args.ip)
        moved = False
        try:
            _assert_safe(receive)
            if not control.moveJ(target, args.speed, args.acceleration, False):
                raise RuntimeError("outbound moveJ returned false")
            moved = True
            reached = list(receive.getActualQ())
            error = abs(reached[args.joint_index] - target[args.joint_index])
            print(f"outbound reached; joint error={math.degrees(error):.4f} deg")
            time.sleep(args.hold_sec)
            _assert_safe(receive)
            if not control.moveJ(start, args.speed, args.acceleration, False):
                raise RuntimeError("return moveJ returned false")
            moved = False
            returned = list(receive.getActualQ())
            error = abs(returned[args.joint_index] - start[args.joint_index])
            print(f"PASS: returned to start; joint error={math.degrees(error):.4f} deg")
        finally:
            if moved and not receive.isEmergencyStopped() and not receive.isProtectiveStopped():
                print("attempting guarded return to captured start position")
                control.moveJ(start, args.speed, args.acceleration, False)
            control.stopScript()
    finally:
        receive.disconnect()


if __name__ == "__main__":
    main()
