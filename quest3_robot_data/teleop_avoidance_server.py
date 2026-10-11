"""Serve the existing safety QP-CBF to the ROS 2 teleoperation process."""
from __future__ import annotations

import argparse
import ctypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import threading
import traceback

import numpy as np
import yaml

from .teleop_avoidance import TeleopAvoidance


def _exit_with_launcher() -> None:
    """Stop the worker if its dedicated teleop launcher dies, even via SIGKILL."""
    raw_pid = os.environ.get("QUEST3_AVOIDANCE_PARENT_PID")
    if raw_pid is None:
        return
    parent_pid = int(raw_pid)
    if os.getppid() != parent_pid:
        raise SystemExit("avoidance teleop launcher already exited")
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:  # Linux PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), "could not monitor avoidance teleop launcher")
    if os.getppid() != parent_pid:
        raise SystemExit("avoidance teleop launcher exited during worker startup")


def _matrix(value: object, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a finite 4x4 pose")
    return matrix


def make_handler(adapter: TeleopAvoidance, lock: threading.Lock):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, status: int, value: dict) -> None:
            def convert(item):
                if isinstance(item, np.generic):
                    return item.item()
                if isinstance(item, np.ndarray):
                    return item.tolist()
                raise TypeError(f"unsupported response value: {type(item).__name__}")

            data = json.dumps(value, separators=(",", ":"), allow_nan=False,
                              default=convert).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self) -> None:
            if self.path != "/health":
                self._reply(404, {"error": "unknown path"})
                return
            self._reply(200, {"status": "ready", "period_s": adapter.period})

        def do_POST(self) -> None:
            if self.path not in ("/inspect", "/project", "/reset"):
                self._reply(404, {"error": "unknown path"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 <= size <= 16384:
                    raise ValueError("request size is invalid")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("request must be a JSON object")
                with lock:
                    if self.path == "/reset":
                        adapter.previous_delta = np.zeros(12)
                        result = {"status": "reset"}
                    else:
                        q = np.asarray(data["q"], dtype=float)
                        grip = np.asarray(data["grip"], dtype=float)
                        if (q.shape != (12,) or grip.shape != (2,) or
                                not np.isfinite(q).all() or not np.isfinite(grip).all() or
                                np.any(grip < 0) or np.any(grip > 1)):
                            raise ValueError("invalid joint or gripper feedback")
                        moving = data["moving"]
                        if set(moving) != {"left", "right"} or not all(
                                isinstance(moving[role], bool) for role in ("left", "right")):
                            raise ValueError("invalid moving-arm flags")
                        goals = {role: _matrix(pose, f"{role} goal") for role, pose in data["goals"].items()}
                        measured = {role: _matrix(data["measured_tcp"][role], f"{role} TCP")
                                    for role in ("left", "right")}
                        target, info = adapter.compute(q, grip, goals, moving,
                                                       measured_tcp=measured,
                                                       optimize=self.path == "/project")
                        result = {"target": None if target is None else target.tolist(), "info": info}
                self._reply(200, result)
            except (KeyError, TypeError, ValueError, RuntimeError) as exc:
                self.log_error("avoidance %s failed (%s): %s", self.path, type(exc).__name__, exc)
                traceback.print_exc()
                self._reply(409, {"error": str(exc), "error_type": type(exc).__name__})

    return Handler


def main() -> None:
    _exit_with_launcher()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8768, type=int)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.expanduser().resolve().read_text(encoding="utf-8"))
    adapter = TeleopAvoidance(config)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(adapter, threading.Lock()))
    print(f"Safety avoidance worker ready: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
