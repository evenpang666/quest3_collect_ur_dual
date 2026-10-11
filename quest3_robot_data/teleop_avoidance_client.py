"""Small local client for the safety Python avoidance worker.

ROS Humble uses Python 3.10 here; the existing safety environment uses 3.11.
Keep SciPy, trimesh and the QP model out of the ROS process.
"""
from __future__ import annotations

import json
import os
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np


class SnapshotExpiredError(RuntimeError):
    """The detector's capture is too old; never command motion from it."""


class TeleopAvoidanceClient:
    def __init__(self, config: dict):
        options = config.get("obstacle_avoidance", {})
        self.url = str(os.environ.get("QUEST3_AVOIDANCE_RUNTIME_URL") or
                       options.get("runtime_url", "http://127.0.0.1:8768")).rstrip("/")
        self.period = float(options.get("period_s", 0.05))
        if self.period <= 0:
            raise ValueError("avoidance period must be positive")
        self.last_step = 0.0
        self._reset_sent = False
        self._request("/health", None)
        self.reset()

    def _request(self, path: str, payload: dict | None) -> dict:
        data = None if payload is None else json.dumps(payload, separators=(",", ":")).encode("utf-8")
        request = Request(self.url + path, data=data,
                          headers={"Content-Type": "application/json"} if data is not None else {})
        try:
            with urlopen(request, timeout=0.5 if data is not None else 0.2) as response:
                answer = json.load(response)
        except HTTPError as exc:
            try:
                problem = json.load(exc)
                detail = problem.get("error", str(exc))
                error_type = problem.get("error_type")
            except (ValueError, OSError):
                detail = str(exc)
                error_type = None
            error_class = SnapshotExpiredError if error_type == "SnapshotExpired" else RuntimeError
            raise error_class(f"safety avoidance worker: {detail}") from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise RuntimeError(f"safety avoidance worker unavailable at {self.url}: {exc}") from exc
        if not isinstance(answer, dict):
            raise RuntimeError("safety avoidance worker returned invalid JSON")
        return answer

    def reset(self) -> None:
        if not self._reset_sent:
            self._request("/reset", {})
            self._reset_sent = True

    def compute(self, q: np.ndarray, grip: np.ndarray, goals: dict[str, np.ndarray],
                moving: dict[str, bool], measured_tcp: dict[str, np.ndarray],
                *, inspect: bool = False) -> tuple[np.ndarray | None, dict]:
        payload = {
            "q": np.asarray(q, dtype=float).tolist(),
            "grip": np.asarray(grip, dtype=float).tolist(),
            "goals": {role: pose.tolist() for role, pose in goals.items()},
            "moving": moving,
            "measured_tcp": {role: pose.tolist() for role, pose in measured_tcp.items()},
        }
        answer = self._request("/inspect" if inspect else "/project", payload)
        info = answer.get("info")
        if not isinstance(info, dict):
            raise RuntimeError("safety avoidance worker returned invalid projection details")
        if inspect and info.get("needs_projection") is True and answer.get("target") is None:
            return None, info
        target = np.asarray(answer.get("target"), dtype=float)
        if target.shape != (12,) or not np.isfinite(target).all():
            raise RuntimeError("safety avoidance worker returned invalid joint target")
        self.last_step = time.monotonic()
        self._reset_sent = False
        return target, info
