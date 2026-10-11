"""Adapter from live Quest TCP targets to safety's joint-action QP-CBF."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

from .transforms import matrix_to_rotvec
from .target_limiter import avoidance_cartesian_limits, limit_cartesian_target


class TeleopAvoidance:
    def __init__(self, config: dict):
        options = config.get("obstacle_avoidance", {})
        root = Path(options.get("safety_root", "../safety")).expanduser()
        if not root.is_absolute():
            root = Path(__file__).resolve().parents[1] / root
        root = root.resolve()
        if not (root / "obstacle_validation" / "cbf.py").is_file():
            raise FileNotFoundError(f"safety project is unavailable: {root}")
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        from obstacle_validation.cbf import RecordedPathCBF
        from obstacle_validation.core import CollisionChecker
        from obstacle_validation.lerobot_replay import _motion_safe
        from obstacle_validation.runtime import boxes_from_json, read_snapshot
        from real_scripts.dual_ur7e_surface import DualUR7eSurfacePointSampler
        from real_scripts.ur7e_collision_mesh import flange_transform
        from safety_module.dual_camera_calibration import validate_calibration_identity
        from safety_module.project_config import load_project_config

        project_path = Path(options.get("project_config", root / "configs/ur7e_dual_quest3.yaml")).expanduser()
        if not project_path.is_absolute():
            project_path = root / project_path
        project_path = project_path.resolve()
        project, _ = load_project_config(project_path)
        self.safety = project["safety"]
        if not self.safety.get("camera_calibration_valid", False):
            raise RuntimeError("safety dual-camera calibration is not validated")
        self.sampler = DualUR7eSurfacePointSampler(
            points_per_link=int(options.get("points_per_link", 16)), project_config=project_path)
        if not all(self.sampler.tcp_transform_verified.values()) or not self.sampler.calibration_digest:
            raise RuntimeError("safety dual TCP / camera calibration is not verified")
        self.project = RecordedPathCBF(
            self.sampler, np.zeros(2), collision_margin_m=self.safety["collision_margin_m"],
            inter_arm_margin_m=self.safety["inter_arm_margin_m"],
            trigger_margin_m=self.safety["trigger_margin_m"], alpha=self.safety["cbf_alpha"],
            iterations=self.safety["cbf_iterations"])
        self.CollisionChecker = CollisionChecker
        self.motion_safe = _motion_safe
        self.boxes_from_json = boxes_from_json
        self.read_snapshot = read_snapshot
        self.validate_calibration_identity = validate_calibration_identity
        self.flange_transform = flange_transform
        self.url = str(options.get("obstacle_url", self.safety["obstacle_url"]))
        self.age = float(self.safety["stale_timeout_s"])
        self.ignore_snapshot_age = options.get("ignore_snapshot_age", False) is True
        self.period = float(options.get("period_s", project["obstacle_validation"]["period_s"]))
        self.speed = float(options.get("joint_speed_rad_s", project["obstacle_validation"]["joint_speed_rad_s"]))
        self.acceleration = float(options.get("joint_acceleration_rad_s2", project["obstacle_validation"]["joint_acceleration_rad_s2"]))
        if min(self.period, self.speed, self.acceleration) <= 0:
            raise ValueError("avoidance motion parameters must be positive")
        self.max_translation, self.max_rotation = avoidance_cartesian_limits(config, self.period)
        self.previous_delta = np.zeros(12)
        self.last_step = 0.0

    def _snapshot(self) -> dict:
        if self.ignore_snapshot_age:
            # Keep all of safety's schema/status/clock-domain checks while
            # opting out of its age cutoff for dedicated teleoperation only.
            # The HTTP timeout remains short even for an indefinitely old frame.
            return self.read_snapshot(self.url, float("inf"), http_timeout_s=0.5)
        return self.read_snapshot(self.url, self.age)

    def _tcp(self, q: np.ndarray, index: int) -> np.ndarray:
        name = ("left_arm", "right_arm")[index]
        return self.flange_transform(q[index * 6:(index + 1) * 6]) @ self.sampler.flange_to_tcp[name]

    def _nominal(self, q: np.ndarray, goals: dict[str, np.ndarray], moving: dict[str, bool]) -> np.ndarray:
        step = np.zeros(12)
        for index, role in enumerate(("left", "right")):
            if not moving[role]:
                continue
            actual = self._tcp(q, index)
            goal = goals[role]
            error = np.r_[goal[:3, 3] - actual[:3, 3],
                          matrix_to_rotvec(goal[:3, :3] @ actual[:3, :3].T)]
            # Differential IK is local to the current measured configuration.
            # QP-CBF receives this bounded hand-driven joint action directly.
            jacobian = np.empty((6, 6))
            for axis in range(6):
                perturbed = q.copy()
                perturbed[index * 6 + axis] += 1e-4
                future = self._tcp(perturbed, index)
                jacobian[:, axis] = np.r_[(future[:3, 3] - actual[:3, 3]) / 1e-4,
                    matrix_to_rotvec(future[:3, :3] @ actual[:3, :3].T) / 1e-4]
            damping = 1e-3
            delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(6), error)
            if not np.isfinite(delta).all():
                raise RuntimeError("differential IK returned non-finite action")
            step[index * 6:(index + 1) * 6] = np.clip(delta, -self.speed * self.period,
                                                       self.speed * self.period)
        return step

    def compute(self, q: np.ndarray, grip: np.ndarray, goals: dict[str, np.ndarray],
                moving: dict[str, bool], measured_tcp: dict[str, np.ndarray] | None = None,
                *, optimize: bool = True) -> tuple[np.ndarray | None, dict]:
        if measured_tcp is not None:
            for index, role in enumerate(("left", "right")):
                estimated = self._tcp(q, index)
                measured = measured_tcp[role]
                position_error = np.linalg.norm(estimated[:3, 3] - measured[:3, 3])
                rotation_error = np.linalg.norm(matrix_to_rotvec(
                    estimated[:3, :3] @ measured[:3, :3].T))
                if position_error > 0.03 or rotation_error > 0.15:
                    raise RuntimeError(f"{role} safety FK differs from active TCP feedback")
        snapshot = self._snapshot()
        self.validate_calibration_identity(snapshot, self.sampler.calibration_digest)
        boxes = self.boxes_from_json(snapshot)
        # Limit the next 50 ms servoJ action at the measured TCP, not the full
        # hand displacement since the clutch was anchored.
        bounded_goals = {
            role: limit_cartesian_target(
                measured_tcp[role] if measured_tcp is not None else self._tcp(q, index),
                goals[role], self.max_translation, self.max_rotation)
            for index, role in enumerate(("left", "right")) if moving[role]
        }
        nominal = self._nominal(q, bounded_goals, moving)
        checker = self.CollisionChecker(self.sampler, grip, boxes,
            margin=self.safety["collision_margin_m"],
            inter_arm_margin=self.safety["inter_arm_margin_m"])
        # safety's swept fixed-point test includes the OBB clearance and the
        # inter-arm clearance. The usual clear step never enters the QP solver.
        if (checker.state_safe(q + nominal) and
                self.motion_safe(self.sampler, q, grip, nominal, grip, boxes, self.safety, True)):
            self.previous_delta = nominal.copy()
            self.last_step = time.monotonic()
            return q + nominal, {"success": True, "needs_projection": False,
                                 "projection_method": "nominal_clear"}
        if not optimize:
            return None, {"success": True, "needs_projection": True,
                          "projection_method": "pending"}
        self.project.gripper = grip.copy()
        bound = self.speed * self.period
        lower, upper = q - bound, q + bound
        for index, role in enumerate(("left", "right")):
            if not moving[role]:
                lower[index * 6:(index + 1) * 6] = q[index * 6:(index + 1) * 6]
                upper[index * 6:(index + 1) * 6] = q[index * 6:(index + 1) * 6]
        corrected, info = self.project.project(
            q, nominal, checker, lower=lower, upper=upper,
            max_step=bound, previous_delta=self.previous_delta,
            acceleration_step=self.acceleration * self.period ** 2,
            target_gripper=grip,
            swept_safe=lambda delta: self.motion_safe(
                self.sampler, q, grip, delta, grip, boxes, self.safety, True))
        if not info["success"] or not np.isfinite(corrected).all():
            raise RuntimeError(f"CBF projection failed: {info.get('reason', 'unknown')}")
        latest = self._snapshot()
        self.validate_calibration_identity(latest, self.sampler.calibration_digest)
        if not self.motion_safe(self.sampler, q, grip, corrected, grip,
                                self.boxes_from_json(latest), self.safety, True):
            raise RuntimeError("updated obstacle snapshot blocks the CBF action")
        for index, role in enumerate(("left", "right")):
            if not moving[role] and np.max(np.abs(corrected[index * 6:(index + 1) * 6])) > 1e-8:
                raise RuntimeError(f"CBF requires movement of released {role} arm")
        self.previous_delta = corrected.copy()
        self.last_step = time.monotonic()
        return q + corrected, info
