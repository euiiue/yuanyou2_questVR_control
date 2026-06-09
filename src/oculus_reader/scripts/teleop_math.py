#!/usr/bin/env python3
from dataclasses import dataclass, field
import math
from typing import Iterable

import numpy as np


def _as_vec3(value: Iterable[float], name: str) -> np.ndarray:
    arr = np.asarray(list(value), dtype=float)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return arr


def _as_quat(value: Iterable[float], name: str) -> np.ndarray:
    arr = np.asarray(list(value), dtype=float)
    if arr.shape != (4,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be a finite quaternion [x, y, z, w]")
    return normalize_quat(arr)


def _normalize_hand(hand_name: str) -> str:
    hand = str(hand_name).strip().lower()
    if hand not in ("left", "right"):
        raise ValueError("hand_name must be 'left' or 'right'")
    return hand


def normalize_quat(q: Iterable[float]) -> np.ndarray:
    quat = np.asarray(list(q), dtype=float)
    norm = np.linalg.norm(quat)
    if quat.shape != (4,) or norm < 1e-12 or not np.all(np.isfinite(quat)):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def quat_multiply(q1: Iterable[float], q2: Iterable[float]) -> np.ndarray:
    x1, y1, z1, w1 = normalize_quat(q1)
    x2, y2, z2, w2 = normalize_quat(q2)
    return normalize_quat(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quat_inverse(q: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quat(q)
    return np.array([-x, -y, -z, w], dtype=float)


def axis_angle_to_quat(axis: Iterable[float], angle: float) -> np.ndarray:
    axis_arr = np.asarray(list(axis), dtype=float)
    norm = np.linalg.norm(axis_arr)
    if axis_arr.shape != (3,) or norm < 1e-12 or abs(angle) < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    axis_arr = axis_arr / norm
    half = float(angle) / 2.0
    s = math.sin(half)
    return normalize_quat([axis_arr[0] * s, axis_arr[1] * s, axis_arr[2] * s, math.cos(half)])


def quat_to_axis_angle(q: Iterable[float]):
    x, y, z, w = normalize_quat(q)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w

    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-8 or abs(angle) < 1e-12:
        return np.array([1.0, 0.0, 0.0], dtype=float), 0.0
    return np.array([x / s, y / s, z / s], dtype=float), angle


def slerp(q0: Iterable[float], q1: Iterable[float], t: float) -> np.ndarray:
    q0_arr = normalize_quat(q0)
    q1_arr = normalize_quat(q1)
    t = max(0.0, min(1.0, float(t)))

    dot = float(np.dot(q0_arr, q1_arr))
    if dot < 0.0:
        q1_arr = -q1_arr
        dot = -dot
    dot = max(-1.0, min(1.0, dot))

    if dot > 0.9995:
        return normalize_quat(q0_arr + t * (q1_arr - q0_arr))

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return normalize_quat(s0 * q0_arr + s1 * q1_arr)


def map_delta_pos(hand_name: str, delta_pos_vr: Iterable[float]) -> np.ndarray:
    hand = _normalize_hand(hand_name)
    vx, vy, vz = _as_vec3(delta_pos_vr, "delta_pos_vr")
    if hand == "right":
        return np.array([vz, vy, vx], dtype=float)
    return np.array([vz, -vy, -vx], dtype=float)


def map_delta_quat(hand_name: str, delta_quat_vr: Iterable[float]) -> np.ndarray:
    hand = _normalize_hand(hand_name)
    axis_vr, angle = quat_to_axis_angle(delta_quat_vr)
    ax, ay, az = axis_vr
    if hand == "right":
        axis_robot = np.array([-az, -ay, -ax], dtype=float)
    else:
        axis_robot = np.array([-az, ay, ax], dtype=float)
    return axis_angle_to_quat(axis_robot, angle)


def apply_deadband_vector(vec: Iterable[float], deadband: float) -> np.ndarray:
    arr = _as_vec3(vec, "vec")
    db = max(0.0, float(deadband))
    return np.where(np.abs(arr) < db, 0.0, arr)


def limit_anchor_delta(mapped_delta: Iterable[float], limits: Iterable[float]) -> np.ndarray:
    mapped = _as_vec3(mapped_delta, "mapped_delta")
    limit = _as_vec3(limits, "limits")
    active = limit > 0.0
    limited = mapped.copy()
    limited[active] = np.clip(limited[active], -limit[active], limit[active])
    return limited


def limit_step(current: Iterable[float], target: Iterable[float], max_step: float) -> np.ndarray:
    current_arr = _as_vec3(current, "current")
    target_arr = _as_vec3(target, "target")
    max_step = max(0.0, float(max_step))
    if max_step <= 0.0:
        return target_arr
    delta = target_arr - current_arr
    dist = np.linalg.norm(delta)
    if dist > max_step and dist > 1e-12:
        delta = delta / dist * max_step
    return current_arr + delta


def low_pass(current: Iterable[float], target: Iterable[float], alpha: float) -> np.ndarray:
    current_arr = _as_vec3(current, "current")
    target_arr = _as_vec3(target, "target")
    alpha = max(0.0, min(1.0, float(alpha)))
    return alpha * target_arr + (1.0 - alpha) * current_arr


def filter_target_quat(
    current_filtered_quat: Iterable[float],
    raw_target_quat: Iterable[float],
    rot_deadband: float,
    max_step_angle: float,
    alpha_rot: float,
) -> np.ndarray:
    q_curr = normalize_quat(current_filtered_quat)
    q_raw = normalize_quat(raw_target_quat)
    q_err = quat_multiply(quat_inverse(q_curr), q_raw)
    axis, angle = quat_to_axis_angle(q_err)
    if abs(angle) < max(0.0, float(rot_deadband)):
        return q_curr.copy()
    limited_angle = min(angle, max(0.0, float(max_step_angle)))
    q_step = axis_angle_to_quat(axis, limited_angle)
    q_limited = quat_multiply(q_curr, q_step)
    return slerp(q_curr, q_limited, alpha_rot)


@dataclass
class AnchoredTeleopConfig:
    hand_name: str = "right"
    input_pos_alpha: float = 0.30
    input_rot_alpha: float = 0.45
    alpha_pos: float = 0.55
    alpha_rot: float = 0.55
    max_step_pos: float = 0.012
    max_step_angle: float = 0.16
    deadband_pos: float = 0.0015
    rot_deadband: float = 0.02
    output_gain: float = 1.0
    max_anchor_delta_xyz: Iterable[float] = field(default_factory=lambda: [0.45, 0.45, 0.35])
    target_pos_hold_deadband: float = 0.002

    def __post_init__(self):
        self.hand_name = _normalize_hand(self.hand_name)
        self.input_pos_alpha = max(0.0, min(1.0, float(self.input_pos_alpha)))
        self.input_rot_alpha = max(0.0, min(1.0, float(self.input_rot_alpha)))
        self.alpha_pos = max(0.0, min(1.0, float(self.alpha_pos)))
        self.alpha_rot = max(0.0, min(1.0, float(self.alpha_rot)))
        self.max_step_pos = max(0.0, float(self.max_step_pos))
        self.max_step_angle = max(0.0, float(self.max_step_angle))
        self.deadband_pos = max(0.0, float(self.deadband_pos))
        self.rot_deadband = max(0.0, float(self.rot_deadband))
        self.output_gain = max(0.0, float(self.output_gain))
        self.max_anchor_delta_xyz = _as_vec3(self.max_anchor_delta_xyz, "max_anchor_delta_xyz")
        self.target_pos_hold_deadband = max(0.0, float(self.target_pos_hold_deadband))


@dataclass
class AnchoredTeleopTarget:
    position: np.ndarray
    quaternion: np.ndarray
    raw_delta_pos: np.ndarray
    mapped_delta_pos: np.ndarray
    raw_target_pos: np.ndarray


class AnchoredTeleopMapper:
    def __init__(self, config: AnchoredTeleopConfig):
        self.config = config
        self.handle_anchor_pos = None
        self.handle_anchor_quat = None
        self.robot_anchor_pos = None
        self.robot_anchor_quat = None
        self.filtered_input_pos = None
        self.filtered_input_quat = None
        self.filtered_target_pos = None
        self.filtered_target_quat = None

    @property
    def ready(self) -> bool:
        return self.robot_anchor_pos is not None and self.handle_anchor_pos is not None

    def reset_anchor(self, handle_pos, handle_quat, robot_pos, robot_quat):
        self.handle_anchor_pos = _as_vec3(handle_pos, "handle_pos")
        self.handle_anchor_quat = _as_quat(handle_quat, "handle_quat")
        self.robot_anchor_pos = _as_vec3(robot_pos, "robot_pos")
        self.robot_anchor_quat = _as_quat(robot_quat, "robot_quat")
        self.filtered_input_pos = None
        self.filtered_input_quat = None
        self.filtered_target_pos = self.robot_anchor_pos.copy()
        self.filtered_target_quat = self.robot_anchor_quat.copy()

    def update(self, handle_pos, handle_quat) -> AnchoredTeleopTarget:
        if not self.ready:
            raise RuntimeError("AnchoredTeleopMapper.update called before reset_anchor")

        cfg = self.config
        current_handle_pos = _as_vec3(handle_pos, "handle_pos")
        current_handle_quat = _as_quat(handle_quat, "handle_quat")

        raw_delta_pos = current_handle_pos - self.handle_anchor_pos
        if self.filtered_input_pos is None:
            self.filtered_input_pos = raw_delta_pos.copy()
        else:
            self.filtered_input_pos = (
                cfg.input_pos_alpha * raw_delta_pos
                + (1.0 - cfg.input_pos_alpha) * self.filtered_input_pos
            )

        delta_after_deadband = apply_deadband_vector(self.filtered_input_pos, cfg.deadband_pos)
        mapped_delta = map_delta_pos(cfg.hand_name, delta_after_deadband) * cfg.output_gain
        mapped_delta = limit_anchor_delta(mapped_delta, cfg.max_anchor_delta_xyz)
        raw_target_pos = self.robot_anchor_pos + mapped_delta

        target_limited = limit_step(self.filtered_target_pos, raw_target_pos, cfg.max_step_pos)
        filtered_new_pos = low_pass(self.filtered_target_pos, target_limited, cfg.alpha_pos)
        if np.linalg.norm(filtered_new_pos - self.filtered_target_pos) < cfg.target_pos_hold_deadband:
            filtered_new_pos = self.filtered_target_pos.copy()

        raw_delta_quat = quat_multiply(quat_inverse(self.handle_anchor_quat), current_handle_quat)
        if self.filtered_input_quat is None:
            self.filtered_input_quat = raw_delta_quat.copy()
        else:
            self.filtered_input_quat = slerp(self.filtered_input_quat, raw_delta_quat, cfg.input_rot_alpha)

        delta_quat_robot = map_delta_quat(cfg.hand_name, self.filtered_input_quat)
        raw_target_quat = quat_multiply(delta_quat_robot, self.robot_anchor_quat)
        filtered_new_quat = filter_target_quat(
            self.filtered_target_quat,
            raw_target_quat,
            cfg.rot_deadband,
            cfg.max_step_angle,
            cfg.alpha_rot,
        )

        self.filtered_target_pos = filtered_new_pos.copy()
        self.filtered_target_quat = filtered_new_quat.copy()

        return AnchoredTeleopTarget(
            position=filtered_new_pos.copy(),
            quaternion=filtered_new_quat.copy(),
            raw_delta_pos=raw_delta_pos.copy(),
            mapped_delta_pos=mapped_delta.copy(),
            raw_target_pos=raw_target_pos.copy(),
        )


__all__ = [
    "AnchoredTeleopConfig",
    "AnchoredTeleopMapper",
    "AnchoredTeleopTarget",
    "apply_deadband_vector",
    "axis_angle_to_quat",
    "filter_target_quat",
    "limit_anchor_delta",
    "limit_step",
    "low_pass",
    "map_delta_pos",
    "map_delta_quat",
    "normalize_quat",
    "quat_inverse",
    "quat_multiply",
    "quat_to_axis_angle",
    "slerp",
]
