#!/usr/bin/env python3
"""SO-101 FK/IK utilities adapted for the user's calibrated URDF.

The IK follows the damped least-squares / Levenberg-Marquardt style used in
jooyongsim/soarm_ik_mecha_tutorial, but is packaged as reusable functions for
hardware control and RViz visualization.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class JointSpec:
    name: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    limits: tuple[float, float]
    joint_type: str


# Chain: base_link -> ... -> gripper_frame_link.
# This intentionally excludes the moving gripper jaw because Cartesian IK should
# solve arm pose, not gripper opening.
JOINTS: tuple[JointSpec, ...] = (
    JointSpec("shoulder_pan", (0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159), (0, 0, 1), (-1.91986, 1.91986), "revolute"),
    JointSpec("shoulder_lift", (-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0), (0, 0, 1), (-1.74533, 1.74533), "revolute"),
    JointSpec("elbow_flex", (-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708), (0, 0, 1), (-1.69, 1.69), "revolute"),
    JointSpec("wrist_flex", (-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708), (0, 0, 1), (-1.65806, 1.65806), "revolute"),
    JointSpec("wrist_roll", (5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159), (0, 0, 1), (-2.74385, 2.84121), "revolute"),
    JointSpec("gripper_frame_joint", (-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0), (0, 0, 0), (0, 0), "fixed"),
)

ARM_JOINT_NAMES = tuple(j.name for j in JOINTS if j.joint_type == "revolute")
ALL_JOINT_NAMES = ARM_JOINT_NAMES + ("gripper",)
LIMITS_LOWER = np.array([j.limits[0] for j in JOINTS if j.joint_type == "revolute"], dtype=float)
LIMITS_UPPER = np.array([j.limits[1] for j in JOINTS if j.joint_type == "revolute"], dtype=float)


def rx(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def ry(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rz(yaw) @ ry(pitch) @ rx(roll)


def rodrigues(axis: Iterable[float], theta: float) -> np.ndarray:
    a = np.asarray(tuple(axis), dtype=float)
    n = np.linalg.norm(a)
    if n == 0:
        return np.eye(3)
    x, y, z = a / n
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    return np.eye(3) + math.sin(theta) * k + (1.0 - math.cos(theta)) * (k @ k)


def t_from_rt(R: np.ndarray, t: Iterable[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tuple(t), dtype=float)
    return T


def joint_t(joint: JointSpec, theta: float) -> np.ndarray:
    T_origin = t_from_rt(rpy(*joint.origin_rpy), joint.origin_xyz)
    if joint.joint_type == "fixed":
        return T_origin
    return T_origin @ t_from_rt(rodrigues(joint.axis, theta), (0.0, 0.0, 0.0))


def fk(q: Iterable[float], return_all: bool = False) -> np.ndarray | list[np.ndarray]:
    q = np.asarray(tuple(q), dtype=float)
    if q.shape[0] < len(ARM_JOINT_NAMES):
        raise ValueError(f"expected at least {len(ARM_JOINT_NAMES)} arm joints, got {q.shape[0]}")
    T = np.eye(4)
    frames = [T.copy()]
    k = 0
    for joint in JOINTS:
        if joint.joint_type == "fixed":
            T = T @ joint_t(joint, 0.0)
        else:
            T = T @ joint_t(joint, float(q[k]))
            k += 1
        if return_all:
            frames.append(T.copy())
    return frames if return_all else T


def ee_pos(q: Iterable[float]) -> np.ndarray:
    return fk(q)[:3, 3]


def position_jacobian(q: Iterable[float], eps: float = 1e-6) -> np.ndarray:
    q = np.asarray(tuple(q), dtype=float)
    J = np.zeros((3, q.shape[0]), dtype=float)
    for i in range(q.shape[0]):
        qp = q.copy(); qp[i] += eps
        qm = q.copy(); qm[i] -= eps
        J[:, i] = (ee_pos(qp) - ee_pos(qm)) / (2.0 * eps)
    return J


def ik(
    target_xyz: Iterable[float],
    q_init: Iterable[float] | None = None,
    tol: float = 1e-4,
    max_iter: int = 120,
    lam_init: float = 0.05,
    lam_factor: float = 2.0,
    step_clip: float = 0.25,
) -> tuple[np.ndarray, bool, int, float]:
    target = np.asarray(tuple(target_xyz), dtype=float)
    q = np.zeros(len(ARM_JOINT_NAMES), dtype=float) if q_init is None else np.asarray(tuple(q_init), dtype=float).copy()
    q = np.clip(q[: len(ARM_JOINT_NAMES)], LIMITS_LOWER, LIMITS_UPPER)

    lam = lam_init
    err = target - ee_pos(q)
    err_norm = float(np.linalg.norm(err))
    I3 = np.eye(3)

    for it in range(max_iter):
        if err_norm < tol:
            return q, True, it, err_norm
        J = position_jacobian(q)
        try:
            dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * I3, err)
        except np.linalg.LinAlgError:
            lam *= lam_factor
            continue
        max_step = float(np.max(np.abs(dq)))
        if max_step > step_clip:
            dq *= step_clip / max_step
        q_try = np.clip(q + dq, LIMITS_LOWER, LIMITS_UPPER)
        err_try = target - ee_pos(q_try)
        err_try_norm = float(np.linalg.norm(err_try))
        if err_try_norm < err_norm:
            q, err, err_norm = q_try, err_try, err_try_norm
            lam = max(lam / lam_factor, 1e-6)
        else:
            lam *= lam_factor
            if lam > 1e3:
                break
    return q, err_norm < tol, max_iter, err_norm


if __name__ == "__main__":
    target = np.array([0.18, 0.0, 0.12])
    q, ok, it, err = ik(target)
    print("joints:", ARM_JOINT_NAMES)
    print("target:", target)
    print("ok:", ok, "iters:", it, "err_mm:", err * 1000)
    print("q_deg:", np.rad2deg(q).round(2).tolist())