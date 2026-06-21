#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np


def heart_3d(n_points: int = 200, scale: float = 0.05, center=(0.18, 0.0, 0.12), plane: str = "YZ") -> np.ndarray:
    t = np.linspace(0.0, 2.0 * math.pi, n_points, endpoint=False)
    u = 16.0 * np.sin(t) ** 3
    v = 13.0 * np.cos(t) - 5.0 * np.cos(2.0 * t) - 2.0 * np.cos(3.0 * t) - np.cos(4.0 * t)
    m = max(float(np.max(np.abs(u))), float(np.max(np.abs(v))))
    u, v = u / m, v / m
    pts = np.zeros((n_points, 3), dtype=float)
    cx, cy, cz = center
    if plane == "YZ":
        pts[:, 0] = cx; pts[:, 1] = cy + scale * u; pts[:, 2] = cz + scale * v
    elif plane == "XZ":
        pts[:, 0] = cx + scale * u; pts[:, 1] = cy; pts[:, 2] = cz + scale * v
    elif plane == "XY":
        pts[:, 0] = cx + scale * u; pts[:, 1] = cy + scale * v; pts[:, 2] = cz
    else:
        raise ValueError(f"unknown plane: {plane}")
    return pts


def line_3d(n_points: int = 120, start=(0.15, -0.04, 0.10), end=(0.21, 0.04, 0.14)) -> np.ndarray:
    return np.linspace(np.asarray(start, dtype=float), np.asarray(end, dtype=float), n_points)


def circle_3d(n_points: int = 160, radius: float = 0.035, center=(0.18, 0.0, 0.12), plane: str = "YZ") -> np.ndarray:
    t = np.linspace(0.0, 2.0 * math.pi, n_points, endpoint=False)
    pts = np.zeros((n_points, 3), dtype=float)
    cx, cy, cz = center
    if plane == "YZ":
        pts[:, 0] = cx; pts[:, 1] = cy + radius * np.cos(t); pts[:, 2] = cz + radius * np.sin(t)
    elif plane == "XZ":
        pts[:, 0] = cx + radius * np.cos(t); pts[:, 1] = cy; pts[:, 2] = cz + radius * np.sin(t)
    elif plane == "XY":
        pts[:, 0] = cx + radius * np.cos(t); pts[:, 1] = cy + radius * np.sin(t); pts[:, 2] = cz
    else:
        raise ValueError(f"unknown plane: {plane}")
    return pts


def load_csv_xyz(path: str | Path) -> np.ndarray:
    rows: list[list[float]] = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append([float(row["x"]), float(row["y"]), float(row["z"])])
    if not rows:
        raise ValueError(f"empty target csv: {path}")
    return np.asarray(rows, dtype=float)


def make_trajectory(kind: str, *, n_points: int, scale: float, center: tuple[float, float, float], plane: str, csv_path: str | None) -> np.ndarray:
    if kind == "heart":
        return heart_3d(n_points=n_points, scale=scale, center=center, plane=plane)
    if kind == "circle":
        return circle_3d(n_points=n_points, radius=scale, center=center, plane=plane)
    if kind == "line":
        return line_3d(n_points=n_points)
    if kind == "csv":
        if not csv_path:
            raise ValueError("trajectory kind 'csv' requires --target-csv")
        return load_csv_xyz(csv_path)
    raise ValueError(f"unknown trajectory: {kind}")