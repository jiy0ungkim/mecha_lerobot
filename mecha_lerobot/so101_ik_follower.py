#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import socket
import time
from pathlib import Path
from datetime import datetime
from typing import Any

import numpy as np

from lerobot.robots import make_robot_from_config

try:
    from lerobot.robots.so_follower import SO101FollowerConfig
except Exception:
    try:
        from lerobot.robots.so101_follower import SO101FollowerConfig
    except Exception:
        from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

try:
    from .so101_kinematics import ALL_JOINT_NAMES, ARM_JOINT_NAMES, ee_pos, ik
    from .so101_target_trajectory import make_trajectory
except Exception:
    from so101_kinematics import ALL_JOINT_NAMES, ARM_JOINT_NAMES, ee_pos, ik
    from so101_target_trajectory import make_trajectory


def extract_from_named_dict(data: dict[str, Any], joint_names, prefixes) -> list[float] | None:
    values = []
    for joint_name in joint_names:
        candidates = []
        for prefix in prefixes:
            candidates.extend([
                f"{prefix}{joint_name}",
                f"{prefix}{joint_name}.pos",
                f"{prefix}{joint_name}.position",
            ])
        for key in candidates:
            if key in data:
                values.append(float(data[key]))
                break
        else:
            return None
    return values


def extract_positions(obs: Any) -> list[float]:
    if isinstance(obs, dict):
        values = extract_from_named_dict(obs, ALL_JOINT_NAMES, prefixes=["", "observation."])
        if values is not None:
            return values
        for key in ("observation.state", "state"):
            if key in obs:
                return [float(x) for x in list(obs[key])[:6]]
    raise RuntimeError(f"Cannot extract positions from observation keys={list(obs.keys()) if isinstance(obs, dict) else type(obs)}")


def convert_units(values: list[float], from_units: str, to_units: str) -> list[float]:
    if from_units == to_units:
        return [float(v) for v in values]
    if from_units == "degrees" and to_units == "radians":
        return [math.radians(float(v)) for v in values]
    if from_units == "radians" and to_units == "degrees":
        return [math.degrees(float(v)) for v in values]
    raise ValueError(f"bad units conversion: {from_units} -> {to_units}")


def infer_action_keys(obs: Any, joint_names) -> list[str]:
    if isinstance(obs, dict):
        for suffix in (".pos", ".position", ""):
            keys = [f"{j}{suffix}" for j in joint_names]
            if all(k in obs for k in keys):
                return keys
        for suffix in (".pos", ".position", ""):
            keys = [f"action.{j}{suffix}" for j in joint_names]
            if all(k in obs for k in keys):
                return keys
    return [f"{j}.pos" for j in joint_names]


def build_action(obs: Any, command_values: list[float], action_mode: str) -> Any:
    if action_mode == "vector":
        return {"action": command_values}
    if action_mode == "state":
        return {"state": command_values}
    keys = infer_action_keys(obs, ALL_JOINT_NAMES)
    return {k: float(v) for k, v in zip(keys, command_values)}


def apply_signs(values: list[float], signs: list[float]) -> list[float]:
    return [float(s) * float(v) for s, v in zip(signs, values)]


def parse_center(text: str) -> tuple[float, float, float]:
    vals = [float(x.strip()) for x in text.split(",")]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("center must be 'x,y,z'")
    return vals[0], vals[1], vals[2]



def make_auto_csv_path() -> Path:
    """Return ~/ros2_ws/src/mecha_lerobot/ik_csv/MMDD/HHMMSS.csv."""
    now = datetime.now()
    csv_dir = Path.home() / "ros2_ws" / "src" / "mecha_lerobot" / "ik_csv" / now.strftime("%m%d")
    return csv_dir / f"{now.strftime('%H%M%S')}.csv"


def resolve_csv_path(csv_arg: str | Path | None) -> Path:
    if csv_arg is None:
        return make_auto_csv_path()
    text = str(csv_arg).strip()
    if text == "" or text.lower() == "auto":
        return make_auto_csv_path()
    return Path(text).expanduser()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--follower-port", required=True)
    ap.add_argument("--follower-id", required=True)
    ap.add_argument("--fps", type=float, default=10.0) #30.0
    ap.add_argument("--udp-ip", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=5005)
    ap.add_argument("--csv", default="auto", help="CSV path, or auto for ~/ros2_ws/src/mecha_lerobot/ik_csv/MMDD/HHMMSS.csv")
    ap.add_argument("--fsync-every", type=int, default=1, help="flush+fsync CSV every N rows; 1 is safest for Ctrl+C / ros2 launch shutdown, 0 disables fsync")

    ap.add_argument("--trajectory", choices=["heart", "circle", "line", "csv"], default="heart")
    ap.add_argument("--target-csv", default=None, help="CSV with x,y,z columns when --trajectory csv")
    ap.add_argument("--n-points", type=int, default=600) # 200
    ap.add_argument("--scale", type=float, default=0.05)
    ap.add_argument("--center", type=parse_center, default=(0.18, 0.0, 0.12))
    ap.add_argument("--plane", choices=["YZ", "XZ", "XY"], default="YZ")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="Do not connect/send to real robot; useful for RViz test")

    ap.add_argument("--robot-action-units", choices=["degrees", "radians"], default="degrees")
    ap.add_argument("--observation-units", choices=["degrees", "radians"], default="degrees")
    ap.add_argument("--action-mode", choices=["named", "vector", "state"], default="named")
    ap.add_argument("--gripper", type=float, default=0.0, help="gripper command in radians before unit conversion")

    for j in ALL_JOINT_NAMES:
        ap.add_argument(f"--invert-{j.replace('_','-')}", action="store_true")
        ap.add_argument(f"--command-invert-{j.replace('_','-')}", action="store_true")

    args = ap.parse_args()
    args.csv = resolve_csv_path(args.csv)
    print(f"CSV path: {args.csv}", flush=True)

    stop_requested = False

    def request_stop(signum, frame_obj):
        nonlocal stop_requested
        stop_requested = True
        print(f"Received signal {signum}; saving CSV and stopping after current frame...", flush=True)

    # ros2 launch may stop ExecuteProcess children with SIGINT and/or SIGTERM.
    # Handle both so the CSV footer/final flush path still runs.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rviz_signs = [-1.0 if getattr(args, f"invert_{j}") else 1.0 for j in ALL_JOINT_NAMES]
    cmd_signs = [-1.0 if getattr(args, f"command_invert_{j}") else 1.0 for j in ALL_JOINT_NAMES]

    target_path = make_trajectory(
        args.trajectory,
        n_points=args.n_points,
        scale=args.scale,
        center=args.center,
        plane=args.plane,
        csv_path=args.target_csv,
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot = None
    if not args.dry_run:
        robot_cfg = SO101FollowerConfig(port=args.follower_port, id=args.follower_id)
        robot = make_robot_from_config(robot_cfg)
        robot.connect()
        print("Follower connected.", flush=True)
    else:
        print("DRY RUN: not connecting to follower hardware.", flush=True)

    args.csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["stamp", "frame", "ik_ok", "ik_error_m", "target_x", "target_y", "target_z", "actual_ee_x", "actual_ee_y", "actual_ee_z"]
    for name in ALL_JOINT_NAMES:
        fieldnames += [f"target_{name}_rad", f"actual_{name}_rad"]

    q_seed = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
    dt = 1.0 / args.fps
    frame = 0

    with open(args.csv, "w", newline="", buffering=1) as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        f.flush()
        if args.fsync_every > 0:
            os.fsync(f.fileno())

        try:
            while not stop_requested:
                for target_xyz in target_path:
                    if stop_requested:
                        break
                    t0 = time.perf_counter()
                    q_arm, ok, _, err = ik(target_xyz, q_init=q_seed)
                    q_seed = q_arm.copy()
                    target_rad = list(q_arm) + [float(args.gripper)]

                    if robot is not None:
                        obs = robot.get_observation()
                        command_rad = apply_signs(target_rad, cmd_signs)
                        command_values = convert_units(command_rad, "radians", args.robot_action_units)
                        action = build_action(obs, command_values, args.action_mode)
                        robot.send_action(action)
                        # Read once more so the CSV/RViz follows the physical arm as closely as this loop permits.
                        obs2 = robot.get_observation()
                        actual_raw = extract_positions(obs2)
                        actual_rad = convert_units(actual_raw, args.observation_units, "radians")
                    else:
                        actual_rad = target_rad[:]

                    actual_rviz_rad = apply_signs(actual_rad, rviz_signs)
                    target_rviz_rad = target_rad[:]
                    try:
                        actual_ee = ee_pos(actual_rviz_rad[:5])
                    except Exception:
                        actual_ee = np.array([float("nan")] * 3)

                    packet = {
                        "joint_names": list(ALL_JOINT_NAMES),
                        "actual_positions": actual_rviz_rad,
                        "target_positions": target_rviz_rad,
                        "target_xyz": [float(x) for x in target_xyz],
                        "target_path": target_path.tolist() if frame == 0 else None,
                        "stamp": time.time(),
                    }
                    sock.sendto(json.dumps(packet).encode("utf-8"), (args.udp_ip, args.udp_port))

                    row = {
                        "stamp": time.time(),
                        "frame": frame,
                        "ik_ok": int(bool(ok)),
                        "ik_error_m": float(err),
                        "target_x": float(target_xyz[0]),
                        "target_y": float(target_xyz[1]),
                        "target_z": float(target_xyz[2]),
                        "actual_ee_x": float(actual_ee[0]),
                        "actual_ee_y": float(actual_ee[1]),
                        "actual_ee_z": float(actual_ee[2]),
                    }
                    for name, tq, aq in zip(ALL_JOINT_NAMES, target_rad, actual_rviz_rad):
                        row[f"target_{name}_rad"] = float(tq)
                        row[f"actual_{name}_rad"] = float(aq)
                    writer.writerow(row)
                    f.flush()
                    if args.fsync_every > 0 and (frame % args.fsync_every == 0):
                        os.fsync(f.fileno())

                    if frame % max(1, int(args.fps)) == 0:
                        print(f"frame={frame} ok={ok} err_mm={err*1000:.2f} target={np.round(target_xyz,3).tolist()}", flush=True)
                    frame += 1
                    sleep_time = dt - (time.perf_counter() - t0)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                if not args.loop:
                    break
        except KeyboardInterrupt:
            stop_requested = True
            print("Interrupted.", flush=True)
        finally:
            try:
                f.flush()
                if args.fsync_every > 0:
                    os.fsync(f.fileno())
            except Exception as e:
                print(f"CSV final flush failed: {repr(e)}", flush=True)
            if robot is not None:
                try:
                    robot.disconnect()
                except Exception:
                    pass
            sock.close()
            print(f"CSV saved: {args.csv}", flush=True)


if __name__ == "__main__":
    main()
