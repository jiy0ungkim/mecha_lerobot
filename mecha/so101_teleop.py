#!/usr/bin/env python3

import argparse
import json
import math
import socket
import time


from lerobot.robots import make_robot_from_config
from lerobot.teleoperators import make_teleoperator_from_config


try:
    from lerobot.robots.so_follower import SO101FollowerConfig
except Exception:
    try:
        from lerobot.robots.so101_follower import SO101FollowerConfig
    except Exception:
        from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig


try:
    from lerobot.teleoperators.so_leader import SO101LeaderConfig
except Exception:
    try:
        from lerobot.teleoperators.so101_leader import SO101LeaderConfig
    except Exception:
        from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def extract_from_named_dict(data, joint_names, prefixes):
    if not isinstance(data, dict):
        return None

    values = []

    for joint_name in joint_names:
        candidates = []

        for prefix in prefixes:
            candidates.extend([
                f"{prefix}{joint_name}",
                f"{prefix}{joint_name}.pos",
                f"{prefix}{joint_name}.position",
            ])

        found = False

        for key in candidates:
            if key in data:
                values.append(float(data[key]))
                found = True
                break

        if not found:
            return None

    return values


def extract_positions(obs, action):
    values = extract_from_named_dict(obs, JOINT_NAMES, prefixes=["", "observation."])
    if values is not None:
        return values

    if isinstance(obs, dict) and "observation.state" in obs:
        return [float(x) for x in list(obs["observation.state"])[:6]]

    if isinstance(obs, dict) and "state" in obs:
        return [float(x) for x in list(obs["state"])[:6]]

    values = extract_from_named_dict(action, JOINT_NAMES, prefixes=["", "action."])
    if values is not None:
        return values

    if isinstance(action, dict) and "action" in action:
        return [float(x) for x in list(action["action"])[:6]]

    raise RuntimeError(
        "Cannot extract joint positions. "
        f"obs keys={list(obs.keys()) if isinstance(obs, dict) else type(obs)}, "
        f"action keys={list(action.keys()) if isinstance(action, dict) else type(action)}"
    )


def maybe_deg_to_rad(values, enabled):
    if not enabled:
        return values

    return [math.radians(v) for v in values]


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-id", required=True)
    parser.add_argument("--leader-id", required=True)

    parser.add_argument("--fps", type=float, required=True)

    parser.add_argument("--udp-ip", required=True)
    parser.add_argument("--udp-port", type=int, required=True)

    parser.add_argument("--degrees-to-radians", action="store_true")

    parser.add_argument("--invert-shoulder-pan", action="store_true")
    parser.add_argument("--invert-shoulder-lift", action="store_true")
    parser.add_argument("--invert-elbow-flex", action="store_true")
    parser.add_argument("--invert-wrist-flex", action="store_true")
    parser.add_argument("--invert-wrist-roll", action="store_true")
    parser.add_argument("--invert-gripper", action="store_true")

    args = parser.parse_args()

    print("==== SO101 UDP Teleop Args ====", flush=True)
    print(f"follower_port      = {args.follower_port}", flush=True)
    print(f"leader_port        = {args.leader_port}", flush=True)
    print(f"follower_id        = {args.follower_id}", flush=True)
    print(f"leader_id          = {args.leader_id}", flush=True)
    print(f"fps                = {args.fps}", flush=True)
    print(f"udp_ip             = {args.udp_ip}", flush=True)
    print(f"udp_port           = {args.udp_port}", flush=True)
    print(f"degrees_to_radians = {args.degrees_to_radians}", flush=True)
    print("================================", flush=True)

    signs = [
        -1.0 if args.invert_shoulder_pan else 1.0,
        -1.0 if args.invert_shoulder_lift else 1.0,
        -1.0 if args.invert_elbow_flex else 1.0,
        -1.0 if args.invert_wrist_flex else 1.0,
        -1.0 if args.invert_wrist_roll else 1.0,
        -1.0 if args.invert_gripper else 1.0,
    ]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    robot_cfg = SO101FollowerConfig(
        port=args.follower_port,
        id=args.follower_id,
    )

    teleop_cfg = SO101LeaderConfig(
        port=args.leader_port,
        id=args.leader_id,
    )

    robot = make_robot_from_config(robot_cfg)
    teleop = make_teleoperator_from_config(teleop_cfg)

    print("Connecting leader and follower...", flush=True)
    teleop.connect()
    robot.connect()
    print("Connected.", flush=True)
    print(f"Sending joint states to UDP {args.udp_ip}:{args.udp_port}", flush=True)

    dt = 1.0 / args.fps

    try:
        while True:
            start = time.perf_counter()

            obs = robot.get_observation()
            action = teleop.get_action()
            robot.send_action(action)

            positions = extract_positions(obs, action)
            positions = maybe_deg_to_rad(positions, args.degrees_to_radians)
            positions = [sign * float(value) for sign, value in zip(signs, positions)]

            packet = {
                "joint_names": JOINT_NAMES,
                "positions": positions,
                "stamp": time.time(),
            }

            sock.sendto(
                json.dumps(packet).encode("utf-8"),
                (args.udp_ip, args.udp_port),
            )

            elapsed = time.perf_counter() - start
            sleep_time = dt - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Interrupted.", flush=True)

    finally:
        print("Disconnecting...", flush=True)

        try:
            teleop.disconnect()
        except Exception:
            pass

        try:
            robot.disconnect()
        except Exception:
            pass

        try:
            sock.close()
        except Exception:
            pass

        print("Done.", flush=True)


if __name__ == "__main__":
    main()