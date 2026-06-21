#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def main():
    csv_path = Path("/home/kjy/ros2_ws/src/mecha_lerobot/ik_csv/0621/141643.csv")
    output_path = Path("/home/kjy/ros2_ws/src/mecha_lerobot/ik_csv/0621/141643.png")
    rmse_csv_path = output_path.with_name(output_path.stem + "_rmse.csv")

    degrees = True

    df = pd.read_csv(csv_path)

    x = df["stamp"] - df["stamp"].iloc[0] if "stamp" in df else df.index

    scale = 180.0 / 3.141592653589793 if degrees else 1.0
    unit = "deg" if degrees else "rad"

    rmse_rows = []

    fig, axes = plt.subplots(len(JOINTS), 1, figsize=(12, 14), sharex=True)

    for i, (ax, joint) in enumerate(zip(axes, JOINTS)):
        target_col = f"target_{joint}_rad"
        actual_col = f"actual_{joint}_rad"

        target = df[target_col] * scale
        actual = df[actual_col] * scale

        ax.plot(x, target, label="target")
        ax.plot(x, actual, label="actual", linestyle="--")

        ax.set_ylabel(f"{joint}\n({unit})")
        ax.grid(True, alpha=0.3)

        # legend는 첫 번째 figure에만 표시
        if i == 0:
            ax.legend(loc="upper right")

        # RMSE 계산
        rmse = ((target - actual) ** 2).mean() ** 0.5
        rmse_rows.append({
            "joint": joint,
            "rmse_deg": rmse,
        })

    axes[-1].set_xlabel("time (s)")
    fig.suptitle("SO-101 target vs actual motor angles")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)

    rmse_df = pd.DataFrame(rmse_rows)
    rmse_df.to_csv(rmse_csv_path, index=False)

    print(f"saved plot: {output_path}")
    print(f"saved rmse: {rmse_csv_path}")


if __name__ == "__main__":
    main()
