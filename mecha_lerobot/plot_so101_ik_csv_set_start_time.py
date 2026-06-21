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


def plot_target_actual(
    df: pd.DataFrame,
    x: pd.Series,
    output_path: Path,
    rmse_start_time: float,
    degrees: bool = True,
    show_rmse_start: bool = False,
):
    scale = 180.0 / 3.141592653589793 if degrees else 1.0
    unit = "deg" if degrees else "rad"

    fig, axes = plt.subplots(len(JOINTS), 1, figsize=(12, 14), sharex=True)

    for i, (ax, joint) in enumerate(zip(axes, JOINTS)):
        target_col = f"target_{joint}_rad"
        actual_col = f"actual_{joint}_rad"

        if target_col not in df.columns:
            raise KeyError(f"Missing column: {target_col}")
        if actual_col not in df.columns:
            raise KeyError(f"Missing column: {actual_col}")

        target = df[target_col] * scale
        actual = df[actual_col] * scale

        ax.plot(x, target, label="target")
        ax.plot(x, actual, label="actual", linestyle="--")

        if show_rmse_start:
            ax.axvline(
                rmse_start_time,
                color="red",
                linestyle=":",
                alpha=0.7,
                label="RMSE start" if i == 0 else None,
            )

        ax.set_ylabel(f"{joint}\n({unit})")
        ax.grid(True, alpha=0.3)

        if i == 0:
            ax.legend(loc="upper right")

    axes[-1].set_xlabel("time (s)")
    fig.suptitle("target vs actual motor angles")

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def calculate_rmse(
    df: pd.DataFrame,
    x: pd.Series,
    rmse_start_time: float,
    degrees: bool = True,
) -> pd.DataFrame:
    scale = 180.0 / 3.141592653589793 if degrees else 1.0
    unit = "deg" if degrees else "rad"

    valid = x >= rmse_start_time

    if valid.sum() == 0:
        raise ValueError(
            f"No samples after rmse_start_time={rmse_start_time}s. "
            f"Check timestamp or reduce rmse_start_time."
        )

    rmse_rows = []

    for joint in JOINTS:
        target_col = f"target_{joint}_rad"
        actual_col = f"actual_{joint}_rad"

        if target_col not in df.columns:
            raise KeyError(f"Missing column: {target_col}")
        if actual_col not in df.columns:
            raise KeyError(f"Missing column: {actual_col}")

        target = df[target_col] * scale
        actual = df[actual_col] * scale

        # 전체 구간 RMSE
        raw_error = target - actual
        raw_rmse = (raw_error**2).mean() ** 0.5

        # 초기 구간 제외 RMSE
        settled_error = target[valid] - actual[valid]
        settled_rmse = (settled_error**2).mean() ** 0.5

        rmse_rows.append(
            {
                "joint": joint,
                f"raw_rmse_{unit}": raw_rmse,
                f"settled_rmse_{unit}": settled_rmse,
                "rmse_start_time_s": rmse_start_time,
                "num_total_samples": len(df),
                "num_rmse_samples": int(valid.sum()),
            }
        )

    return pd.DataFrame(rmse_rows)


def main():
    csv_path = Path("/home/kjy/ros2_ws/src/mecha_lerobot/ik_csv/0621/142146.csv")

    # RMSE start 표시 없는 plot
    output_path = Path("/home/kjy/ros2_ws/src/mecha_lerobot/ik_csv/0621/142146.png")

    # RMSE start 표시 있는 plot
    output_with_rmse_start_path = output_path.with_name(
        output_path.stem + "_rmse_start.png"
    )

    # RMSE CSV
    rmse_csv_path = output_path.with_name(output_path.stem + "_rmse.csv")

    degrees = True

    # 초기 settling 구간 제외 시간
    rmse_start_time = 3.0  # sec

    df = pd.read_csv(csv_path)

    if "stamp" in df:
        x = df["stamp"] - df["stamp"].iloc[0]
    else:
        x = pd.Series(df.index, index=df.index)

    # plot 1: RMSE start 표시 없는 버전
    plot_target_actual(
        df=df,
        x=x,
        output_path=output_path,
        rmse_start_time=rmse_start_time,
        degrees=degrees,
        show_rmse_start=False,
    )

    # plot 2: RMSE start 표시 있는 버전
    plot_target_actual(
        df=df,
        x=x,
        output_path=output_with_rmse_start_path,
        rmse_start_time=rmse_start_time,
        degrees=degrees,
        show_rmse_start=True,
    )

    # RMSE 계산
    rmse_df = calculate_rmse(
        df=df,
        x=x,
        rmse_start_time=rmse_start_time,
        degrees=degrees,
    )

    rmse_df.to_csv(rmse_csv_path, index=False)

    print(f"saved plot without RMSE start: {output_path}")
    print(f"saved plot with RMSE start:    {output_with_rmse_start_path}")
    print(f"saved rmse:                    {rmse_csv_path}")
    print()
    print(rmse_df)


if __name__ == "__main__":
    main()
