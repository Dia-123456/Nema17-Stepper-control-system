#!/usr/bin/env python3
"""Create synchronized PID response plots from a telemetry CSV."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load_runs(path: Path) -> dict[str, list[dict[str, str]]]:
    runs: dict[str, list[dict[str, str]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            runs[row["run_id"]].append(row)
    if not runs:
        raise ValueError(f"no telemetry rows in {path}")
    return dict(runs)


def values(rows: list[dict[str, str]], key: str) -> np.ndarray:
    return np.asarray([float(row[key]) for row in rows], dtype=float)


def safe_name(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in value)


def metrics(rows: list[dict[str, str]]) -> dict[str, float]:
    t = values(rows, "time_s")
    tracking = values(rows, "tracking_error_deg")
    velocity = values(rows, "velocity_deg_s")
    position = values(rows, "position_deg")
    target = values(rows, "target_deg")
    plan_total = float(rows[0]["planned_time_s"])
    settle = float(rows[0]["settle_time_s"])
    direction = 1.0 if target[-1] >= position[0] else -1.0
    travel = direction * (position - position[0])
    commanded = abs(target[-1] - position[0])
    overshoot = max(0.0, float(np.max(travel)) - commanded)
    return {
        "planned_time_s": plan_total,
        "settle_time_s": settle,
        "settle_delay_s": settle - plan_total,
        "final_error_deg": float(target[-1] - position[-1]),
        "peak_tracking_error_deg": float(np.max(np.abs(tracking))),
        "rms_tracking_error_deg": float(np.sqrt(np.mean(tracking * tracking))),
        "iae_deg_s": float(np.trapz(np.abs(tracking), t)),
        "peak_velocity_deg_s": float(np.max(np.abs(velocity))),
        "overshoot_deg": overshoot,
    }


def plot_run(run_id: str, rows: list[dict[str, str]], output_dir: Path) -> Path:
    rows = sorted(rows, key=lambda row: float(row["time_s"]))
    t = values(rows, "time_s")
    position = values(rows, "position_deg")
    planned_position = values(rows, "planned_position_deg")
    target = values(rows, "target_deg")
    velocity = values(rows, "velocity_deg_s")
    planned_velocity = values(rows, "planned_velocity_deg_s")
    pid_output = values(rows, "pid_output_deg_s")
    target_error = values(rows, "target_error_deg")
    tracking_error = values(rows, "tracking_error_deg")
    result = metrics(rows)

    first = rows[0]
    kp, ki, kd = (float(first[key]) for key in ("kp", "ki", "kd"))
    node = int(first["node_id"])
    speed = float(first["speed_deg_s"])
    accel = float(first["accel_deg_s2"])

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(
        f"Node {node} PID response — {speed:g}°/s, {accel:g}°/s²  |  "
        f"Kp={kp:g}, Ki={ki:g}, Kd={kd:g}",
        fontsize=14,
    )

    axes[0].plot(t, planned_position, "--", color="tab:orange", label="planned position")
    axes[0].plot(t, position, color="tab:blue", linewidth=1.8, label="encoder position")
    axes[0].plot(t, target, ":", color="black", alpha=0.65, label="final target")
    axes[0].set_ylabel("Position (deg)")
    axes[0].legend(loc="best")

    axes[1].plot(t, planned_velocity, "--", color="tab:orange", label="planned velocity")
    axes[1].plot(t, velocity, color="tab:green", linewidth=1.6, label="encoder velocity")
    axes[1].plot(t, pid_output, color="tab:red", alpha=0.75, label="controller output")
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylabel("Velocity (deg/s)")
    axes[1].legend(loc="best")

    axes[2].plot(t, tracking_error, color="tab:purple", linewidth=1.6,
                 label="trajectory tracking error")
    axes[2].axhline(0.0, color="black", linewidth=0.7)
    axes[2].set_ylabel("Tracking error (deg)")
    axes[2].set_xlabel("Time from command (s)")
    axes[2].legend(loc="upper left")
    target_axis = axes[2].twinx()
    target_axis.plot(t, target_error, color="tab:gray", alpha=0.28,
                     label="distance to final target")
    target_axis.set_ylabel("Target error (deg)", color="tab:gray")
    target_axis.tick_params(axis="y", colors="tab:gray")

    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.axvline(result["planned_time_s"], color="tab:orange", linestyle=":",
                     alpha=0.8, label="planned end")
        axis.axvline(result["settle_time_s"], color="tab:green", linestyle=":",
                     alpha=0.8, label="settled")

    metric_text = (
        f"settle: {result['settle_time_s']:.3f} s\n"
        f"planned: {result['planned_time_s']:.3f} s\n"
        f"delay: {result['settle_delay_s']:+.3f} s\n"
        f"final error: {result['final_error_deg']:+.3f}°\n"
        f"peak track err: {result['peak_tracking_error_deg']:.3f}°\n"
        f"RMS track err: {result['rms_tracking_error_deg']:.3f}°\n"
        f"IAE: {result['iae_deg_s']:.3f} deg·s\n"
        f"overshoot: {result['overshoot_deg']:.3f}°"
    )
    axes[0].text(
        0.985, 0.04, metric_text, transform=axes[0].transAxes,
        ha="right", va="bottom", family="monospace", fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.88},
    )

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    output = output_dir / f"{safe_name(run_id)}.png"
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return output


def plot_comparison(
    runs: dict[str, list[dict[str, str]]], output_dir: Path, stem: str
) -> Path:
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    for run_id, rows in runs.items():
        rows = sorted(rows, key=lambda row: float(row["time_s"]))
        t = values(rows, "time_s")
        first = rows[0]
        label = (
            f"N{first['node_id']} Kp={float(first['kp']):g} "
            f"Ki={float(first['ki']):g} Kd={float(first['kd']):g}"
        )
        axes[0].plot(t, values(rows, "tracking_error_deg"), label=label)
        axes[1].plot(t, values(rows, "velocity_deg_s"), label=label)
    axes[0].set_ylabel("Tracking error (deg)")
    axes[1].set_ylabel("Velocity (deg/s)")
    axes[1].set_xlabel("Time from command (s)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.axhline(0.0, color="black", linewidth=0.7)
        axis.legend(loc="best", fontsize=8)
    fig.suptitle("PID response comparison")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    output = output_dir / f"{safe_name(stem)}_comparison.png"
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    output_dir = args.output_dir or args.csv.with_suffix("").with_name(
        args.csv.stem + "_plots"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    runs = load_runs(args.csv)
    for run_id, rows in runs.items():
        print(plot_run(run_id, rows, output_dir))
    print(plot_comparison(runs, output_dir, args.csv.stem))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
