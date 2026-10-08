#!/usr/bin/env python3
"""Capture high-rate closed-loop response telemetry for PID tuning."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
STATION = Path("/home/sonu/Downloads/grafito-canstepper-station")
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(STATION))

from absolute_alignment_sequence import (  # noqa: E402
    MOTION_TOLERANCE_DEG,
    align_node,
    absolute_error_deg,
    healthy,
    set_motion_limits,
)
from canstepper import CANStepperBus, Param  # noqa: E402
from canstepper.kinematics import eval_trapezoid, plan_trapezoid  # noqa: E402
from canstepper.protocol import Tel  # noqa: E402
import pid_response_plotter as plotter  # noqa: E402

REFERENCE_PATH = SCRIPT_DIR / "alignment_reference.json"
TELEMETRY_HZ = 100
MOVE_DEG = -360.0

SNAPSHOT_PARAMS = [
    Param.PID_KP,
    Param.PID_KI,
    Param.PID_KD,
    Param.PID_TOLERANCE,
    Param.MAX_SPEED,
    Param.ACCELERATION,
    Param.CL_MAX_SPEED,
    Param.CL_MAX_ACCEL,
    Param.CLOSED_LOOP,
    Param.MICROSTEPS,
    Param.FAST_RATE_HZ,
    Param.SLOW_RATE_HZ,
]


def parse_gain(text: str) -> tuple[float, float, float]:
    values = [float(value.strip()) for value in text.split(",")]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError("gain must be Kp,Ki,Kd")
    return values[0], values[1], values[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--nodes", type=int, nargs="+", default=[2, 3])
    parser.add_argument("--speed", type=float, default=720.0)
    parser.add_argument("--accel", type=float)
    parser.add_argument("--gain", type=parse_gain, action="append")
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--label", default="baseline")
    return parser.parse_args()


def snapshot(node) -> dict[Param, float | int]:
    return {param: node.get_param(param) for param in SNAPSHOT_PARAMS}


def restore(node, values: dict[Param, float | int]) -> None:
    for param, value in values.items():
        try:
            node.set_param(param, value)
        except Exception:
            pass


def run_metrics(samples: list[dict[str, object]]) -> dict[str, float]:
    t = np.asarray([float(row["time_s"]) for row in samples])
    tracking = np.asarray([float(row["tracking_error_deg"]) for row in samples])
    velocity = np.asarray([float(row["velocity_deg_s"]) for row in samples])
    position = np.asarray([float(row["position_deg"]) for row in samples])
    target = float(samples[0]["target_deg"])
    start = float(samples[0]["start_position_deg"])
    direction = 1.0 if target >= start else -1.0
    travel = direction * (position - start)
    commanded = abs(target - start)
    overshoot = max(0.0, float(np.max(travel)) - commanded)
    planned = float(samples[0]["planned_time_s"])
    settle = float(samples[0]["settle_time_s"])
    duration = max(1e-9, float(t[-1] - t[0]))
    return {
        "planned_time_s": planned,
        "settle_time_s": settle,
        "settle_delay_s": settle - planned,
        "final_error_deg": target - float(position[-1]),
        "peak_tracking_error_deg": float(np.max(np.abs(tracking))),
        "rms_tracking_error_deg": float(np.sqrt(np.mean(tracking * tracking))),
        "iae_deg_s": float(np.trapz(np.abs(tracking), t)),
        "peak_velocity_deg_s": float(np.max(np.abs(velocity))),
        "overshoot_deg": overshoot,
        "samples": float(len(samples)),
        "sample_rate_hz": float((len(samples) - 1) / duration),
    }


def main() -> int:
    args = parse_args()
    if args.speed <= 0 or args.repetitions < 1:
        raise SystemExit("speed and repetitions must be positive")
    if any(node_id not in (2, 3) for node_id in args.nodes):
        raise SystemExit("this rig supports node IDs 2 and 3")

    accel = args.accel if args.accel is not None else args.speed * 4.0
    gains = args.gain or [(12.0, 0.3, 0.1)]
    reference_data = json.loads(REFERENCE_PATH.read_text(encoding="utf-8"))
    cpr = int(reference_data["encoder_counts_per_revolution"])
    max_correction = float(reference_data["max_automatic_correction_deg"])
    references = {
        int(node_id): int(values["aligned_raw_count"])
        for node_id, values in reference_data["nodes"].items()
    }

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = SCRIPT_DIR / "pid_results" / f"{stamp}_{args.label}"
    output_dir.mkdir(parents=True, exist_ok=True)
    telemetry_path = output_dir / "telemetry.csv"
    metrics_path = output_dir / "metrics.csv"
    rows: list[dict[str, object]] = []
    metric_rows: list[dict[str, object]] = []
    active: dict[str, object] | None = None
    active_lock = threading.Lock()
    snapshots: dict[int, dict[Param, float | int]] = {}
    run_indices = {2: 0, 3: 0}
    failure = ""

    bus = CANStepperBus.serial(args.port)
    time.sleep(2.5)
    nodes = {2: bus.node(2), 3: bus.node(3)}

    def on_pid(frame) -> None:
        nonlocal active
        with active_lock:
            context = active
            if context is None or int(context["node_id"]) != frame.node_id:
                return
            node = nodes[frame.node_id]
            state = node.state
            if (
                state.position_deg is None
                or state.velocity_deg_s is None
                or state.pid is None
            ):
                return
            now = time.monotonic()
            elapsed = now - float(context["started"])
            distance = float(context["target"]) - float(context["start"])
            along, planned_velocity = eval_trapezoid(
                distance,
                float(context["speed"]),
                float(context["accel"]),
                elapsed,
            )
            planned_position = float(context["start"]) + along
            position = float(state.position_deg)
            context["samples"].append(
                {
                    "run_id": context["run_id"],
                    "node_id": frame.node_id,
                    "speed_deg_s": context["speed"],
                    "accel_deg_s2": context["accel"],
                    "kp": context["kp"],
                    "ki": context["ki"],
                    "kd": context["kd"],
                    "time_s": elapsed,
                    "start_position_deg": context["start"],
                    "target_deg": context["target"],
                    "planned_position_deg": planned_position,
                    "position_deg": position,
                    "planned_velocity_deg_s": planned_velocity,
                    "velocity_deg_s": float(state.velocity_deg_s),
                    "pid_output_deg_s": float(state.pid.output_deg_s),
                    "target_error_deg": float(context["target"]) - position,
                    "tracking_error_deg": planned_position - position,
                    "pid_state": int(state.pid.state),
                    "planned_time_s": context["planned_time"],
                    "settle_time_s": 0.0,
                }
            )

    bus.subscribe(None, Tel.PID_STATUS, on_pid)

    try:
        for node_id, node in nodes.items():
            snapshots[node_id] = snapshot(node)
            node.disable()
            if node.get_param(Param.ZERO_ON_BOOT) != 0:
                raise RuntimeError(f"node {node_id} zero_on_boot must remain 0")
            node.set_param(Param.MICROSTEPS, 8)
            node.set_param(Param.CLOSED_LOOP, 1)
            node.set_param(Param.PID_TOLERANCE, MOTION_TOLERANCE_DEG)
            node.set_report_rates(TELEMETRY_HZ, 2)
            healthy(node)

        base_targets: dict[int, float] = {}
        for node_id in (2, 3):
            base_targets[node_id] = align_node(
                nodes[node_id], references[node_id], cpr, max_correction
            )

        for kp, ki, kd in gains:
            for repetition in range(1, args.repetitions + 1):
                for node_id in args.nodes:
                    node = nodes[node_id]
                    other = nodes[3 if node_id == 2 else 2]
                    if abs(absolute_error_deg(other, references[other.node_id], cpr)) > MOTION_TOLERANCE_DEG:
                        raise RuntimeError(
                            f"clearance node {other.node_id} is not aligned"
                        )
                    node.set_pid(kp, ki, kd, tolerance_deg=MOTION_TOLERANCE_DEG)
                    node.set_param(Param.MAX_SPEED, args.speed)
                    node.set_param(Param.ACCELERATION, accel)
                    node.set_param(Param.CL_MAX_SPEED, args.speed)
                    node.set_param(Param.CL_MAX_ACCEL, accel)
                    healthy(node)

                    run_indices[node_id] += 1
                    target = base_targets[node_id] + MOVE_DEG * run_indices[node_id]
                    start = node.get_position()
                    _, _, _, _, planned_time = plan_trapezoid(
                        target - start, args.speed, accel
                    )
                    run_id = (
                        f"n{node_id}_s{args.speed:g}_kp{kp:g}_ki{ki:g}_"
                        f"kd{kd:g}_r{repetition}"
                    )
                    context: dict[str, object] = {
                        "run_id": run_id,
                        "node_id": node_id,
                        "speed": args.speed,
                        "accel": accel,
                        "kp": kp,
                        "ki": ki,
                        "kd": kd,
                        "start": start,
                        "target": target,
                        "planned_time": planned_time,
                        "started": time.monotonic(),
                        "samples": [],
                    }
                    with active_lock:
                        active = context
                    print(
                        f"RUN {run_id} start={start:+.6f} target={target:+.6f}",
                        flush=True,
                    )
                    node.move_to(
                        target,
                        blocking=True,
                        timeout=max(8.0, planned_time * 4.0 + 3.0),
                    )
                    settle_time = time.monotonic() - float(context["started"])
                    time.sleep(0.15)
                    with active_lock:
                        active = None
                    samples = list(context["samples"])
                    if not samples:
                        raise RuntimeError(f"no telemetry captured for {run_id}")
                    for sample in samples:
                        sample["settle_time_s"] = settle_time
                    final_error = target - node.get_position()
                    if abs(final_error) > MOTION_TOLERANCE_DEG:
                        raise RuntimeError(
                            f"{run_id} final error {final_error:+.6f} deg"
                        )
                    if abs(absolute_error_deg(node, references[node_id], cpr)) > MOTION_TOLERANCE_DEG:
                        raise RuntimeError(f"{run_id} did not return to slot reference")
                    rows.extend(samples)
                    result = run_metrics(samples)
                    result_row: dict[str, object] = {
                        "run_id": run_id,
                        "node_id": node_id,
                        "speed_deg_s": args.speed,
                        "accel_deg_s2": accel,
                        "kp": kp,
                        "ki": ki,
                        "kd": kd,
                        **result,
                    }
                    metric_rows.append(result_row)
                    print(
                        f"RESULT {run_id} settle={result['settle_time_s']:.3f}s "
                        f"delay={result['settle_delay_s']:+.3f}s "
                        f"final={result['final_error_deg']:+.3f}deg "
                        f"rms_track={result['rms_tracking_error_deg']:.3f}deg",
                        flush=True,
                    )

    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
        print(f"ABORT {failure}", flush=True)
        try:
            bus.estop_all()
        except Exception:
            pass
    finally:
        with active_lock:
            active = None
        for _ in range(2):
            for node in nodes.values():
                try:
                    node.disable()
                except Exception:
                    pass
            time.sleep(0.25)
        for node_id, values in snapshots.items():
            restore(nodes[node_id], values)
        try:
            print(
                "FINAL_DISABLED",
                {node_id: not node.get_status().enabled for node_id, node in nodes.items()},
                flush=True,
            )
        except Exception:
            pass
        bus.close()

    telemetry_fields = [
        "run_id", "node_id", "speed_deg_s", "accel_deg_s2", "kp", "ki", "kd",
        "time_s", "start_position_deg", "target_deg", "planned_position_deg",
        "position_deg", "planned_velocity_deg_s", "velocity_deg_s",
        "pid_output_deg_s", "target_error_deg", "tracking_error_deg", "pid_state",
        "planned_time_s", "settle_time_s",
    ]
    with telemetry_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=telemetry_fields)
        writer.writeheader()
        writer.writerows(rows)

    metric_fields = [
        "run_id", "node_id", "speed_deg_s", "accel_deg_s2", "kp", "ki", "kd",
        "planned_time_s", "settle_time_s", "settle_delay_s", "final_error_deg",
        "peak_tracking_error_deg", "rms_tracking_error_deg", "iae_deg_s",
        "peak_velocity_deg_s", "overshoot_deg", "samples", "sample_rate_hz",
    ]
    with metrics_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=metric_fields)
        writer.writeheader()
        writer.writerows(metric_rows)

    if rows:
        runs = plotter.load_runs(telemetry_path)
        for run_id, run_rows in runs.items():
            print(f"PLOT {plotter.plot_run(run_id, run_rows, output_dir)}", flush=True)
        print(
            f"PLOT {plotter.plot_comparison(runs, output_dir, args.label)}",
            flush=True,
        )
    print(f"TELEMETRY {telemetry_path}", flush=True)
    print(f"METRICS {metrics_path}", flush=True)
    print("FAILED" if failure else "PASSED", flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
