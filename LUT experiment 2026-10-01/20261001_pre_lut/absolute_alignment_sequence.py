#!/usr/bin/env python3
"""Absolute-encoder startup alignment and alternating precision sequence.

This program never calls SET_ZERO.  It uses one-time taught raw encoder
references, corrects only a small bounded startup offset, and then alternates
one clockwise revolution per node with immediate firmware-event handoffs.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

STATION = Path("/home/sonu/Downloads/grafito-canstepper-station")
sys.path.insert(0, str(STATION))

from canstepper import CANStepperBus, Param  # noqa: E402

PORT = "/dev/ttyACM0"
REFERENCE_PATH = Path(__file__).with_name("alignment_reference.json")
MOTION_TOLERANCE_DEG = 0.5
STARTUP_ALIGNMENT_SPEED = 30.0
DEFAULT_SPEED = 360.0
DEFAULT_CYCLES = 5
MAX_SINGLE_DISC_RECOVERY_DEG = 180.0
# Runtime-only gains selected from the recorded 360 and 720 deg/s sweeps.
# These are intentionally node-specific: node 3 was not repeatable with the
# node 2 tune.  No SAVE_CONFIG command is issued anywhere in this program.
PID_GAINS = {
    2: (12.0, 0.05, 0.35),
    3: (10.0, 0.0, 0.4),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES)
    parser.add_argument("--align-only", action="store_true")
    return parser.parse_args()


def set_motion_limits(node, speed: float) -> None:
    accel = speed * 4.0
    node.set_param(Param.MAX_SPEED, speed)
    node.set_param(Param.ACCELERATION, accel)
    node.set_param(Param.CL_MAX_SPEED, speed)
    node.set_param(Param.CL_MAX_ACCEL, accel)


def healthy(node) -> None:
    status = node.get_status()
    driver = node.get_driver_status()
    if status.fault.value != 0 or not status.encoder_ok:
        raise RuntimeError(f"node {node.node_id} health failure: {status}")
    if not driver.uart_ok or driver.any_short or driver.thermal_fault:
        raise RuntimeError(f"node {node.node_id} driver failure: {driver}")


def signed_modular_delta(reference: int, current: int, cpr: int) -> int:
    """Shortest signed count delta from current angle to reference angle."""
    return (reference - (current % cpr) + cpr // 2) % cpr - cpr // 2


def alignment_target(node, reference: int, cpr: int) -> tuple[float, float, int]:
    current = node.get_encoder_counts()
    delta = signed_modular_delta(reference, current, cpr)
    return (current + delta) * 360.0 / cpr, delta * 360.0 / cpr, current


def absolute_error_deg(node, reference: int, cpr: int) -> float:
    current = node.get_encoder_counts()
    return signed_modular_delta(reference, current, cpr) * 360.0 / cpr


def align_node(node, reference: int, cpr: int, max_correction: float) -> float:
    node.enable()
    time.sleep(0.4)
    target, correction, current = alignment_target(node, reference, cpr)
    print(
        f"AUTO_ALIGN node={node.node_id} counts={current} "
        f"correction={correction:+.6f} target={target:+.6f}",
        flush=True,
    )
    if abs(correction) > max_correction:
        raise RuntimeError(
            f"node {node.node_id} startup offset {correction:+.6f} deg exceeds "
            f"safe automatic limit {max_correction:.3f} deg"
        )

    set_motion_limits(node, STARTUP_ALIGNMENT_SPEED)
    timeout = max(6.0, abs(correction) / STARTUP_ALIGNMENT_SPEED * 2.0 + 3.0)
    node.move_to(target, blocking=True, timeout=timeout)
    residual = absolute_error_deg(node, reference, cpr)
    print(
        f"AUTO_ALIGNED node={node.node_id} residual={residual:+.6f}",
        flush=True,
    )
    if abs(residual) > MOTION_TOLERANCE_DEG:
        raise RuntimeError(
            f"node {node.node_id} automatic alignment residual "
            f"{residual:+.6f} deg exceeds {MOTION_TOLERANCE_DEG:.3f} deg"
        )
    healthy(node)
    return target


def move_one_revolution(node, desired: float, speed: float) -> dict[str, float]:
    set_motion_limits(node, speed)
    started = time.monotonic()
    node.move_to(desired, blocking=True, timeout=max(6.0, 360.0 / speed * 3.0 + 3.0))
    final = node.get_position()
    error = final - desired
    if abs(error) > MOTION_TOLERANCE_DEG:
        raise RuntimeError(
            f"node {node.node_id} return error {error:+.6f} deg exceeds "
            f"{MOTION_TOLERANCE_DEG:.3f} deg"
        )
    return {
        "desired_deg": desired,
        "position_deg": final,
        "error_deg": error,
        "elapsed_s": time.monotonic() - started,
    }


def main() -> int:
    args = parse_args()
    if args.speed <= 0 or args.cycles < 1:
        raise SystemExit("speed and cycles must be positive")

    reference_data = json.loads(REFERENCE_PATH.read_text(encoding="utf-8"))
    cpr = int(reference_data["encoder_counts_per_revolution"])
    max_correction = float(reference_data["max_automatic_correction_deg"])
    references = {
        int(node_id): int(values["aligned_raw_count"])
        for node_id, values in reference_data["nodes"].items()
    }

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(__file__).resolve().parent
    csv_path = output_dir / f"absolute_sequence_{stamp}.csv"
    summary_path = output_dir / f"absolute_sequence_{stamp}.txt"
    rows: list[dict[str, object]] = []
    failure = ""

    print(
        f"ABSOLUTE_SEQUENCE speed={args.speed:g} cycles={args.cycles} "
        f"align_only={args.align_only}",
        flush=True,
    )
    bus = CANStepperBus.serial(args.port)
    time.sleep(2.5)
    nodes = {2: bus.node(2), 3: bus.node(3)}

    try:
        for node_id, node in nodes.items():
            node.disable()
            if node.get_param(Param.ZERO_ON_BOOT) != 0:
                raise RuntimeError(f"node {node_id} zero_on_boot must remain 0")
            if node.get_param(Param.ENABLE_ON_BOOT) != 0:
                raise RuntimeError(f"node {node_id} enable_on_boot must remain 0")
            node.set_param(Param.MICROSTEPS, 32)
            node.set_param(Param.CLOSED_LOOP, 1)
            node.set_param(Param.INVERT_DIR, 0)
            kp, ki, kd = PID_GAINS[node_id]
            node.set_pid(kp, ki, kd, tolerance_deg=MOTION_TOLERANCE_DEG)
            print(
                f"RUNTIME_PID node={node_id} kp={kp:g} ki={ki:g} kd={kd:g} "
                f"tolerance={MOTION_TOLERANCE_DEG:g}",
                flush=True,
            )
            healthy(node)

        time.sleep(0.3)
        initial_offsets: dict[int, float] = {}
        for node_id in (2, 3):
            _target, correction, current = alignment_target(
                nodes[node_id], references[node_id], cpr
            )
            initial_offsets[node_id] = correction
            print(
                f"INITIAL_OFFSET node={node_id} counts={current} "
                f"correction={correction:+.6f}",
                flush=True,
            )

        large_offsets = [
            node_id
            for node_id, correction in initial_offsets.items()
            if abs(correction) > max_correction
        ]
        if len(large_offsets) > 1:
            raise RuntimeError(
                "both discs are outside the safe alignment window; "
                "automatic recovery would risk collision"
            )

        base_targets: dict[int, float] = {}
        if large_offsets:
            recovery_id = large_offsets[0]
            guard_id = 3 if recovery_id == 2 else 2
            guard = nodes[guard_id]
            guard.enable()
            time.sleep(0.4)
            _guard_target, guard_correction, guard_counts = alignment_target(
                guard, references[guard_id], cpr
            )
            print(
                f"CLEARANCE_GUARD node={guard_id} counts={guard_counts} "
                f"correction={guard_correction:+.6f}",
                flush=True,
            )
            if abs(guard_correction) > max_correction:
                raise RuntimeError(
                    f"clearance node {guard_id} shifted outside the safe "
                    f"window: {guard_correction:+.6f} deg"
                )
            healthy(guard)

            base_targets[recovery_id] = align_node(
                nodes[recovery_id],
                references[recovery_id],
                cpr,
                MAX_SINGLE_DISC_RECOVERY_DEG,
            )
            base_targets[guard_id] = align_node(
                guard, references[guard_id], cpr, max_correction
            )
        else:
            for node_id in (2, 3):
                base_targets[node_id] = align_node(
                    nodes[node_id], references[node_id], cpr, max_correction
                )

        print(
            "AUTO_ALIGNMENT_COMPLETE "
            + " ".join(
                f"node={node_id} base={base_targets[node_id]:+.6f}"
                for node_id in (2, 3)
            ),
            flush=True,
        )

        if not args.align_only:
            for cycle in range(1, args.cycles + 1):
                node3_previous = base_targets[3] - 360.0 * (cycle - 1)
                node3_position = nodes[3].get_position()
                if abs(node3_position - node3_previous) > MOTION_TOLERANCE_DEG:
                    raise RuntimeError(
                        f"node 3 not aligned before cycle {cycle}: "
                        f"position={node3_position:+.6f} "
                        f"expected={node3_previous:+.6f}"
                    )

                desired2 = base_targets[2] - 360.0 * cycle
                print(f"CYCLE {cycle}/{args.cycles} MOVE node=2", flush=True)
                result2 = move_one_revolution(nodes[2], desired2, args.speed)
                rows.append(
                    {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "cycle": cycle,
                        "node_id": 2,
                        "speed_deg_s": args.speed,
                        **result2,
                    }
                )
                print(
                    f"ALIGNED node=2 error={result2['error_deg']:+.6f}",
                    flush=True,
                )

                # Immediate handoff: MOVE_DONE plus the position check above
                # is the gate.  There is no host dwell or compensation pass.
                desired3 = base_targets[3] - 360.0 * cycle
                print(f"CYCLE {cycle}/{args.cycles} MOVE node=3", flush=True)
                result3 = move_one_revolution(nodes[3], desired3, args.speed)
                rows.append(
                    {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "cycle": cycle,
                        "node_id": 3,
                        "speed_deg_s": args.speed,
                        **result3,
                    }
                )
                print(
                    f"ALIGNED node=3 error={result3['error_deg']:+.6f}",
                    flush=True,
                )
                healthy(nodes[2])
                healthy(nodes[3])

            print("SEQUENCE_COMPLETE", flush=True)

    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
        print(f"ABORT {failure}", flush=True)
        try:
            bus.estop_all()
        except Exception:
            pass
    finally:
        disabled = {2: False, 3: False}
        for _ in range(2):
            for node in nodes.values():
                try:
                    node.disable()
                except Exception:
                    pass
            time.sleep(0.25)
        for node in nodes.values():
            try:
                node.set_param(Param.MICROSTEPS, 8)
                node.set_param(Param.CLOSED_LOOP, 1)
                node.set_param(Param.PID_TOLERANCE, 0.5)
                set_motion_limits(node, 720.0)
            except Exception:
                pass
        try:
            disabled = {
                node_id: not node.get_status().enabled
                for node_id, node in nodes.items()
            }
            print("FINAL_DISABLED", disabled, flush=True)
        except Exception:
            pass
        bus.close()
        if not all(disabled.values()):
            failure = failure or "RuntimeError: final driver-disable verification failed"

    fields = [
        "timestamp", "cycle", "node_id", "speed_deg_s", "desired_deg",
        "position_deg", "error_deg", "elapsed_s",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "Absolute-reference alternating precision sequence",
        f"Result: {'FAILED' if failure else 'PASSED'}",
        f"Mode: {'alignment only' if args.align_only else 'alignment and motion'}",
        f"Completed moves: {len(rows)}",
        f"Speed: {args.speed:g} deg/s",
        f"Cycles: {args.cycles}",
        f"References: {references}",
        f"Runtime PID gains: {PID_GAINS}",
        "Persistent PID save: no",
    ]
    if failure:
        lines.append(f"Failure: {failure}")
    for row in rows:
        lines.append(
            f"cycle {row['cycle']} node {row['node_id']}: "
            f"error {row['error_deg']:+.6f} deg, "
            f"elapsed {row['elapsed_s']:.3f} s"
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"CSV_LOG {csv_path}", flush=True)
    print(f"SUMMARY_LOG {summary_path}", flush=True)
    print("FAILED" if failure else "PASSED", flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
