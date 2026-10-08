#!/usr/bin/env python3
"""LUT-assisted absolute alignment and alternating precision sequence.

This is an experimental copy of the proven controller.  It never calls
SET_ZERO or SAVE_CONFIG.  The default ``monitor`` mode reports eligible LUT
corrections without applying them; ``--lut-mode apply`` is required to alter
motion targets.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

STATION = Path("/home/sonu/Downloads/grafito-canstepper-station")
sys.path.insert(0, str(STATION))

from canstepper import CANStepperBus, Param  # noqa: E402

PORT = "/dev/ttyACM0"
REFERENCE_PATH = Path(__file__).with_name("alignment_reference.json")
LUT_PATH = Path(__file__).with_name("alignment_lut.json")
MOTION_TOLERANCE_DEG = 0.5
STARTUP_ALIGNMENT_SPEED = 30.0
DEFAULT_SPEED = 10000.0
DEFAULT_CYCLES = 5
MAX_SINGLE_DISC_RECOVERY_DEG = 180.0
# Runtime-only gains selected from the recorded 360 and 720 deg/s sweeps.
# These are intentionally node-specific: node 3 was not repeatable with the
# node 2 tune.  No SAVE_CONFIG command is issued anywhere in this program.
PID_GAINS = {
    2: (12.0, 0.05, 0.35),
    3: (10.0, 0.0, 0.4),
}
MAX_LUT_CORRECTION_DEG = 0.35
MIN_LUT_SAMPLES = 5
MAX_LUT_STDDEV_DEG = 0.10
EXPERIMENTAL_MAX_LUT_STDDEV_DEG = 0.25


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES)
    parser.add_argument("--align-only", action="store_true")
    parser.add_argument(
        "--lut-mode",
        choices=("off", "monitor", "apply"),
        default="monitor",
        help="off=no LUT, monitor=report only, apply=shift motion targets",
    )
    parser.add_argument("--lut-path", type=Path, default=LUT_PATH)
    parser.add_argument(
        "--log-mode",
        choices=("files", "stdout"),
        default="files",
        help="write timestamped files or emit the complete logs to stdout",
    )
    parser.add_argument(
        "--allow-experimental-lut",
        action="store_true",
        help="permit explicitly experimental entries with relaxed variance",
    )
    return parser.parse_args()


def load_lut(path: Path) -> dict[str, object]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise RuntimeError(f"unsupported LUT schema in {path}")
    if not isinstance(data.get("entries"), list):
        raise RuntimeError(f"LUT entries must be a list in {path}")
    return data


def select_lut_correction(
    lut: dict[str, object],
    node_id: int,
    speed: float,
    direction: str,
    allow_experimental: bool = False,
) -> tuple[float, str]:
    """Return a validated exact-speed correction, or zero with a reason.

    Interpolation and extrapolation are deliberately disabled until each node
    has repeatable measurements at more than one operating speed.
    """
    matches: list[dict[str, object]] = []
    for raw_entry in lut["entries"]:
        if not isinstance(raw_entry, dict):
            continue
        if (
            int(raw_entry.get("node_id", -1)) == node_id
            and str(raw_entry.get("direction", "")) == direction
            and math.isclose(
                float(raw_entry.get("speed_deg_s", -1.0)),
                speed,
                rel_tol=0.0,
                abs_tol=1e-6,
            )
        ):
            matches.append(raw_entry)

    if len(matches) != 1:
        return 0.0, f"expected one exact-speed entry, found {len(matches)}"

    entry = matches[0]
    if not bool(entry.get("enabled", False)):
        return 0.0, "entry disabled"
    validation_status = str(entry.get("validation_status", "candidate"))
    if validation_status == "experimental" and not allow_experimental:
        return 0.0, "experimental entry requires explicit opt-in"
    if validation_status not in ("validated", "experimental"):
        return 0.0, f"unsupported validation status {validation_status!r}"
    correction = float(entry["command_offset_deg"])
    samples = int(entry["samples"])
    stddev = float(entry["residual_stddev_deg"])
    if not all(math.isfinite(value) for value in (correction, stddev)):
        return 0.0, "non-finite correction statistics"
    if samples < MIN_LUT_SAMPLES:
        return 0.0, f"only {samples} samples; need {MIN_LUT_SAMPLES}"
    stddev_limit = (
        EXPERIMENTAL_MAX_LUT_STDDEV_DEG
        if validation_status == "experimental"
        else MAX_LUT_STDDEV_DEG
    )
    if stddev > stddev_limit:
        return 0.0, (
            f"stddev {stddev:.6f} exceeds {stddev_limit:.6f} deg"
        )
    if abs(correction) > MAX_LUT_CORRECTION_DEG:
        return 0.0, (
            f"correction {correction:+.6f} exceeds "
            f"{MAX_LUT_CORRECTION_DEG:.6f} deg cap"
        )
    return correction, f"eligible {validation_status} exact-speed entry"


def emit_logs(
    log_mode: str,
    rows: list[dict[str, object]],
    fields: list[str],
    lines: list[str],
    csv_path: Path,
    summary_path: Path,
    stream=None,
) -> None:
    """Emit complete logs either to the terminal or timestamped files."""
    if log_mode == "stdout":
        output = stream if stream is not None else sys.stdout
        print("SUMMARY_LOG_BEGIN", file=output)
        print("\n".join(lines), file=output)
        print("SUMMARY_LOG_END", file=output)
        print("CSV_LOG_BEGIN", file=output)
        writer = csv.DictWriter(output, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
        print("CSV_LOG_END", file=output)
        print("LOG_FILES_SAVED no", file=output, flush=True)
        return

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"CSV_LOG {csv_path}", flush=True)
    print(f"SUMMARY_LOG {summary_path}", flush=True)


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


def move_one_revolution(
    node,
    slot_target: float,
    speed: float,
    lut_candidate: float,
    lut_applied: float,
    reference: int,
    cpr: int,
) -> dict[str, float]:
    command_target = slot_target + lut_applied
    set_motion_limits(node, speed)
    started = time.monotonic()
    node.move_to(
        command_target,
        blocking=True,
        timeout=max(6.0, 360.0 / speed * 3.0 + 3.0),
    )
    final = node.get_position()
    slot_error = final - slot_target
    command_error = final - command_target
    reference_error = absolute_error_deg(node, reference, cpr)
    if abs(slot_error) > MOTION_TOLERANCE_DEG:
        raise RuntimeError(
            f"node {node.node_id} slot error {slot_error:+.6f} deg exceeds "
            f"{MOTION_TOLERANCE_DEG:.3f} deg"
        )
    if abs(reference_error) > MOTION_TOLERANCE_DEG:
        raise RuntimeError(
            f"node {node.node_id} raw-reference error "
            f"{reference_error:+.6f} deg exceeds "
            f"{MOTION_TOLERANCE_DEG:.3f} deg"
        )
    return {
        "desired_deg": slot_target,
        "slot_target_deg": slot_target,
        "command_target_deg": command_target,
        "lut_candidate_deg": lut_candidate,
        "lut_applied_deg": lut_applied,
        "position_deg": final,
        "error_deg": slot_error,
        "slot_error_deg": slot_error,
        "command_error_deg": command_error,
        "reference_error_deg": reference_error,
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
    lut = load_lut(args.lut_path) if args.lut_mode != "off" else {
        "schema_version": 1,
        "entries": [],
    }
    lut_candidates: dict[int, float] = {}
    lut_reasons: dict[int, str] = {}
    lut_applied: dict[int, float] = {}
    for node_id in (2, 3):
        candidate, reason = select_lut_correction(
            lut,
            node_id,
            args.speed,
            "clockwise",
            allow_experimental=args.allow_experimental_lut,
        )
        lut_candidates[node_id] = candidate
        lut_reasons[node_id] = reason
        lut_applied[node_id] = candidate if args.lut_mode == "apply" else 0.0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(__file__).resolve().parent
    csv_path = output_dir / f"absolute_lut_sequence_{stamp}.csv"
    summary_path = output_dir / f"absolute_lut_sequence_{stamp}.txt"
    rows: list[dict[str, object]] = []
    failure = ""

    print(
        f"ABSOLUTE_SEQUENCE speed={args.speed:g} cycles={args.cycles} "
        f"align_only={args.align_only} lut_mode={args.lut_mode} "
        f"allow_experimental={args.allow_experimental_lut}",
        flush=True,
    )
    for node_id in (2, 3):
        print(
            f"LUT node={node_id} candidate={lut_candidates[node_id]:+.6f} "
            f"applied={lut_applied[node_id]:+.6f} "
            f"reason={lut_reasons[node_id]}",
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
                node3_reference_error = absolute_error_deg(
                    nodes[3], references[3], cpr
                )
                if abs(node3_reference_error) > MOTION_TOLERANCE_DEG:
                    raise RuntimeError(
                        f"node 3 not aligned before cycle {cycle}: "
                        f"raw-reference error={node3_reference_error:+.6f}"
                    )

                desired2 = base_targets[2] - 360.0 * cycle
                print(f"CYCLE {cycle}/{args.cycles} MOVE node=2", flush=True)
                result2 = move_one_revolution(
                    nodes[2],
                    desired2,
                    args.speed,
                    lut_candidates[2],
                    lut_applied[2],
                    references[2],
                    cpr,
                )
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
                    f"ALIGNED node=2 slot_error="
                    f"{result2['slot_error_deg']:+.6f} "
                    f"lut={result2['lut_applied_deg']:+.6f}",
                    flush=True,
                )

                # Immediate handoff: MOVE_DONE plus the position check above
                # is the gate.  There is no host dwell or compensation pass.
                desired3 = base_targets[3] - 360.0 * cycle
                print(f"CYCLE {cycle}/{args.cycles} MOVE node=3", flush=True)
                result3 = move_one_revolution(
                    nodes[3],
                    desired3,
                    args.speed,
                    lut_candidates[3],
                    lut_applied[3],
                    references[3],
                    cpr,
                )
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
                    f"ALIGNED node=3 slot_error="
                    f"{result3['slot_error_deg']:+.6f} "
                    f"lut={result3['lut_applied_deg']:+.6f}",
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
        "slot_target_deg", "command_target_deg", "lut_candidate_deg",
        "lut_applied_deg", "position_deg", "error_deg", "slot_error_deg",
        "command_error_deg", "reference_error_deg", "elapsed_s",
    ]
    lines = [
        "LUT-assisted absolute-reference alternating precision sequence",
        f"Result: {'FAILED' if failure else 'PASSED'}",
        f"Mode: {'alignment only' if args.align_only else 'alignment and motion'}",
        f"Completed moves: {len(rows)}",
        f"Speed: {args.speed:g} deg/s",
        f"Cycles: {args.cycles}",
        f"References: {references}",
        f"Runtime PID gains: {PID_GAINS}",
        f"LUT mode: {args.lut_mode}",
        f"Experimental LUT allowed: {args.allow_experimental_lut}",
        f"LUT path: {args.lut_path}",
        f"LUT candidates: {lut_candidates}",
        f"LUT applied: {lut_applied}",
        f"LUT reasons: {lut_reasons}",
        f"Log mode: {args.log_mode}",
        "Persistent PID save: no",
    ]
    if failure:
        lines.append(f"Failure: {failure}")
    for row in rows:
        lines.append(
            f"cycle {row['cycle']} node {row['node_id']}: "
            f"slot error {row['slot_error_deg']:+.6f} deg, "
            f"LUT {row['lut_applied_deg']:+.6f} deg, "
            f"elapsed {row['elapsed_s']:.3f} s"
        )
    emit_logs(
        args.log_mode,
        rows,
        fields,
        lines,
        csv_path,
        summary_path,
    )
    print("FAILED" if failure else "PASSED", flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
