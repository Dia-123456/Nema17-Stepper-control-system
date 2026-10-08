#!/usr/bin/env python3
"""Standalone two-node PID + LUT precision sequence.

The LUT lives in this Python file, not in firmware.  Nothing is saved to
board flash and no CSV/TXT files are created.  The complete settings, motion
results and CSV-formatted rows are printed directly in the terminal.

Node 2 is deliberately marked experimental: a previous repeated test exceeded
the 0.5-degree slot tolerance.  Applying both LUTs therefore requires the
explicit ``--accept-node2-experimental-risk`` option.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

STATION = Path("/home/sonu/Downloads/grafito-canstepper-station")
sys.path.insert(0, str(STATION))

from canstepper import CANStepperBus, Param  # noqa: E402


PORT = "/dev/ttyACM0"
REFERENCE_PATH = Path(__file__).with_name("alignment_reference_lut.json")
LUT_CALIBRATION_SPEED_DEG_S = 720.0
DEFAULT_SPEED_DEG_S = 720.0
DEFAULT_CYCLES = 1
MAX_CYCLES = 5
MOTION_TOLERANCE_DEG = 0.5
# The user visually confirmed that the observed ~0.59-degree node-2 return
# offset clears the other disc. Keep the firmware PID tolerance at 0.5 degree,
# but use a separate physical handoff limit for each disc.
SLOT_SAFETY_TOLERANCE_DEG = {
    2: 0.65,
    3: 0.5,
}
STARTUP_ALIGNMENT_SPEED = 30.0
MAX_SINGLE_DISC_RECOVERY_DEG = 180.0

# These runtime PID gains match the conditions used to measure the LUT.
# This program never calls SAVE_CONFIG, so persistent board settings are not
# changed.
PID_GAINS = {
    2: (12.0, 0.05, 0.35),
    3: (10.0, 0.0, 0.4),
}

# Algebraic offsets added to the physical slot target before it is sent to
# the firmware PID. Clockwise full-revolution targets are negative on this rig.
LUT = {
    2: {
        "offset_deg": +0.1373291015625,
        "samples": 12,
        "stddev_deg": 0.1981142402486238,
        "status": "experimental",
        "warning": (
            "previous repeated test reached +0.593262 deg and safety-aborted"
        ),
    },
    3: {
        "offset_deg": -0.25927734375,
        "samples": 5,
        "stddev_deg": 0.03610475842929562,
        "status": "validated",
        "warning": "5/5 validation moves passed; max error 0.021973 deg",
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone variable-speed PID+LUT controller for nodes 2 and 3"
    )
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_DEG_S)
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES)
    parser.add_argument("--align-only", action="store_true")
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="run the same sequence but apply zero LUT correction",
    )
    parser.add_argument(
        "--accept-node2-experimental-risk",
        action="store_true",
        help=(
            "required to apply LUTs; node 2 previously exceeded the "
            "0.5-degree slot tolerance"
        ),
    )
    args = parser.parse_args()
    if not 1 <= args.cycles <= MAX_CYCLES:
        parser.error(f"cycles must be between 1 and {MAX_CYCLES}")
    if not 0 < args.speed <= LUT_CALIBRATION_SPEED_DEG_S:
        parser.error(
            f"speed must be greater than 0 and no more than "
            f"{LUT_CALIBRATION_SPEED_DEG_S:g} deg/s"
        )
    if (
        not args.align_only
        and not args.monitor
        and not args.accept_node2_experimental_risk
    ):
        parser.error(
            "--accept-node2-experimental-risk is required to apply both LUTs; "
            "use --monitor for an uncompensated run"
        )
    return args


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
            f"node {node.node_id} startup offset {correction:+.6f} deg "
            f"exceeds {max_correction:.3f} deg limit"
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
            f"node {node.node_id} alignment residual {residual:+.6f} deg "
            f"exceeds {MOTION_TOLERANCE_DEG:.3f} deg"
        )
    healthy(node)
    return target


def move_one_revolution(
    node,
    slot_target: float,
    lut_offset: float,
    speed: float,
    reference: int,
    cpr: int,
) -> dict[str, float]:
    command_target = slot_target + lut_offset
    set_motion_limits(node, speed)
    started = time.monotonic()
    node.move_to(command_target, blocking=True, timeout=6.0)
    final = node.get_position()
    slot_error = final - slot_target
    command_error = final - command_target
    reference_error = absolute_error_deg(node, reference, cpr)

    slot_limit = SLOT_SAFETY_TOLERANCE_DEG[node.node_id]
    if abs(slot_error) > slot_limit:
        raise RuntimeError(
            f"node {node.node_id} slot error {slot_error:+.6f} deg exceeds "
            f"physical handoff limit {slot_limit:.3f} deg"
        )
    if abs(reference_error) > slot_limit:
        raise RuntimeError(
            f"node {node.node_id} raw-reference error "
            f"{reference_error:+.6f} deg exceeds "
            f"physical handoff limit {slot_limit:.3f} deg"
        )
    precision_warning = abs(slot_error) > MOTION_TOLERANCE_DEG

    return {
        "slot_target_deg": slot_target,
        "command_target_deg": command_target,
        "lut_offset_deg": lut_offset,
        "position_deg": final,
        "slot_error_deg": slot_error,
        "command_error_deg": command_error,
        "reference_error_deg": reference_error,
        "precision_warning": precision_warning,
        "elapsed_s": time.monotonic() - started,
    }


def print_settings(lut_offsets: dict[int, float], mode: str, speed: float) -> None:
    print("PID_LUT_SETTINGS", flush=True)
    print(
        f"mode={mode} speed={speed:g}deg/s "
        f"acceleration={speed * 4.0:g}deg/s^2 "
        f"tolerance={MOTION_TOLERANCE_DEG:g}deg",
        flush=True,
    )
    speed_status = (
        "measured-speed"
        if speed == LUT_CALIBRATION_SPEED_DEG_S
        else "experimental-speed-extrapolation"
    )
    print(
        f"lut_calibration_speed={LUT_CALIBRATION_SPEED_DEG_S:g}deg/s "
        f"lut_speed_status={speed_status}",
        flush=True,
    )
    for node_id in (2, 3):
        kp, ki, kd = PID_GAINS[node_id]
        entry = LUT[node_id]
        print(
            f"node={node_id} PID(kp={kp:g},ki={ki:g},kd={kd:g}) "
            f"LUT(stored={entry['offset_deg']:+.6f}deg,"
            f"applied={lut_offsets[node_id]:+.6f}deg,"
            f"samples={entry['samples']},stddev={entry['stddev_deg']:.6f}deg,"
            f"status={entry['status']})",
            flush=True,
        )
        print(
            f"node={node_id} pid_tolerance={MOTION_TOLERANCE_DEG:.3f}deg "
            f"physical_handoff_limit="
            f"{SLOT_SAFETY_TOLERANCE_DEG[node_id]:.3f}deg",
            flush=True,
        )
        print(f"node={node_id} note={entry['warning']}", flush=True)
    print("log_destination=terminal_only files_saved=no", flush=True)


def print_report(
    rows: list[dict[str, object]],
    failure: str,
    cycles: int,
    mode: str,
    disabled: dict[int, bool],
    speed: float,
) -> None:
    print("SUMMARY_LOG_BEGIN", flush=True)
    print("Standalone two-node PID + LUT sequence")
    print(f"Result: {'FAILED' if failure else 'PASSED'}")
    print(f"Mode: {mode}")
    print(f"Speed: {speed:g} deg/s")
    print(f"Cycles requested: {cycles}")
    print(f"Completed moves: {len(rows)}")
    print(f"Final disabled: {disabled}")
    print("Persistent PID/LUT save: no")
    print("Log files saved: no")
    if failure:
        print(f"Failure: {failure}")
    print("SUMMARY_LOG_END", flush=True)

    fields = [
        "cycle",
        "node_id",
        "speed_deg_s",
        "slot_target_deg",
        "command_target_deg",
        "lut_offset_deg",
        "position_deg",
        "slot_error_deg",
        "command_error_deg",
        "reference_error_deg",
        "precision_warning",
        "elapsed_s",
    ]
    print("CSV_LOG_BEGIN", flush=True)
    writer = csv.DictWriter(sys.stdout, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    print("CSV_LOG_END", flush=True)


def main() -> int:
    args = parse_args()
    mode = "align-only" if args.align_only else ("monitor" if args.monitor else "apply")
    lut_offsets = {
        node_id: (
            float(LUT[node_id]["offset_deg"])
            if mode == "apply"
            else 0.0
        )
        for node_id in (2, 3)
    }
    print_settings(lut_offsets, mode, args.speed)

    reference_data = json.loads(REFERENCE_PATH.read_text(encoding="utf-8"))
    cpr = int(reference_data["encoder_counts_per_revolution"])
    max_correction = float(reference_data["max_automatic_correction_deg"])
    references = {
        int(node_id): int(values["aligned_raw_count"])
        for node_id, values in reference_data["nodes"].items()
    }

    rows: list[dict[str, object]] = []
    failure = ""
    disabled = {2: False, 3: False}
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

        large = [
            node_id
            for node_id, correction in initial_offsets.items()
            if abs(correction) > max_correction
        ]
        if len(large) > 1:
            raise RuntimeError(
                "both discs are outside the safe alignment window; "
                "automatic recovery would risk collision"
            )

        base_targets: dict[int, float] = {}
        if large:
            recovery_id = large[0]
            guard_id = 3 if recovery_id == 2 else 2
            guard = nodes[guard_id]
            guard.enable()
            time.sleep(0.4)
            _target, guard_correction, guard_counts = alignment_target(
                guard, references[guard_id], cpr
            )
            print(
                f"CLEARANCE_GUARD node={guard_id} counts={guard_counts} "
                f"correction={guard_correction:+.6f}",
                flush=True,
            )
            if abs(guard_correction) > max_correction:
                raise RuntimeError(
                    f"clearance node {guard_id} is outside the safe window"
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

        if not args.align_only:
            for cycle in range(1, args.cycles + 1):
                guard_error = absolute_error_deg(nodes[3], references[3], cpr)
                if abs(guard_error) > MOTION_TOLERANCE_DEG:
                    raise RuntimeError(
                        f"node 3 is not aligned before cycle {cycle}: "
                        f"raw-reference error={guard_error:+.6f} deg"
                    )

                for node_id in (2, 3):
                    slot_target = base_targets[node_id] - 360.0 * cycle
                    print(
                        f"CYCLE {cycle}/{args.cycles} MOVE node={node_id}",
                        flush=True,
                    )
                    result = move_one_revolution(
                        nodes[node_id],
                        slot_target,
                        lut_offsets[node_id],
                        args.speed,
                        references[node_id],
                        cpr,
                    )
                    rows.append(
                        {
                            "cycle": cycle,
                            "node_id": node_id,
                            "speed_deg_s": args.speed,
                            **result,
                        }
                    )
                    print(
                        f"ALIGNED node={node_id} "
                        f"slot_error={result['slot_error_deg']:+.6f} "
                        f"lut={result['lut_offset_deg']:+.6f} "
                        f"precision_warning={result['precision_warning']}",
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
                node.set_param(Param.PID_TOLERANCE, MOTION_TOLERANCE_DEG)
                set_motion_limits(node, LUT_CALIBRATION_SPEED_DEG_S)
            except Exception:
                pass
        try:
            disabled = {
                node_id: not node.get_status().enabled
                for node_id, node in nodes.items()
            }
            print(f"FINAL_DISABLED {disabled}", flush=True)
        except Exception:
            pass
        bus.close()
        if not all(disabled.values()):
            failure = failure or "RuntimeError: driver-disable verification failed"

    print_report(rows, failure, args.cycles, mode, disabled, args.speed)
    print("FAILED" if failure else "PASSED", flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
