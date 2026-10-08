#!/usr/bin/env python3
"""Alternate one aligned revolution per node at an increasing speed ladder."""

from __future__ import annotations

import csv
import sys
import time
from datetime import datetime
from pathlib import Path

STATION = Path("/home/sonu/Downloads/grafito-canstepper-station")
sys.path.insert(0, str(STATION))

from canstepper import CANStepperBus, Param  # noqa: E402

PORT = "/dev/ttyACM0"
SPEEDS = [360.0] * 5
ALIGN_TOL = 0.5
MAX_ENABLE_SHIFT = 2.5
CORRECTION_SPEED = 10.0
MAX_ATTEMPTS = 8


def healthy(node) -> None:
    status = node.get_status()
    driver = node.get_driver_status()
    if status.fault.value != 0 or not status.encoder_ok:
        raise RuntimeError(f"node {node.node_id} health failure: {status}")
    if not driver.uart_ok or driver.any_short or driver.thermal_fault:
        raise RuntimeError(f"node {node.node_id} driver failure: {driver}")


def move_aligned(node, desired: float, speed: float, compensation: float):
    node.set_param(Param.CLOSED_LOOP, 1)
    accel = speed * 4.0
    node.set_param(Param.MAX_SPEED, speed)
    node.set_param(Param.ACCELERATION, accel)
    node.set_param(Param.CL_MAX_SPEED, speed)
    node.set_param(Param.CL_MAX_ACCEL, accel)
    healthy(node)

    started = time.monotonic()
    control_target = desired
    timeout = max(6.0, 360.0 / speed * 3.0 + 3.0)
    node.move_to(control_target, blocking=True, timeout=timeout)

    final = node.get_position()
    error = final - desired
    max_error = abs(error)
    print(
        f"  SETTLED node={node.node_id} position={final:+.6f} "
        f"desired_error={error:+.6f}",
        flush=True,
    )
    healthy(node)
    if max_error > ALIGN_TOL:
        raise RuntimeError(
            f"node {node.node_id} failed alignment at {speed:g} deg/s: "
            f"desired={desired:+.6f} final={final:+.6f} error={error:+.6f}"
        )

    return {
        "attempts": 1,
        "control_target": control_target,
        "final_position": final,
        "error": error,
        "max_stable_error": max_error,
        "elapsed_s": time.monotonic() - started,
        "compensation": 0.0,
    }


def main() -> int:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(__file__).resolve().parent
    csv_path = output_dir / f"alternating_sequence_{stamp}.csv"
    summary_path = output_dir / f"alternating_sequence_{stamp}.txt"
    rows: list[dict[str, object]] = []
    failure = ""
    compensation = {2: 0.0, 3: 0.13}

    print(f"ALTERNATING_SEQUENCE speeds={SPEEDS}", flush=True)
    bus = CANStepperBus.serial(PORT)
    time.sleep(2.5)
    node2 = bus.node(2)
    node3 = bus.node(3)
    nodes = {2: node2, 3: node3}

    try:
        # Keep a reconnect/reset from intentionally energizing the rig during
        # this run.  Persistence is handled separately with operator approval.
        node2.set_param(Param.ENABLE_ON_BOOT, 0)
        node3.set_param(Param.ENABLE_ON_BOOT, 0)
        node2.disable()
        node3.disable()
        time.sleep(0.5)
        for node_id, node in nodes.items():
            if node.get_status().enabled:
                raise RuntimeError(f"node {node_id} did not disable in preflight")

        for node_id, microsteps in ((2, 32), (3, 32)):
            node = nodes[node_id]
            node.set_param(Param.MICROSTEPS, microsteps)
            node.set_param(Param.CLOSED_LOOP, 1)
            node.set_param(Param.INVERT_DIR, 0)
            node.set_param(Param.PID_TOLERANCE, ALIGN_TOL)
            node.set_zero()
            time.sleep(0.3)
            zero = node.get_position()
            print(f"ZERO node={node_id} position={zero:+.6f}", flush=True)
            if abs(zero) > ALIGN_TOL:
                raise RuntimeError(
                    f"node {node_id} zero verification failed: {zero:+.6f}"
                )
            healthy(node)

        # Energizing can pull a rotor to the nearest electrical holding point.
        # Bound that shift, then re-zero at the actual held slot position.
        for node_id, node in nodes.items():
            node.enable()
            time.sleep(0.5)
            shift = node.get_position()
            print(f"ENABLE_SHIFT node={node_id} position={shift:+.6f}", flush=True)
            if abs(shift) > MAX_ENABLE_SHIFT:
                raise RuntimeError(
                    f"node {node_id} shifted {shift:+.6f} deg after enable"
                )
            node.set_zero()
            time.sleep(0.3)
            held_zero = node.get_position()
            print(f"HELD_ZERO node={node_id} position={held_zero:+.6f}", flush=True)
            if abs(held_zero) > ALIGN_TOL:
                raise RuntimeError(
                    f"node {node_id} held-zero verification failed: "
                    f"{held_zero:+.6f}"
                )
            healthy(node)

        print("ALIGNMENT_HOLD", flush=True)
        print("Enter PROCEED to run or ABORT to disable.", flush=True)
        while True:
            decision = input().strip().upper()
            if decision == "PROCEED":
                break
            if decision == "ABORT":
                raise RuntimeError("operator aborted at alignment hold")
            print("WAITING_FOR_PROCEED_OR_ABORT", flush=True)

        for stage, speed in enumerate(SPEEDS, start=1):
            desired = -360.0 * stage
            print(
                f"STAGE {stage}/{len(SPEEDS)} speed={speed:g} "
                f"desired={desired:+.3f}",
                flush=True,
            )

            for node_id in (2, 3):
                node = nodes[node_id]
                other = nodes[3 if node_id == 2 else 2]
                other_position = other.get_position()
                other_desired = -360.0 * (stage - 1 if node_id == 2 else stage)
                if abs(other_position - other_desired) > ALIGN_TOL:
                    raise RuntimeError(
                        f"stationary node {other.node_id} is not aligned: "
                        f"position={other_position:+.6f} "
                        f"expected={other_desired:+.6f}"
                    )
                print(f"MOVE node={node_id} speed={speed:g}", flush=True)
                result = move_aligned(
                    node,
                    desired,
                    speed,
                    compensation[node_id],
                )
                compensation[node_id] = float(result["compensation"])
                rows.append(
                    {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "stage": stage,
                        "speed_deg_s": speed,
                        "node_id": node_id,
                        "desired_deg": desired,
                        **result,
                    }
                )
                print(
                    f"ALIGNED node={node_id} position="
                    f"{float(result['final_position']):+.6f} "
                    f"error={float(result['error']):+.6f} "
                    f"attempts={result['attempts']}",
                    flush=True,
                )

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
            for node in (node2, node3):
                try:
                    node.disable()
                except Exception:
                    pass
            time.sleep(0.25)
        for node in (node2, node3):
            try:
                node.set_param(Param.MICROSTEPS, 8)
                node.set_param(Param.CLOSED_LOOP, 1)
                node.set_param(Param.MAX_SPEED, 720.0)
                node.set_param(Param.ACCELERATION, 2880.0)
                node.set_param(Param.CL_MAX_SPEED, 720.0)
                node.set_param(Param.CL_MAX_ACCEL, 2880.0)
                node.set_param(Param.PID_TOLERANCE, 0.5)
            except Exception:
                pass
        try:
            disabled = {
                2: not node2.get_status().enabled,
                3: not node3.get_status().enabled,
            }
            print("FINAL_DISABLED", disabled, flush=True)
        except Exception:
            pass
        bus.close()

        if not all(disabled.values()):
            failure = failure or "RuntimeError: final driver-disable verification failed"

    fields = [
        "timestamp",
        "stage",
        "speed_deg_s",
        "node_id",
        "desired_deg",
        "attempts",
        "control_target",
        "final_position",
        "error",
        "max_stable_error",
        "elapsed_s",
        "compensation",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "Alternating node 2 / node 3 precision sequence",
        f"Result: {'FAILED' if failure else 'PASSED'}",
        f"Completed moves: {len(rows)}/{len(SPEEDS) * 2}",
        f"Speeds: {SPEEDS}",
    ]
    if failure:
        lines.append(f"Failure: {failure}")
    for row in rows:
        lines.append(
            f"stage {row['stage']} node {row['node_id']} "
            f"speed {row['speed_deg_s']:g}: error {row['error']:+.6f} deg, "
            f"attempts {row['attempts']}, elapsed {row['elapsed_s']:.3f} s"
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"CSV_LOG {csv_path}", flush=True)
    print(f"SUMMARY_LOG {summary_path}", flush=True)
    print("FAILED" if failure else "PASSED", flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
