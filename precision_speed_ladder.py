#!/usr/bin/env python3
"""Run a guarded one-revolution precision ladder on one CANStepper node."""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

STATION = Path("/home/sonu/Downloads/grafito-canstepper-station")
sys.path.insert(0, str(STATION))

from canstepper import CANStepperBus, Param  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("node_id", type=int, choices=(2, 3))
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument(
        "--speeds",
        type=float,
        nargs="+",
        default=[30, 60, 90, 180, 360, 720],
    )
    parser.add_argument("--accel-multiplier", type=float, default=4.0)
    parser.add_argument("--abort-error", type=float, default=2.0)
    args = parser.parse_args()

    other_id = 3 if args.node_id == 2 else 2
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(__file__).resolve().parent
    csv_path = output_dir / f"node_{args.node_id}_speed_ladder_{stamp}.csv"
    summary_path = output_dir / f"node_{args.node_id}_speed_ladder_{stamp}.txt"
    rows: list[dict[str, object]] = []
    aborted = False
    abort_reason = ""

    print(
        f"NODE {args.node_id} LADDER speeds={args.speeds} "
        f"repetitions={args.repetitions} abort_error={args.abort_error}deg",
        flush=True,
    )

    bus = CANStepperBus.serial(args.port)
    time.sleep(2.5)
    node = bus.node(args.node_id)
    other = bus.node(other_id)

    try:
        node.disable()
        other.disable()
        node.set_param(Param.CLOSED_LOOP, 1)
        node.set_param(Param.INVERT_DIR, 0)

        node.set_zero()
        time.sleep(0.4)
        zero = node.get_position()
        print(f"ZERO_READBACK {zero:+.6f} deg", flush=True)
        if abs(zero) > 0.5:
            raise RuntimeError(f"zero verification failed: {zero:+.6f} deg")

        driver = node.get_driver_status()
        status = node.get_status()
        if (
            status.fault.value != 0
            or not status.encoder_ok
            or driver.any_short
            or driver.thermal_fault
        ):
            raise RuntimeError(f"pre-motion health failure: {status} {driver}")

        node.enable()
        time.sleep(0.5)
        if abs(node.get_position()) > 0.75:
            raise RuntimeError("position shifted after enable")

        for speed in args.speeds:
            accel = max(1.0, speed * args.accel_multiplier)
            node.set_param(Param.CL_MAX_SPEED, speed)
            node.set_param(Param.CL_MAX_ACCEL, accel)
            node.set_param(Param.MAX_SPEED, speed)
            node.set_param(Param.ACCELERATION, accel)
            print(f"SPEED {speed:g} deg/s ACCEL {accel:g} deg/s^2", flush=True)

            for repetition in range(1, args.repetitions + 1):
                status = node.get_status()
                driver = node.get_driver_status()
                if (
                    status.fault.value != 0
                    or not status.encoder_ok
                    or driver.any_short
                    or driver.thermal_fault
                ):
                    raise RuntimeError(
                        f"health failure before speed={speed:g} rep={repetition}: "
                        f"{status} {driver}"
                    )

                start = node.get_position()
                timeout = max(10.0, (360.0 / speed) * 3.0 + 5.0)
                started = time.monotonic()
                node.move_by(-360.0, blocking=True, timeout=timeout)
                elapsed = time.monotonic() - started
                time.sleep(0.35)
                final = node.get_position()
                displacement = final - start
                error = displacement + 360.0
                velocity, tracking_error = node.get_motion()
                status = node.get_status()
                driver = node.get_driver_status()

                row = {
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "node_id": args.node_id,
                    "speed_deg_s": speed,
                    "accel_deg_s2": accel,
                    "repetition": repetition,
                    "start_deg": start,
                    "final_deg": final,
                    "displacement_deg": displacement,
                    "revolution_error_deg": error,
                    "elapsed_s": elapsed,
                    "final_velocity_deg_s": velocity,
                    "tracking_error_deg": tracking_error,
                    "fault": status.fault.name,
                    "encoder_ok": status.encoder_ok,
                    "driver_ok": (
                        driver.uart_ok
                        and not driver.any_short
                        and not driver.thermal_fault
                    ),
                }
                rows.append(row)
                print(
                    f"  REP {repetition}/{args.repetitions} "
                    f"delta={displacement:+.6f} error={error:+.6f} "
                    f"time={elapsed:.3f}s",
                    flush=True,
                )

                if status.fault.value != 0 or not status.encoder_ok:
                    raise RuntimeError(
                        f"node fault after speed={speed:g} rep={repetition}: {status}"
                    )
                if driver.any_short or driver.thermal_fault:
                    raise RuntimeError(
                        f"driver fault after speed={speed:g} rep={repetition}: {driver}"
                    )
                if abs(error) > args.abort_error:
                    aborted = True
                    abort_reason = (
                        f"revolution error {error:+.6f} exceeds "
                        f"{args.abort_error:.3f} deg at speed={speed:g} "
                        f"rep={repetition}"
                    )
                    raise RuntimeError(abort_reason)

    except Exception as exc:
        aborted = True
        abort_reason = str(exc)
        print(f"ABORT {type(exc).__name__}: {exc}", flush=True)
    finally:
        try:
            node.set_param(Param.CL_MAX_SPEED, 720.0)
            node.set_param(Param.CL_MAX_ACCEL, 2880.0)
            node.set_param(Param.MAX_SPEED, 720.0)
            node.set_param(Param.ACCELERATION, 2880.0)
        except Exception:
            pass
        for axis in (node, other):
            try:
                axis.disable()
            except Exception:
                pass
        try:
            final_disabled = {
                args.node_id: not node.get_status().enabled,
                other_id: not other.get_status().enabled,
            }
            print(f"FINAL_DISABLED {final_disabled}", flush=True)
        except Exception:
            pass
        bus.close()

    fields = [
        "timestamp",
        "node_id",
        "speed_deg_s",
        "accel_deg_s2",
        "repetition",
        "start_deg",
        "final_deg",
        "displacement_deg",
        "revolution_error_deg",
        "elapsed_s",
        "final_velocity_deg_s",
        "tracking_error_deg",
        "fault",
        "encoder_ok",
        "driver_ok",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        f"Node {args.node_id} precision speed ladder",
        f"Result: {'ABORTED' if aborted else 'PASSED'}",
        f"CSV: {csv_path.name}",
        f"Completed revolutions: {len(rows)}",
    ]
    if abort_reason:
        lines.append(f"Abort reason: {abort_reason}")
    for speed in args.speeds:
        speed_rows = [r for r in rows if r["speed_deg_s"] == speed]
        if not speed_rows:
            continue
        errors = [float(r["revolution_error_deg"]) for r in speed_rows]
        elapsed = [float(r["elapsed_s"]) for r in speed_rows]
        lines.append(
            f"{speed:g} deg/s: n={len(errors)} "
            f"mean_error={statistics.mean(errors):+.6f} deg "
            f"max_abs_error={max(abs(e) for e in errors):.6f} deg "
            f"mean_time={statistics.mean(elapsed):.3f} s"
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"CSV_LOG {csv_path}", flush=True)
    print(f"SUMMARY_LOG {summary_path}", flush=True)
    print("PASSED" if not aborted else "FAILED", flush=True)
    return 1 if aborted else 0


if __name__ == "__main__":
    raise SystemExit(main())
