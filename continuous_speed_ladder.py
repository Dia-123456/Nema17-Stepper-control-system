#!/usr/bin/env python3
"""Continuous speed ladder with encoder-boundary transitions and final correction."""

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
NODE_ID = 2
OTHER_ID = 3
SPEEDS = [30.0, 60.0, 90.0, 180.0, 360.0, 720.0]
VELOCITY_ACCEL = 2880.0
FINAL_CORRECTION_SPEED = 90.0
FINAL_CORRECTION_ACCEL = 360.0


def main() -> int:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(__file__).resolve().parent
    csv_path = out_dir / f"node_{NODE_ID}_continuous_ladder_{stamp}.csv"
    summary_path = out_dir / f"node_{NODE_ID}_continuous_ladder_{stamp}.txt"
    rows: list[dict[str, object]] = []
    failure = ""
    final_error: float | None = None

    print(f"CONTINUOUS NODE {NODE_ID} SPEEDS={SPEEDS}", flush=True)
    bus = CANStepperBus.serial(PORT)
    time.sleep(2.5)
    node = bus.node(NODE_ID)
    other = bus.node(OTHER_ID)

    try:
        node.disable()
        other.disable()
        node.set_param(Param.INVERT_DIR, 0)
        node.set_param(Param.CLOSED_LOOP, 1)
        node.set_param(Param.ACCELERATION, VELOCITY_ACCEL)
        node.set_param(Param.MAX_SPEED, max(SPEEDS))

        node.set_zero()
        time.sleep(0.4)
        zero = node.get_position()
        print(f"ZERO_READBACK {zero:+.6f} deg", flush=True)
        if abs(zero) > 0.5:
            raise RuntimeError(f"zero verification failed: {zero:+.6f} deg")

        status = node.get_status()
        driver = node.get_driver_status()
        if (
            status.fault.value != 0
            or not status.encoder_ok
            or driver.any_short
            or driver.thermal_fault
        ):
            raise RuntimeError(f"pre-motion health failure: {status} {driver}")

        node.enable()
        time.sleep(0.5)
        start = node.get_position()
        if abs(start) > 0.75:
            raise RuntimeError(f"position shifted after enable: {start:+.6f} deg")

        print(f"START speed={SPEEDS[0]:g} deg/s", flush=True)
        node.run(-SPEEDS[0])
        segment_started = time.monotonic()

        for index, speed in enumerate(SPEEDS):
            boundary = -360.0 * (index + 1)
            expected = 360.0 / speed
            deadline = time.monotonic() + max(8.0, expected * 3.0 + 5.0)
            samples = 0
            last_health = time.monotonic()

            while True:
                position = node.get_position()
                samples += 1
                now = time.monotonic()
                if position <= boundary:
                    crossing_offset = position - boundary
                    segment_time = now - segment_started
                    row = {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "node_id": NODE_ID,
                        "revolution": index + 1,
                        "speed_deg_s": speed,
                        "boundary_deg": boundary,
                        "crossing_position_deg": position,
                        "crossing_offset_deg": crossing_offset,
                        "segment_time_s": segment_time,
                        "samples": samples,
                    }
                    rows.append(row)
                    print(
                        f"BOUNDARY rev={index + 1} speed={speed:g} "
                        f"target={boundary:+.3f} pos={position:+.3f} "
                        f"offset={crossing_offset:+.3f} time={segment_time:.3f}s",
                        flush=True,
                    )
                    if index + 1 < len(SPEEDS):
                        next_speed = SPEEDS[index + 1]
                        node.run(-next_speed)
                        print(
                            f"SPEED_CHANGE at={position:+.3f} "
                            f"{speed:g}->{next_speed:g} deg/s",
                            flush=True,
                        )
                        segment_started = time.monotonic()
                    break

                if now >= deadline:
                    raise TimeoutError(
                        f"boundary timeout rev={index + 1} speed={speed:g} "
                        f"position={position:+.3f} target={boundary:+.3f}"
                    )
                if now - last_health >= 0.5:
                    status = node.get_status()
                    if status.fault.value != 0 or not status.encoder_ok:
                        raise RuntimeError(f"node health failure: {status}")
                    last_health = now
                time.sleep(0.002)

        # No intermediate stop occurred. Ramp down after crossing the final
        # alignment, then correct slowly to the exact final boundary.
        final_target = -360.0 * len(SPEEDS)
        print("FINAL_BOUNDARY_CROSSED ramped stop", flush=True)
        node.stop()
        stop_deadline = time.monotonic() + 5.0
        while time.monotonic() < stop_deadline:
            velocity = node.get_velocity()
            if abs(velocity) < 2.0:
                break
            time.sleep(0.02)
        stopped_at = node.get_position()
        print(f"STOPPED_AT {stopped_at:+.6f} deg", flush=True)

        node.set_param(Param.CL_MAX_SPEED, FINAL_CORRECTION_SPEED)
        node.set_param(Param.CL_MAX_ACCEL, FINAL_CORRECTION_ACCEL)
        node.set_param(Param.MAX_SPEED, FINAL_CORRECTION_SPEED)
        node.set_param(Param.ACCELERATION, FINAL_CORRECTION_ACCEL)
        node.move_to(final_target, blocking=True, timeout=20.0)
        time.sleep(0.4)
        final_position = node.get_position()
        final_error = final_position - final_target
        status = node.get_status()
        driver = node.get_driver_status()
        print(
            f"FINAL target={final_target:+.6f} position={final_position:+.6f} "
            f"error={final_error:+.6f}",
            flush=True,
        )
        if status.fault.value != 0 or not status.encoder_ok:
            raise RuntimeError(f"final node health failure: {status}")
        if driver.any_short or driver.thermal_fault:
            raise RuntimeError(f"final driver health failure: {driver}")
        if abs(final_error) > 2.0:
            raise RuntimeError(f"final alignment error too large: {final_error:+.6f}")

    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
        print(f"ABORT {failure}", flush=True)
        try:
            bus.estop_all()
        except Exception:
            pass
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
            print(
                "FINAL_DISABLED",
                {NODE_ID: not node.get_status().enabled, OTHER_ID: not other.get_status().enabled},
                flush=True,
            )
        except Exception:
            pass
        bus.close()

    fields = [
        "timestamp",
        "node_id",
        "revolution",
        "speed_deg_s",
        "boundary_deg",
        "crossing_position_deg",
        "crossing_offset_deg",
        "segment_time_s",
        "samples",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        f"Node {NODE_ID} continuous speed ladder",
        f"Result: {'FAILED' if failure else 'PASSED'}",
        f"Speeds: {SPEEDS}",
        f"Completed boundaries: {len(rows)}/{len(SPEEDS)}",
        f"Final alignment error: {final_error if final_error is not None else 'unavailable'}",
    ]
    if failure:
        lines.append(f"Failure: {failure}")
    for row in rows:
        lines.append(
            f"rev {row['revolution']} @ {row['speed_deg_s']:g} deg/s: "
            f"crossing offset {row['crossing_offset_deg']:+.6f} deg, "
            f"time {row['segment_time_s']:.3f} s"
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"CSV_LOG {csv_path}", flush=True)
    print(f"SUMMARY_LOG {summary_path}", flush=True)
    print("FAILED" if failure else "PASSED", flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
