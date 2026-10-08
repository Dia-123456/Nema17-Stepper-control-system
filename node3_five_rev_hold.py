#!/usr/bin/env python3
"""Five-revolution node-3 test that holds until DISABLE is entered."""

from __future__ import annotations

import sys
import time
from pathlib import Path

STATION = Path("/home/sonu/Downloads/grafito-canstepper-station")
sys.path.insert(0, str(STATION))

from canstepper import CANStepperBus, Param  # noqa: E402

PORT = "/dev/ttyACM0"
DESIRED = -1800.0


def main() -> int:
    bus = CANStepperBus.serial(PORT)
    time.sleep(2.5)
    node2 = bus.node(2)
    node3 = bus.node(3)
    holding = False
    success = False

    try:
        node2.disable()
        node3.disable()
        node3.set_param(Param.MICROSTEPS, 32)
        node3.set_param(Param.CLOSED_LOOP, 1)
        node3.set_param(Param.INVERT_DIR, 0)
        node3.set_param(Param.MAX_SPEED, 360.0)
        node3.set_param(Param.ACCELERATION, 1440.0)
        node3.set_param(Param.CL_MAX_SPEED, 360.0)
        node3.set_param(Param.CL_MAX_ACCEL, 1440.0)
        node3.set_param(Param.PID_TOLERANCE, 0.05)

        node3.set_zero()
        time.sleep(0.4)
        zero = node3.get_position()
        print(f"ZERO_READBACK {zero:+.6f}", flush=True)
        if abs(zero) > 0.05:
            raise RuntimeError(f"zero verification failed: {zero:+.6f}")

        status = node3.get_status()
        driver = node3.get_driver_status()
        if (
            status.fault.value != 0
            or not status.encoder_ok
            or driver.any_short
            or driver.thermal_fault
        ):
            raise RuntimeError(f"preflight failed: {status} {driver}")

        node3.enable()
        time.sleep(0.5)
        control_target = DESIRED
        started = time.monotonic()

        for attempt in range(1, 6):
            print(
                f"ATTEMPT {attempt} control_target={control_target:+.6f}",
                flush=True,
            )
            node3.move_to(control_target, blocking=False)
            deadline = time.monotonic() + (15.0 if attempt == 1 else 8.0)
            stable = 0
            while time.monotonic() < deadline:
                position = node3.get_position()
                velocity = node3.get_velocity()
                status = node3.get_status()
                if status.fault.value != 0 or not status.encoder_ok:
                    raise RuntimeError(f"motion health failure: {status}")
                if abs(position - DESIRED) <= 0.05 and abs(velocity) <= 1.0:
                    stable += 1
                    if stable >= 5:
                        break
                else:
                    stable = 0
                time.sleep(0.02)

            position = node3.get_position()
            error = position - DESIRED
            print(
                f"ATTEMPT_RESULT {attempt} position={position:+.6f} "
                f"desired_error={error:+.6f}",
                flush=True,
            )
            if abs(error) <= 0.05:
                success = True
                break
            control_target -= error

        samples = []
        for _ in range(20):
            samples.append(node3.get_position())
            time.sleep(0.05)
        final = samples[-1]
        error = final - DESIRED
        elapsed = time.monotonic() - started
        status = node3.get_status()
        driver = node3.get_driver_status()
        print(
            f"RESULT desired={DESIRED:+.6f} enabled_position={final:+.6f} "
            f"error={error:+.6f} elapsed_s={elapsed:.3f} "
            f"max_abs_error_1s={max(abs(p - DESIRED) for p in samples):.6f}",
            flush=True,
        )
        print(
            "SAMPLES " + " ".join(f"{p - DESIRED:+.6f}" for p in samples),
            flush=True,
        )
        print(f"STATUS {status} DRIVER {driver}", flush=True)
        if not success or abs(error) > 0.05:
            raise RuntimeError(
                f"unable to achieve enabled error <=0.05 deg; final {error:+.6f}"
            )

        holding = True
        print("HOLDING_NODE_3_ENABLED", flush=True)
        print("Enter DISABLE to release node 3 and restore runtime settings.", flush=True)
        while True:
            command = input().strip().upper()
            if command == "DISABLE":
                break
            print("WAITING_FOR_DISABLE", flush=True)

    except Exception as exc:
        print(f"FAILED {type(exc).__name__}: {exc}", flush=True)
        success = False
    finally:
        try:
            node2.disable()
        except Exception:
            pass
        if not holding or not success:
            try:
                node3.disable()
            except Exception:
                pass
        elif success:
            # Reached only after the operator sends DISABLE.
            try:
                node3.disable()
            except Exception:
                pass

        if not holding or not success or (holding and success):
            try:
                node3.set_param(Param.MICROSTEPS, 8)
                node3.set_param(Param.MAX_SPEED, 720.0)
                node3.set_param(Param.ACCELERATION, 2880.0)
                node3.set_param(Param.CL_MAX_SPEED, 720.0)
                node3.set_param(Param.CL_MAX_ACCEL, 2880.0)
                node3.set_param(Param.PID_TOLERANCE, 0.5)
            except Exception:
                pass

        try:
            print(
                "FINAL_ENABLED",
                {2: node2.get_status().enabled, 3: node3.get_status().enabled},
                flush=True,
            )
        except Exception:
            pass
        bus.close()

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
