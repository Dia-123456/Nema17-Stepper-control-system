#!/usr/bin/env python3
"""Bounded 1440-deg/s alternating experiment; terminal output, no flash saves.

The working controller is imported without modifying its source or references.
Node 2 uses the approved 128 microsteps during rotation, 256 at the finish.
PID gains, LUT values, tolerances and physical abort limits are unchanged.
One cycle means node 2 turns once, then node 3 turns once. There is no added
dwell, but encoder/health checks and bounded precision corrections remain.
"""

import argparse
import importlib.util
import json
import math
import struct
import threading
import time
from pathlib import Path

CONTROLLER = Path('/home/sonu/Desktop/precision test/absolute_alignment_sequence_lut.py')
spec = importlib.util.spec_from_file_location('working_precision_controller', CONTROLLER)
rig = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rig)
from canstepper.protocol import Tel

SPEED = 1440.0
ROTATION_MICROSTEPS = {2: 128, 3: 32}
PULSE_LIMIT_HZ = 200000
SNAPSHOT = (
    rig.Param.MICROSTEPS, rig.Param.CLOSED_LOOP, rig.Param.INVERT_DIR,
    rig.Param.PID_KP, rig.Param.PID_KI, rig.Param.PID_KD,
    rig.Param.PID_TOLERANCE, rig.Param.MAX_SPEED, rig.Param.ACCELERATION,
    rig.Param.CL_MAX_SPEED, rig.Param.CL_MAX_ACCEL,
    rig.Param.FAST_RATE_HZ, rig.Param.SLOW_RATE_HZ,
)


def install_temporary_profile(controller):
    base = controller.NODE2_STOP_PROFILES[720.0]
    controller.NODE2_STOP_PROFILES[SPEED] = {
        'speed_deg_s': SPEED,
        'microsteps': ROTATION_MICROSTEPS[2],
        'pid_gains': base['pid_gains'],
        'pid_tolerance_deg': base['pid_tolerance_deg'],
        'offset_deg': base['offset_deg'],
        'status': 'experimental-speed-and-microstep-extrapolation',
    }
    controller.NODE2_SPEED_LUT[SPEED] = base['offset_deg']


def check_guard(node, reference, cpr, controller=rig):
    status = node.get_status()
    error = -controller.absolute_error_deg(node, reference, cpr)
    limit = (0.0 if node.node_id == 2
             else controller.SLOT_SAFETY_TOLERANCE_DEG[3])
    if not status.enabled or status.moving or abs(error) > limit:
        raise RuntimeError(
            f'guard node {node.node_id} unsafe: enabled={status.enabled} '
            f'moving={status.moving} error={error:+.9f} limit={limit:g}'
        )
    return error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cycles', type=int, choices=(1, 10), default=10)
    args = parser.parse_args()
    install_temporary_profile(rig)
    offsets = {2: rig.NODE2_SPEED_LUT[SPEED], 3: rig.LUT[3]['offset_deg']}
    data = json.loads(rig.REFERENCE_PATH.read_text())
    cpr = int(data['encoder_counts_per_revolution'])
    refs = {int(k): int(v['aligned_raw_count']) for k, v in data['nodes'].items()}
    bus = rig.CANStepperBus.serial(rig.PORT)
    nodes = {i: bus.node(i) for i in (2, 3)}
    saved = {}
    rows = []
    failure = ''
    disabled = {2: False, 3: False}
    lock = threading.Lock()
    active = None
    velocities = []

    def on_motion(frame):
        if len(frame.data) != 8:
            return
        velocity, _ = struct.unpack('<ff', frame.data)
        with lock:
            if frame.node_id == active and math.isfinite(velocity):
                velocities.append(abs(velocity))

    bus.subscribe(None, Tel.MOTION, on_motion)
    try:
        time.sleep(2.5)
        for i, node in nodes.items():
            status = node.get_status()
            rig.healthy(node)
            error = -rig.absolute_error_deg(node, refs[i], cpr)
            print(f'PREFLIGHT node={i} firmware={status.firmware} '
                  f'enabled={status.enabled} moving={status.moving} '
                  f'slot_error={error:+.9f}', flush=True)
            if status.enabled or status.moving or abs(error) > data['max_automatic_correction_deg']:
                raise RuntimeError(f'node {i} preflight failed; no large automatic recovery allowed')
            if node.get_param(rig.Param.ZERO_ON_BOOT) or node.get_param(rig.Param.ENABLE_ON_BOOT):
                raise RuntimeError(f'node {i} boot settings do not match the taught reference')
            steps = node.get_param(rig.Param.STEPS_PER_REV)
            worst_hz = SPEED * 1.15 * steps * ROTATION_MICROSTEPS[i] / 360
            if worst_hz > PULSE_LIMIT_HZ:
                raise RuntimeError(f'node {i} pulse demand {worst_hz:g} exceeds conservative limit')
            saved[i] = {p: node.get_param(p) for p in SNAPSHOT}
            print(f'PULSE_CHECK node={i} max_with_PID_headroom={worst_hz:g}Hz '
                  f'limit={PULSE_LIMIT_HZ}', flush=True)
        for i, node in nodes.items():
            node.set_param(rig.Param.MICROSTEPS, 32)
            node.set_param(rig.Param.CLOSED_LOOP, 1)
            node.set_param(rig.Param.INVERT_DIR, 0)
            node.set_pid(*rig.PID_GAINS[i], tolerance_deg=rig.MOTION_TOLERANCE_DEG)
            node.set_report_rates(100, 2)
        # Establish the stationary clearance guard before the node-2 correction.
        bases = {3: rig.align_node(nodes[3], refs[3], cpr, data['max_automatic_correction_deg'])}
        bases[2] = rig.align_node(nodes[2], refs[2], cpr, data['max_automatic_correction_deg'])
        # Unconditionally use the approved fine microstepping before every
        # node-2 finish, including an arrival that initially appears exact.
        original_finish = rig.finish_node2_at_slot

        def fine_finish(node, target, reference, counts_per_rev):
            node.set_param(rig.Param.MICROSTEPS, rig.NODE2_FINAL_MICROSTEPS)
            return original_finish(node, target, reference, counts_per_rev)

        rig.finish_node2_at_slot = fine_finish
        fine_finish(nodes[2], bases[2], refs[2], cpr)
        targets = dict(bases)
        print(f'SETTINGS cycles={args.cycles} speed={SPEED:g} accel={SPEED*4:g} '
              f'node2_PID={rig.NODE2_STOP_PROFILES[SPEED]["pid_gains"]} '
              f'node3_PID={rig.PID_GAINS[3]} LUT={offsets} '
              f'rotation_microsteps={ROTATION_MICROSTEPS} '
              f'node2_finish_microsteps={rig.NODE2_FINAL_MICROSTEPS} '
              'added_dwell=0 logs=terminal_only flash_save=no', flush=True)
        previous_finish = None
        for cycle in range(1, args.cycles + 1):
            for i in (2, 3):
                other = 3 if i == 2 else 2
                for node in nodes.values():
                    rig.healthy(node)
                    if node.get_driver_status().thermal_warning:
                        raise RuntimeError(f'node {node.node_id} driver temperature warning')
                # Never release node 3 on the MOVE_DONE event alone. If node 2
                # has drifted since its settled finish, correct it first.
                if i == 3 and rig.absolute_error_deg(nodes[2], refs[2], cpr) != 0:
                    fine_finish(nodes[2], targets[2], refs[2], cpr)
                check_guard(nodes[other], refs[other], cpr)
                targets[i] -= 360.0
                with lock:
                    active = i
                    velocities = []
                started = time.monotonic()
                handoff = None if previous_finish is None else started - previous_finish
                print(f'MOVE_START cycle={cycle}/{args.cycles} node={i} '
                      f'speed={SPEED:g} handoff_setup_s={handoff}', flush=True)
                result = rig.move_one_revolution(
                    nodes[i], targets[i], offsets[i], SPEED, refs[i], cpr,
                )
                # Confirm the rotating disc is itself a valid stationary guard.
                check_guard(nodes[i], refs[i], cpr)
                previous_finish = time.monotonic()
                with lock:
                    active = None
                    peak = max(velocities, default=0.0)
                    count = len(velocities)
                row = {'cycle': cycle, 'node': i, 'requested_speed': SPEED,
                       'measured_peak_deg_s': peak, 'velocity_samples': count,
                       'handoff_setup_s': handoff, **result}
                rows.append(row)
                print('MOVE_RESULT ' + json.dumps(row), flush=True)
        print('SEQUENCE_COMPLETE', flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        failure = f'{type(exc).__name__}: {exc}'
        print(f'ABORT {failure}', flush=True)
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
            time.sleep(.2)
        for i, params in saved.items():
            for p, value in params.items():
                try:
                    nodes[i].set_param(p, value)
                except Exception as exc:
                    failure = failure or f'restore failed node {i}: {exc}'
                    print(f'RESTORE_ERROR node={i}: {exc}', flush=True)
        for i, node in nodes.items():
            try:
                disabled[i] = not node.get_status().enabled
                print(f'AFTER_DISABLE node={i} '
                      f'slot_error={-rig.absolute_error_deg(node, refs[i], cpr):+.9f}', flush=True)
            except Exception:
                disabled[i] = False
        bus.close()
    passed = not failure and len(rows) == args.cycles * 2 and all(disabled.values())
    print(f'SUMMARY result={"PASSED" if passed else "FAILED"} '
          f'completed_moves={len(rows)}/{args.cycles*2} disabled={disabled} '
          f'failure={failure!r} log_files_saved=no flash_save=no', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
