# Precision test calibration summary

Date: 2026-09-29
Port: `/dev/ttyACM0`
Firmware: 1.7, GCSP protocol 1
Supply: 24 V

## Node mapping

- Node 2: lower motor
- Node 3: elevated motor
- Both shafts rotate anticlockwise for positive motion commands when viewed
  directly from their shaft/disc-mounting faces.
- The precision-test application must therefore use negative motion commands
  for clockwise rotation.

## Persisted PID configuration

Both nodes completed the firmware-matched PID quick calibration and each
reported 13 passed checks, 0 failed checks. Each node acknowledged that its
tune was persisted.

- `invert_dir = 0`
- `closed_loop = 1`
- `run_current = 70%`
- `hold_current = 20%`
- `microsteps = 8`
- `stealthchop = 0` (SpreadCycle)
- `cl_max_speed = 720 deg/s`
- `cl_max_accel = 2880 deg/s^2`
- `pid_kp = 12.0`
- `pid_ki = 0.3`
- `pid_kd = 0.1`
- `pid_tolerance = 0.5 deg`

## Calibration results

### Node 2

- Polarity result: `invert_dir=0`
- Settle matrix: 6/6 passed
- Targets: +90, 0, -90, 0, +180, 0 degrees
- Worst recorded final error: -0.483 degrees
- Driver and encoder checks passed

### Node 3

- Polarity result: `invert_dir=0`
- Settle matrix: 6/6 passed
- Targets: +90, 0, -90, 0, +180, 0 degrees
- Worst recorded final error: -0.396 degrees
- Driver and encoder checks passed

## Direction and revolution checks before calibration

- Node 2 clockwise command -360 degrees: measured -360.220 degrees
  (error -0.220 degrees)
- Node 3 clockwise command -360 degrees: measured -359.956 degrees
  (error +0.044 degrees)
- No driver faults occurred.

## Safety state

The calibration tool issued a bus-wide emergency stop at completion and
disabled the selected motor after each run. The discs were not installed.

## Verification note

Post-save readback completed successfully for both nodes. Both returned the
persisted values listed above. Firmware remained 1.7 / protocol 1, encoder
status was healthy, TMC UART communication was healthy, and neither driver
reported a short or thermal fault. Both motors were disabled after readback.

## Runtime PID optimization (2026-09-30)

The plotter uses three synchronized Matplotlib panels for each run: commanded
and measured position versus time, commanded and measured velocity versus
time, and tracking/final-target error versus time. Each plot includes its
Kp, Ki, Kd, speed, acceleration and response metrics. Raw telemetry and
summary metrics remain in CSV for repeatable comparison.

The original persistent gains were not overwritten. The sequence controller
applies these optimized gains at runtime:

- Node 2: Kp 12.0, Ki 0.05, Kd 0.35
- Node 3: Kp 10.0, Ki 0.0, Kd 0.4
- Position tolerance: 0.5 degrees

At 720 deg/s, the original Kp 12.0, Ki 0.3, Kd 0.1 response had:

- Node 2: 1.579 s settle time and 20.435 deg overshoot
- Node 3: 1.377 s settle time and 20.588 deg overshoot

Repeated 720 deg/s validation of the optimized settings produced:

- Node 2 (5/5 passed): mean settle 1.000 s, mean overshoot 2.246 deg,
  final error range +0.176 to +0.374 deg
- Node 3 (5/5 passed): mean settle 1.182 s, mean overshoot 0.127 deg,
  final error range -0.110 to +0.176 deg

A final five-cycle alternating rig test at 720 deg/s passed all 10 moves.
Every return error was within +/-0.330 deg. Handoffs were event-driven with
no programmed dwell. Both drivers were disabled and verified disabled after
the run. No optimized gains have been saved to board flash pending approval.
