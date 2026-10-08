# Two-Stepper Precision Test: PID + LUT Implementation

Documentation date: 3 October 2026, Asia/Kolkata. Implementation and physical test results covered: the working programs and the 1 October 2026 sessions.

This is an implementation guide, not a new motor test or a firmware update. Creating this document did not move either motor, change the taught alignment, or alter the PID/LUT programs. Full Python source snapshots are included in Appendices A–C.

## Contents

1. [Purpose and control architecture](#1-purpose-and-control-architecture)
2. [Files and dependencies](#2-files-and-dependencies)
3. [Implemented settings](#3-implemented-settings)
4. [Step 1 — Establish the physical reference](#4-step-1--establish-the-physical-reference)
5. [Step 2 — Convert counts and calculate alignment](#5-step-2--convert-counts-and-calculate-alignment)
6. [Step 3 — Preflight and initial alignment](#6-step-3--preflight-and-initial-alignment)
7. [Step 4 — Generate one-revolution targets](#7-step-4--generate-one-revolution-targets)
8. [Step 5 — Calibrate and validate the LUT](#8-step-5--calibrate-and-validate-the-lut)
9. [Step 6 — Execute board-side PID motion](#9-step-6--execute-board-side-pid-motion)
10. [Step 7 — Select speed, acceleration and microsteps](#10-step-7--select-speed-acceleration-and-microsteps)
11. [Step 8 — Finish node 2 at the true reference](#11-step-8--finish-node-2-at-the-true-reference)
12. [Step 9 — Alternate the motors safely](#12-step-9--alternate-the-motors-safely)
13. [Step 10 — Stop, disable and restore](#13-step-10--stop-disable-and-restore)
14. [Commands to reproduce the workflow](#14-commands-to-reproduce-the-workflow)
15. [Output fields and timing definitions](#15-output-fields-and-timing-definitions)
16. [Recorded test results](#16-recorded-test-results)
17. [Tests and known limitations](#17-tests-and-known-limitations)
18. [Source provenance and version checks](#18-source-provenance-and-version-checks)
19. [Appendix A — Full baseline controller](#appendix-a--full-baseline-controller)
20. [Appendix B — Full 1440deg-s-alternating-runner](#appendix-b--full-1440deg-s-alternating-runner)
21. [Appendix C — Full five-trial calibration program](#appendix-c--full-five-trial-calibration-program)

## 1. Purpose and control architecture

The rig has two stepper motors with encoder-equipped driver boards, linked by CAN. One board provides the laptop's USB connection. The supply used during the tests was 24 V.

Node 2 is the lower motor; node 3 is the elevated motor. Their shafts/disc planes are perpendicular and their positions are offset. The user confirmed equal disc dimensions/thickness and radial gaps. This document does not invent disc diameters, slot widths, mechanical clearances, or tolerances in millimetres.

The required motion is sequential, not simultaneous:

```text
Taught/aligned discs
    -> node 2: one clockwise revolution
    -> node 2: stop and verify its radial-gap reference
    -> node 3: one clockwise revolution
    -> node 3: stop and verify its reference within its existing tolerance
    -> repeat
```

One alternating cycle contains two rotations: one by each node. Ten cycles therefore contain twenty rotations.

The control has four separate responsibilities:

| Component | Location | Responsibility |
| --- | --- | --- |
| Taught reference | Python-side JSON file | Stores the encoder phase corresponding to each manually aligned radial gap |
| LUT | Python dictionaries | Adds a measured endpoint correction to the commanded angle |
| Encoder PID controller | Driver-board firmware | Drives the shaft towards the commanded angle |
| Finish/guard logic | Python host | Checks the actual encoder reference and decides whether the other disc may move |

```text
Python: physical slot target + LUT offset
    -> USB serial bridge on one of the two boards
    -> node-addressed CAN command
    -> selected board's encoder-feedback PID controller
    -> step pulses -> motor
                       |
                       +-> encoder -> board PID feedback
                                  -> CAN/USB telemetry -> Python guard
```

The LUT does not replace PID. Python is not implementing the fast inner PID loop. With Ki = 0, the working node-2 motion controller is a PD special case of the configurable PID structure; closed-loop encoder control is still enabled.

## 2. Files and dependencies

The working Desktop directory is `/home/sonu/Desktop/precision test`.

| File | Role |
| --- | --- |
| `absolute_alignment_sequence_lut.py` | Working 360/720deg/s controller; baseline for imported helper functions |
| `alignment_reference_lut.json` | Active taught physical reference; not the similarly named older reference files |
| `pid_lut_alternating_1440.py` | Separate higher-speed experiment, importing the Desktop baseline without rewriting it |
| `absolute_alignment_sequence_lut_before_1440_20261001.py` | Backup of the baseline before the 1440deg/s experiment |
| `absolute_alignment_sequence_lut_before_node2_recalibration_20261001.py` | Earlier stopping-profile backup |
| `pid optimization/absolute_alignment_sequence.py` | Original PID-only program; not replaced by this implementation |

Workspace sources are under `/home/sonu/Grafito-Edge-Services/can_stepper`:

- `standalone_absolute_alignment_sequence_lut.py`: baseline development copy; identical to the Desktop baseline at documentation time.
- `pid_lut_alternating_1440.py`: development copy of the separate runner.
- `node2_lut_calibration.py`: five-trial calibration/verification helper.
- `pid_lut_speed_limit_probe.py`: imported dependency used by the calibration helper to obtain the Desktop controller and its parameter snapshot list. Its old standalone speed-probe workflow is not the current recommended test.
- `tests/test_node2_precision_finish.py` and `tests/test_alternating_1440.py`: host-logic tests.

The baseline inserts `/home/sonu/Downloads/grafito-canstepper-station` into `sys.path` and imports `CANStepperBus` and `Param` from that Python package. The scripts use `/dev/ttyACM0`. They require the existing station package and its serial dependencies; a Python source snapshot alone is not a self-contained hardware driver.

Important path dependency: the 1440deg/s runner explicitly imports the Desktop baseline by absolute path. Moving only the runner elsewhere does not change which controller it imports. Other copied or archived `precision test` folders are not automatically the active source.

## 3. Implemented settings

All angles are in degrees, speeds in degrees/second, and accelerations in degrees/second squared.

| Stage | Node | Kp | Ki | Kd | PID tolerance | Microsteps |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Startup alignment | 2 | 12 | 0.05 | 0.35 | 0.5deg | 32 |
| Startup alignment | 3 | 10 | 0 | 0.4 | 0.5deg | 32 |
| Validated 360/720deg/s motion | 2 | 12 | 0 | 0.35 | 0.012deg | 256 |
| Experimental 1440deg/s motion | 2 | 12 | 0 | 0.35 | 0.012deg | 128 |
| Precision finish | 2 | 12 | 0 | 0.35 | 0.012deg | 256 |
| Main rotation | 3 | 10 | 0 | 0.4 | 0.5deg | 32 |

The initial settings are intentionally distinguished from the main motion profile. Reading only the global `PID_GAINS[2]` tuple, `(12, 0.05, 0.35)`, misses the speed-specific profile that overrides Ki to zero before the working node-2 revolution.

| Setting | Implemented value |
| --- | --- |
| Encoder counts per revolution, N | 16,384 |
| Reported full steps/revolution, S | 200 |
| Node-2 applied LUT at 360/720 and in the separate 1440 runner | -0.00439453125deg |
| Node-3 applied LUT | -0.25927734375deg |
| Node-2 physical arrival abort limit | 0.65deg |
| Node-3 physical arrival/guard limit | 0.5deg |
| Node-2 final encoder gate | Exactly 0 counts from the taught phase |
| Maximum startup correction in calibration/1440 runner | 2.5deg |
| Startup/finish speed setting | 30deg/s |
| Node-2 correction acceleration | 720deg/s² |
| Maximum node-2 fine correction attempts | 3 |
| Blocking move timeout | 6 seconds per commanded correction/revolution |
| Post-finish wait | 0.15 seconds before the fresh encoder checks |
| Fine gate readings | 3, with a 0.02-second sleep after each reading |
| Calibration readings | 5, with a 0.02-second sleep after each reading |
| 1440 runner requested report rates | Fast 100 Hz, slow 2 Hz |

The physical limits, the startup window, the board PID tolerance and the final encoder gate are different things. A 0.65deg node-2 arrival limit is not permission to hand off with that error: node 2 must subsequently pass the zero-count gate. These thresholds were used on this rig; they are not universal collision-clearance specifications.

The baseline retains an old fallback node-2 LUT of +0.1373291015625deg for other, unprofiled speeds. It is experimental and is not the value used at 360, 720, or by the separate 1440 runner. LUT selection is an exact speed-key lookup, not interpolation.

## 4. Step 1 — Establish the physical reference

The user manually aligned the radial gaps, then confirmed alignment. The active JSON records encoder phase counts, not a command that resets the encoder:

```json
{
  "encoder_counts_per_revolution": 16384,
  "max_automatic_correction_deg": 2.5,
  "nodes": {
    "2": {"aligned_raw_count": 10571, "aligned_raw_deg": 232.27294921875},
    "3": {"aligned_raw_count": 5075, "aligned_raw_deg": 111.51123046875}
  }
}
```

This is an excerpt of the existing reference file, not instructions to overwrite it.

The programs do not call `SET_ZERO` or `SET_POSITION`. Therefore the taught reference is not silently redefined at the start of every test. Current multi-turn encoder counts can be negative or many revolutions away; their phase modulo N is compared with the taught phase.

Mechanical slippage between the disc and shaft, moving the encoder magnet, swapping motors/node IDs, or changing relevant encoder zero configuration can invalidate the reference. Automatic correction cannot repair an incorrect physical teaching. Do not overwrite the reference merely to hide an error.

## 5. Step 2 — Convert counts and calculate alignment

### Equation 1 — Encoder count size

```text
q = 360 / N
  = 360 / 16384
  = 0.02197265625deg per count
```

Zero count error means matching the taught encoder code. It does not prove zero physical angular error or metrology accuracy finer than the encoder resolution.

### Equation 2 — Shortest signed phase correction

Let C be the current multi-turn count, R the taught phase count, and N the counts/revolution:

```text
delta = ((R - (C mod N) + N/2) mod N) - N/2
```

The result is the shortest signed count correction, in `[-N/2, N/2)`. Exactly half a turn has an ambiguous direction; the code chooses the negative representative. Normal startup correction is restricted to a much smaller window.

Actual source:

```python
def signed_modular_delta(reference: int, current: int, cpr: int) -> int:
    return (reference - (current % cpr) + cpr // 2) % cpr - cpr // 2
```

### Equation 3 — Correction and observed slot error

```text
correction_deg = delta * q             # desired minus measured
slot_error_deg = -delta * q            # measured minus desired
```

The function `absolute_error_deg()` returns the signed correction. Despite its name, it is not an unsigned magnitude. Some printed fields negate it to report measured-minus-reference error.

Example: current phase 10570, reference 10571 gives delta = +1 count. The required correction is +0.02197265625deg, and the actual slot error is -0.02197265625deg.

### Equation 4 — Nearby multi-turn reference target

```text
aligned_target_deg = (C + delta) * q
```

Actual source:

```python
def alignment_target(node, reference: int, cpr: int) -> tuple[float, float, int]:
    current = node.get_encoder_counts()
    delta = signed_modular_delta(reference, current, cpr)
    return (current + delta) * 360.0 / cpr, delta * 360.0 / cpr, current
```

This chooses the nearby copy of the taught reference in the current multi-turn coordinate system. It avoids commanding an unnecessary long return to the first-turn angle.

## 6. Step 3 — Preflight and initial alignment

The 1440 runner implements these steps before full rotations:

1. Connect to the serial bridge and allow 2.5 seconds for initial communication.
2. Read both nodes' status and driver health.
3. Reject a node already enabled/moving, unhealthy, or outside the 2.5deg startup correction window.
4. Require `ZERO_ON_BOOT = 0` and `ENABLE_ON_BOOT = 0` to match the teaching conditions.
5. Read full steps/revolution and check pulse demand including speed headroom.
6. Snapshot runtime parameters for later restoration.
7. Enable closed-loop operation, use the known direction setting, configure startup PID, and request telemetry rates.
8. Align node 3 first, so it becomes the stationary clearance guard.
9. Align node 2 to the nearby taught reference.
10. Perform node 2's fine encoder finish before the alternating sequence begins.

Startup `align_node()` enables a node, waits 0.4 seconds, calculates the shortest correction, applies the 30deg/s motion limit and sends a blocking absolute move. It checks the residual against 0.5deg and checks health. The separate node-2 fine finish then tightens the actual reference requirement to zero counts.

The baseline controller also contains an older single-disc recovery branch that can permit a correction up to 180deg when the other disc is in its startup window. The calibration helper and 1440 runner do not use that large-recovery branch. A software window is not a geometric proof of collision clearance; the small-window startup used here should not be confused with unrestricted homing.

## 7. Step 4 — Generate one-revolution targets

Clockwise on this rig corresponds to decreasing commanded angle. Direction was visually confirmed during the previous tests; `INVERT_DIR` is configured to zero in these programs.

### Equation 5 — Physical target for cycle k

```text
T_slot(i, k) = B_i - 360*k
```

`B_i` is the nearby aligned multi-turn target for node i. The runner implements this incrementally:

```python
targets = dict(bases)
# Each time node i is selected:
targets[i] -= 360.0
```

An absolute target is used instead of repeatedly redefining zero. Each motor has its own target sequence.

### Equation 6 — LUT-adjusted command

```text
T_command(i, k, speed) = T_slot(i, k) + L_i(speed, profile)
```

`L_i` is an algebraic angle offset, not a multiplier on speed or PID gain. For node 2, the current value is -0.00439453125deg. For node 3 it is -0.25927734375deg.

Actual command logic:

```python
command_target = slot_target + lut_offset
node.move_to(command_target, blocking=True, timeout=6.0)
```

Success against `T_command` is not the same as success against `T_slot`. The node-2 finishing stage explicitly commands the unshifted physical target when correction is required.

## 8. Step 5 — Calibrate and validate the LUT

### Measurement algorithm

The calibration helper performs exactly five node-2 full turns per invocation, holding node 3 as the clearance guard:

1. Configure a specified speed, microstep count, PID tolerance and Ki.
2. Rotate node 2 to `T_slot + L_old` using board-side closed-loop PID.
3. Read the immediate encoder count after the completion response.
4. Wait 0.15 seconds.
5. Read five fresh encoder counts, with a 0.02-second sleep after each.
6. Convert them to signed measured-minus-reference count errors.
7. Abort if any sampled error exceeds the node-2 physical limit of 0.65deg.
8. Use the median of the five errors as that trial's calibration measurement.
9. If requested, perform the separate exact finish afterwards.
10. Calculate mean, sample standard deviation, maximum absolute median error and a new LUT candidate after five trials.

The calibration measurements are taken before fine correction. Learning the LUT from the zero errors after fine correction would hide the stopping bias being measured.

### Equations 7–10 — Statistics and LUT update

For trial j, with five raw samples indexed by m:

```text
e_j = median_m(-delta(R, C_jm, N)) * q

mean_error = (e_1 + e_2 + ... + e_n) / n

sample_stddev = sqrt(sum_j((e_j - mean_error)^2) / (n - 1))

L_candidate = L_old - mean_error
```

Here n = 5. A positive error means the shaft stopped above the physical target, so the next command is shifted downward. The update assumes locally near-unit endpoint response to a small command change. Quantization, microstep phase, load and settling behaviour mean it is only a candidate until physically verified.

Actual statistics code:

```python
mean = statistics.mean(errors)
deviation = statistics.stdev(errors)
candidate = args.offset - mean
```

### Why mean error alone was insufficient

The initial coarse profile had sign-changing errors. Its first five 360deg/s trials, using a +0.1373291015625deg offset, gave:

```text
[-0.087890625, 0, +0.06591796875, +0.2197265625, 0]deg
mean = +0.03955078125deg
candidate = +0.0977783203125deg
```

Five candidate-verification trials still included a +0.3955078125deg stopping error. Reducing the average did not remove the variable spread. That coarse candidate was not installed as a successful correction.

Finer microstepping, a smaller PID tolerance and a zero integral gain were then compared. Tight profiles with Ki = 0.05 sometimes reached exact encoder codes but timed out in later trials. Ki = 0 allowed the tested batch to complete, although small post-completion drift remained. This is experimental evidence for the chosen profile, not proof that integral windup was the only physical cause, and not a formal auto-tuning or Ziegler–Nichols procedure.

### Worked calculation for the installed node-2 LUT

With the final profile `(Kp, Ki, Kd) = (12, 0, 0.35)`, 256 microsteps, 0.012deg tolerance, and `L_old = +0.017578125deg`, five 360deg/s calibration arrivals were each one encoder count high:

```text
e_j = +0.02197265625deg for j = 1...5
mean_error = +0.02197265625deg
L_candidate = +0.017578125 - 0.02197265625
            = -0.00439453125deg
```

Validation of that value at 360deg/s gave four direct zero-count returns and one -1-count return that the fine finish corrected to zero. Five final checks were therefore zero. The installed baseline was also exercised for five node-2-only cycles and passed all final checks, with correction retries in the first cycle.

At 720deg/s, five final checks passed; trial 3 drifted one count after completion and required correction. Its five-sample median was zero even though the last two samples had shifted. A median and a printed standard deviation of zero are therefore not proof that every raw sample or every future arrival is identical.

The helper only prints candidates. It does not automatically rewrite LUT dictionaries, reteach the reference, or save to flash. The validated value was intentionally installed after testing. The 1440 runner reuses it as an experimental speed/microstep extrapolation; it does not recalibrate it.

## 9. Step 6 — Execute board-side PID motion

### Firmware provenance boundary

The boards reported firmware 1.7 during the physical sessions. The available local firmware source files currently declare version 1.9. Consequently, the following detailed inner-loop equations describe the available 1.9 source, which helps explain the architecture; they are not a verified reconstruction of the exact flashed 1.7 binary. No 1.9 firmware was flashed as part of the LUT tests or this documentation task.

The host behaviour documented elsewhere is directly verified against the Python source and recorded terminal responses. Exact inner-loop equivalence would require the matching 1.7 source/build or a separately approved firmware investigation.

### PID/trajectory equations in the available source

For a position move, the available firmware plans a rest-to-rest trajectory. Let r(t) be its position reference, v_ff(t) its velocity feedforward, and theta(t) measured encoder angle.

```text
e_track[k] = r[k] - theta[k]
e_target[k] = T_command - theta[k]

v_raw[k] = (theta[k] - theta[k-1]) / dt
v_filtered[k] = v_filtered[k-1]
                + 0.30*(v_raw[k] - v_filtered[k-1])

I_candidate[k] = I[k-1] + e_track[k]*dt
trim[k] = Kp*e_track[k] + Ki*I_candidate[k] - Kd*v_filtered[k]
v_command[k] = v_ff[k] + trim[k]
```

The derivative acts on measured velocity rather than a numerical derivative of the changing reference error. This avoids a derivative kick when the reference changes. With Ki = 0, the integral candidate is set to zero in this available source.

Relevant C++ excerpt from that source:

```cpp
float integ = pidIntegral + trackErr * dt;
if (ki > 1e-6f) {
  float ilim = (0.20f * vmax) / ki;
  integ = constrain(integ, -ilim, ilim);
} else {
  integ = 0.0f;
}
float trim = kp * trackErr + ki * integ - kd * pidVelFilt;
float trimCap = fmaxf(0.25f * vmax, 40.0f);
trim = constrain(trim, -trimCap, trimCap);
float out = vFf + trim;
float outLim = vmax * 1.15f;
out = constrain(out, -outLim, outLim);
```

The available source also includes conditional integral accumulation to reduce windup, acceleration/deceleration rate limits, reversal handling, encoder-loss handling and a no-progress watchdog. Its control period is nominally 5 ms (200 Hz), separate from the host's requested reporting rate.

Its settle conditions include plan completion, target error within PID tolerance, and sufficiently low filtered velocity for an 80 ms interval. The source uses `max(12, 0.06*vmax)` deg/s as that velocity threshold. It changes the PID state to settled and emits `MOVE_DONE`; it does not establish an independently verified, continuously active zero-error position hold afterwards. These source-level details are version-qualified, not assumptions about the exact 1.7 build.

### CAN command and completion handling

The station protocol defines the node-addressed CAN identifier as:

```text
CAN_ID = (node_id << 6) | message_id
```

`MOVE_ABS` has message ID 4 and an eight-byte little-endian float64 target angle. `ENC_COUNTS` returns signed 64-bit multi-turn counts. `MOTION` carries two float32 values: measured velocity and position error. The SDK arms the completion/fault event waiter before sending a blocking move, so a fast event is not missed.

```python
with self._bus.arm_events(
    self.node_id, [Event.MOVE_DONE, Event.FAULT, Event.ESTOP]
) as waiter:
    self._cmd(Cmd.MOVE_ABS, struct.pack("<d", float(degrees)))
    self._wait_motion_event(waiter, timeout, "move")
```

The host sends an individual node's move. It does not broadcast the same full-turn command to both motors, use FOLLOW mode, or start simultaneous rotations. A completion event is necessary for this workflow, but raw-reference validation is also necessary because small subsequent drift was observed.

## 10. Step 7 — Select speed, acceleration and microsteps

### Equation 11 — Host acceleration policy

```text
a = 4*v_setting
```

Actual source applies the same speed ceiling to `MAX_SPEED` and `CL_MAX_SPEED`, and the same acceleration to `ACCELERATION` and `CL_MAX_ACCEL`:

```python
def set_motion_limits(node, speed: float) -> None:
    accel = speed * 4.0
    node.set_param(Param.MAX_SPEED, speed)
    node.set_param(Param.ACCELERATION, accel)
    node.set_param(Param.CL_MAX_SPEED, speed)
    node.set_param(Param.CL_MAX_ACCEL, accel)
```

### Equations 12–13 — Planned peak and ideal motion time

For a symmetric rest-to-rest move of distance D:

```text
v_peak_planned = min(v_setting, sqrt(a*D))

If D <= v_setting^2/a:     # triangular profile
    t_ideal = 2*sqrt(D/a)
Else:                     # trapezoidal profile
    t_ideal = D/v_setting + v_setting/a
```

These expressions agree with the available firmware trajectory planner. They describe ideal planned motion, not guaranteed actual board timing. For D approximately 360deg and a = 4*v_setting:

| Speed setting | Acceleration | Ideal planned peak | Ideal trajectory time |
| ---: | ---: | ---: | ---: |
| 360deg/s | 1440deg/s² | 360deg/s | 1.25 s |
| 720deg/s | 2880deg/s² | 720deg/s | 0.75 s |
| 1440deg/s | 5760deg/s² | 1440deg/s | 0.50 s |

The LUT offset and actual starting residual slightly change D. PID tracking, motor torque/load, settling, corrections and communications make the measured elapsed time longer. A rest-to-rest one-turn command at 1440deg/s is not constant-speed 1440deg/s motion for the entire revolution.

### Equation 14 — Nominal microstep angle

```text
microstep_angle = 360 / (S*M)
```

With S = 200 full steps/revolution:

| Microsteps, M | Nominal angular command increment |
| ---: | ---: |
| 32 | 0.05625deg |
| 128 | 0.0140625deg |
| 256 | 0.00703125deg |

Microstepping increases nominal command resolution. It does not guarantee equally accurate mechanical motion under load. Encoder resolution remains q = 0.02197265625deg/count.

### Equations 15–16 — Pulse demand and conservative limit

```text
f_step = abs(v)*S*M/360
f_with_headroom = 1.15*v_setting*S*M/360
```

The runner checks the second expression against 200,000 pulses/s. The installed ESP32 stepper library source uses 16,000,000 ticks/s and a default minimum interval of 80 ticks, implying 200,000 pulses/s. This is a conservative planning limit from the inspected local library, not a measured maximum of the flashed 1.7 build or a guarantee of achievable motor speed.

At 1440deg/s with S = 200:

| Node/stage | Microsteps | Nominal pulses/s | With 15% headroom |
| --- | ---: | ---: | ---: |
| Node 2 if the old 256-microstep cruise were kept | 256 | 204,800 | 235,520 |
| Node 2 approved high-speed rotation | 128 | 102,400 | 117,760 |
| Node 3 unchanged rotation | 32 | 25,600 | 29,440 |
| Node 2 finish at 30deg/s | 256 | 4,266.67 | 4,906.67 |

This is why the user approved 128 microsteps during node-2 high-speed rotation, returning to 256 for the precision finish. PID gains, LUT values and tolerances were not retuned. Changing microsteps changes the motion profile, so the 1440 runner labels the reused LUT as experimental extrapolation and uses a qualification cycle first.

## 11. Step 8 — Finish node 2 at the true reference

This is the main algorithm that makes node-2 endpoint acceptance stricter than the board's coarse completion event.

```text
Read encoder phase relative to the taught reference
    -> If off-reference: configure fine profile and command T_slot
    -> Wait 0.15 s
    -> Read three fresh raw encoder samples
    -> If all three errors are exactly zero counts: pass
    -> Otherwise retry a bounded correction
    -> After at most three correction commands: abort if still not zero
```

The fine profile uses 256 microsteps, `(12, 0, 0.35)` gains, 0.012deg PID tolerance, speed limit 30deg/s and acceleration 720deg/s². It commands the true slot target, not the LUT-adjusted target. The 1440 runner wraps the baseline finish to select 256 microsteps unconditionally before checking, even when the first reading initially appears exact.

Core actual acceptance code, executed inside the bounded retry loop:

```python
time.sleep(0.15)
counts = []
for _ in range(3):
    counts.append(node.get_encoder_counts())
    time.sleep(0.02)
errors = [-signed_modular_delta(reference, count, cpr) for count in counts]
if all(abs(error) <= NODE2_PRECISION_COUNTS for error in errors):
    break
```

`NODE2_PRECISION_COUNTS = 0`. The full retry implementation is included in Appendix A. Each correction command has its own six-second timeout; three allowed attempts do not mean a six-second total finish budget. A timeout, communication exception or failed final gate aborts rather than accepting an offset.

The three reads are spaced by the sleep plus communications time, not exactly 20 ms. The real zero-error check took about 0.35–0.38 seconds in the observed sessions even without an actual corrective move. Corrections can take additional seconds.

Node 3 does not currently have this exact-count fine-finishing algorithm. Its PID tolerance and guard limit remain 0.5deg. The implementation therefore does not promise identical endpoint accuracy for both nodes.

## 12. Step 9 — Alternate the motors safely

The separate runner temporarily adds a 1440deg/s profile to the imported module's in-memory dictionaries. It does not edit the baseline source on disk. This entry takes the tested node-2 PID/LUT values from the 720deg/s profile and changes only the speed label and rotation microsteps.

The qualification uses one alternating cycle. After it passed the existing checks, a separate invocation ran ten cycles.

The control algorithm below is explanatory pseudocode; Appendix B contains the full actual implementation:

```text
For cycle = 1 through requested_cycles:
    For active_node = 2, then 3:
        Read health and thermal status of both nodes
        If active_node is 3 and node 2 has drifted:
            Run the bounded node-2 fine finish again
        Check the other node is enabled, stationary and aligned
        Decrease the active node's absolute target by 360deg
        Run its LUT-adjusted blocking move
        Run node-2 fine finishing when active_node is 2
        Check the moved node is now a valid stationary guard
        Print result and measured peak speed
```

Actual stationary-guard limits:

```python
limit = (0.0 if node.node_id == 2
         else controller.SLOT_SAFETY_TOLERANCE_DEG[3])
if not status.enabled or status.moving or abs(error) > limit:
    raise RuntimeError(...)
```

The preceding revolution also checks physical arrival limits of 0.65deg for node 2 and 0.5deg for node 3 before accepting subsequent finishing/handoff.

There is no programmed inter-cycle dwell. However, this is not zero-latency switching: health requests, parameter writes, encoder verification and any corrections take time. The implementation never launches the second disc merely because a fixed predicted duration has elapsed.

These checks happen at discrete times. They do not continuously prove disc geometry, detect every intermediate collision possibility, or replace an independent emergency stop. Matching an encoder reference also cannot detect a disc slipping independently of its shaft.

## 13. Step 10 — Stop, disable and restore

On an exception, keyboard interruption, health failure, timeout or guard violation, the runners attempt a broadcast emergency stop. In `finally`, they:

1. Send disable to both motors twice, with a short separation.
2. Restore the captured runtime parameters.
3. Verify both nodes report disabled.
4. Report encoder error after disable.
5. Close the connection and print success/failure.

The 1440 runner snapshots microsteps, closed-loop mode, direction, gains, PID tolerance, motion limits and report rates. The baseline snapshots the parameters it changes but not report rates, which it does not change. Restoration errors are reported and make the run fail. Set-parameter acknowledgements are used; there is no separate exhaustive final readback comparison of every saved value.

No `SAVE_CONFIG`/flash write is called. No encoder-zero or reference reteaching command is called. Runtime PID changes are not a permanent board-flash calibration.

Disabling removes holding torque. The disc can relax even when its final enabled reading was exact. In the ten-cycle session, after-disable offsets were -0.0439453125deg for node 2 and +0.0439453125deg for node 3. That is distinct from the accepted enabled-state stopping error. A future run must preflight and correct again; powered-off alignment is not guaranteed.

## 14. Commands to reproduce the workflow

These commands are documentation only; none were executed to move hardware while producing this document. Motion commands require the same physical rig, valid taught references, clear slots, a verified guard, appropriate supply and a reachable emergency stop. Do not run two hardware scripts concurrently.

### A. Five node-2 trials using the working 360deg/s controller

```bash
cd '/home/sonu/Desktop/precision test'
python3 absolute_alignment_sequence_lut.py --node2-only --speed 360 --cycles 5 --accept-node2-experimental-risk
```

### B. Five node-2 trials at the tested 720deg/s setting

```bash
cd '/home/sonu/Desktop/precision test'
python3 absolute_alignment_sequence_lut.py --node2-only --speed 720 --cycles 5 --accept-node2-experimental-risk
```

### C. Qualification at the 1440deg/s setting

```bash
cd '/home/sonu/Desktop/precision test'
python3 pid_lut_alternating_1440.py --cycles 1
```

### D. Ten alternating cycles, after qualification passes

```bash
cd '/home/sonu/Desktop/precision test'
python3 pid_lut_alternating_1440.py --cycles 10
```

The baseline CLI is capped at 720deg/s and five cycles. The separate runner accepts one or ten cycles and fixes speed to 1440deg/s. Do not bypass the baseline caps to request this experiment; use the separate program. The baseline risk-acceptance flag remains required for LUT application, including the tested node-2 profiles; the separate experimental runner has its own explicit executable/qualification workflow.

### E. Five-trial calibration/verification helper, only when recalibration is intended

```bash
cd /home/sonu/Grafito-Edge-Services
python3 can_stepper/node2_lut_calibration.py --speed 360 --offset=-0.00439453125 --microsteps 256 --pid-tolerance 0.012 --ki 0 --precision-finish
```

Changing `--speed` to 720 selects the other supported calibration speed. Always specify the complete profile when reproducing the final configuration: the helper's historical defaults still select the older coarse microstep/tolerance/offset settings. It prints candidates but does not install them. It imports `pid_lut_speed_limit_probe.py` but does not execute that module's old high-speed sweep.

## 15. Output fields and timing definitions

The baseline prints settings, summary and CSV-formatted rows to standard output. The 1440 runner prints `PREFLIGHT`, `PULSE_CHECK`, `SETTINGS`, `MOVE_START`, `MOVE_RESULT`, `AFTER_DISABLE` and `SUMMARY` messages. It prints each result as JSON text; no JSON/CSV/TXT log files are created.

| Field | Definition |
| --- | --- |
| `slot_target_deg` | True unshifted physical slot target in multi-turn coordinates |
| `command_target_deg` | Slot target plus applied LUT correction |
| `lut_offset_deg` | Actual correction applied to this move |
| `slot_error_deg` | Measured position minus physical slot target |
| `command_error_deg` | Measured position minus LUT-adjusted target |
| `reference_error_deg` | Signed desired-minus-measured raw phase correction; sign differs from slot error |
| `initial_slot_error_deg` | First degree-position error after the main move, before node-2 finishing |
| `precision_finish_applied` | Whether a fine correction/retry was needed |
| `precision_finish_s` | Time spent inside the baseline fine-finish helper |
| `elapsed_s` | Main move and included fine finish, starting after main-motion configuration writes |
| `requested_speed` | Configured speed ceiling, not proven constant velocity |
| `measured_peak_deg_s` | Largest absolute MOTION velocity sample collected for the selected node during that move/finish |
| `velocity_samples` | Number of collected MOTION frames, not guaranteed uniformly spaced independent samples |
| `handoff_setup_s` | Time from previous completed guard check to the next pre-move timestamp |

The handoff timestamp is taken before `move_one_revolution()` reconfigures motion parameters and sends the next move. Thus `handoff_setup_s` does not include all the subsequent parameter-write time or exactly measure the physical interval between shafts moving. It is useful diagnostics, not a complete instant-switch guarantee.

The degree position read, raw encoder reads and guard reads are separate serial/CAN requests. Their values need not be identical if the shaft moves slightly between them. For example, a zero initial position error can be followed by a nonzero raw count during the settled check.

The runner requests 100 Hz fast telemetry and decodes MOTION as `struct.unpack('<ff', frame.data)`. It retains velocity samples in RAM to calculate peak speed. It does not currently persist a timestamped position/velocity/error time series or generate plots. Such plots would need a separately approved capture workflow; the terminal-only test must not be described as saving those graphs.

## 16. Recorded test results

These results are transcribed from the 1 October 2026 terminal messages in the session. They were not remeasured on 3 October. The newer test programs intentionally did not save raw log files, so this report is not a substitute for archived synchronized full-rate measurement data.

### Node-2 stopping validation

- At 360deg/s, five validation final checks were zero encoder counts, using the installed -0.00439453125deg LUT plus bounded finishing. One validation arrival needed correction.
- An installed-baseline five-cycle node-2-only run also completed with zero final checks. Its first cycle needed three corrections, adding about 4.67 seconds of finishing; later cycles required only verification.
- At 720deg/s, five final checks were zero counts; one late one-count drift needed correction.
- No claim is made that the LUT alone achieved every exact stop or that every stop was instantaneous.

### One-cycle 1440deg/s qualification

Both rotations passed the existing checks. Sampled peaks were approximately 1398.77deg/s on node 2 and 1345.81deg/s on node 3. Node 2's final checked error was zero; node 3's reported degree-position slot error was -0.02197265625deg, with a later raw-reference reading showing a 0.0439453125deg correction. These are separate samples, not contradictory synchronized measurements.

### Ten alternating cycles at the 1440deg/s setting

All twenty rotations completed. The following are the printed `slot_error_deg` values for the accepted moves, not maximum tracking error throughout each revolution:

| Cycle | Node 2 final reported error (deg) | Node 3 reported error (deg) |
| ---: | ---: | ---: |
| 1 | 0 | -0.0439453125 |
| 2 | 0 | -0.0439453125 |
| 3 | 0 | +0.02197265625 |
| 4 | 0 | +0.02197265625 |
| 5 | 0 | +0.02197265625 |
| 6 | 0 | -0.0439453125 |
| 7 | 0 | +0.02197265625 |
| 8 | 0 | +0.02197265625 |
| 9 | 0 | +0.02197265625 |
| 10 | 0 | +0.02197265625 |

Sampled peak ranges were approximately 1389.66–1404.97deg/s on node 2 and 1329.45–1360.01deg/s on node 3. Therefore the defensible statement is “tested at a 1440deg/s setting,” not “both motors reached or sustained exactly 1440deg/s.” Sampling and tracking both affect the recorded peaks; the test does not separate their contributions.

Typical printed handoff setup intervals were about 0.41–0.46 seconds. Node-2 re-corrections before node-3 starts increased some printed intervals, with a largest observed value about 3.76 seconds. Additional motion-configuration writes are excluded from that metric. No intentional dwell was added, but the sequence was not pause-free in a physical or latency sense.

Node 2 sometimes drifted again after an initial exact finish, so the runner repeated correction before allowing node 3 to start. The guard logic, not the LUT alone, made those handoffs pass. Both motors were finally disabled, and both showed two-count magnitude relaxation (0.0439453125deg) afterwards.

## 17. Tests and known limitations

The 1 October implementation check reported 80 passing host tests. On 3 October, the two dedicated precision/1440 test files passed again: 12 tests, no hardware motion. A fresh broader-suite run reported failures and was stopped without establishing a current full-suite pass; those broader failures were not diagnosed or repaired in this documentation-only task. Do not interpret the historical 80-test result as a claim about the entire present workspace. These checks are software/simulated tests, not new hardware accuracy measurements.

The dedicated tests verify:

- A fine correction targets the true physical reference, not the shifted LUT target.
- A one-count or two-count persistent node-2 residual is rejected after the bounded attempts.
- The 360/720deg/s profile selects the correct gains, tolerance and LUT.
- Unprofiled speeds do not silently receive the validated profile.
- The temporary 1440deg/s profile retains PID/LUT values but uses the approved 128 rotation microsteps.
- A stationary guard is rejected when moving, disabled or outside its applicable reference limit.

Targeted software test command (does not command the physical motors):

```bash
cd /home/sonu/Grafito-Edge-Services
python3 -m pytest can_stepper/tests/test_node2_precision_finish.py can_stepper/tests/test_alternating_1440.py -q
```

Limitations to retain when describing the system:

1. Zero encoder code error is not zero mechanical error, independent disc alignment, or accuracy better than the sensor can resolve.
2. Fine correction can take seconds; exact-count acceptance and instant switching are not both demonstrated by this implementation.
3. Node 3 does not have node 2's exact-count finish requirement.
4. The current LUT is a small speed/profile-specific endpoint-offset table, not a 16,384-entry encoder-linearity map or a full torque/velocity compensation surface.
5. LUT values are not proven invariant across speed, acceleration, direction, load, temperature, current settings, mounting or microstep changes.
6. The 1440deg/s entry remains labelled experimental extrapolation even though the recorded one-cycle and ten-cycle tests passed.
7. Sampled peak speed is not guaranteed physical maximum speed or sustained speed.
8. The software checks only observed encoder/status data and cannot replace verified clearance geometry, mounting security and an independent emergency stop.
9. Driver-disable and restoration checks depend on working communications; the program can report a failure, not guarantee physical shutdown after total communication loss.
10. Exact flashed firmware provenance remains unresolved: tests reported 1.7, local sources declare 1.9.

## 18. Source provenance and version checks

Sources inspected on 3 October 2026:

- Desktop baseline: `/home/sonu/Desktop/precision test/absolute_alignment_sequence_lut.py`.
- Desktop experiment: `/home/sonu/Desktop/precision test/pid_lut_alternating_1440.py`.
- Calibration helper: `/home/sonu/Grafito-Edge-Services/can_stepper/node2_lut_calibration.py`.
- Active reference: `/home/sonu/Desktop/precision test/alignment_reference_lut.json`.
- SDK protocol and node API: `/home/sonu/Downloads/grafito-canstepper-station/canstepper/protocol.py` and `node.py`.
- Available firmware reference: `/home/sonu/Downloads/grafito-canstepper-station/firmware/GrafitoCANStepper_C3/GrafitoCANStepper_C3.ino`, declaring 1.9.
- Installed library pulse-limit reference: `/home/sonu/Arduino/libraries/FastAccelStepper/src/pd_esp32/pd_config.h` and `esp32_queue.h`.

SHA-256 identities of the source snapshots below:

```text
Baseline controller:
73aecf228a34e32a1a066810379f42e653e3a6a7459d5fed5e3821bcd920daeb

1440deg/s runner:
183f503f049a8d66a80f145d71da16067d6ec750fd4511452a9238b74b8eb2c7

Five-trial calibration helper:
f4c5a0058d1945c2b0d810d39e70093a298eb380bbe145e8fe7b13c431cd86f0
```

The baseline's hash matches its before-1440 backup and the workspace development copy at documentation time. The source appendices are reference snapshots; running a program requires its stated dependencies and current valid hardware/reference conditions. They should not be blindly copied over newer user changes.

## Appendix A — Full baseline controller

Snapshot of `absolute_alignment_sequence_lut.py`; the complete source follows.

```python
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
DEFAULT_SPEED_DEG_S = 360.0
MAX_TEST_SPEED_DEG_S = 720.0
DEFAULT_CYCLES = 1
MAX_CYCLES = 5
MOTION_TOLERANCE_DEG = 0.5
# Separate physical abort limits from the stricter node-2 encoder finish gate.
SLOT_SAFETY_TOLERANCE_DEG = {
    2: 0.65,
    3: 0.5,
}
STARTUP_ALIGNMENT_SPEED = 30.0
NODE2_PRECISION_COUNTS = 0
NODE2_FINAL_PID_TOLERANCE_DEG = 0.012
NODE2_FINAL_ACCEL_DEG_S2 = 720.0
NODE2_MAX_FINISH_ATTEMPTS = 3
NODE2_FINAL_MICROSTEPS = 256
NODE2_FINAL_PID_GAINS = (12.0, 0.0, 0.35)
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
# This entry applies only to the tested PID/microstep profile below. Five
# calibration moves with +0.017578125 offset all arrived one count high:
# corrected offset = +0.017578125 - 360/16384 = -0.00439453125.
# Validation: four zero-count arrivals, one -1-count arrival corrected to zero.
NODE2_STOP_PROFILE = {
    "speed_deg_s": 360.0,
    "microsteps": 256,
    "pid_gains": (12.0, 0.0, 0.35),
    "pid_tolerance_deg": 0.012,
    "offset_deg": -0.00439453125,
    "calibration_samples": 5,
    "validation_samples": 5,
    "validation_arrival_errors_counts": (0, 0, 0, 0, -1),
    "validation_final_errors_counts": (0, 0, 0, 0, 0),
    "status": "validated-with-exact-encoder-finish",
}
NODE2_STOP_PROFILES = {
    360.0: NODE2_STOP_PROFILE,
    720.0: {
        **NODE2_STOP_PROFILE,
        "speed_deg_s": 720.0,
        "validation_arrival_errors_counts": (0, 0, 1, 0, 0),
        "note": "Trial 3 shifted one count after MOVE_DONE; fine finish restored zero.",
    },
}
NODE2_SPEED_LUT = {
    speed: profile["offset_deg"] for speed, profile in NODE2_STOP_PROFILES.items()
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
        "--node2-only", action="store_true",
        help="rotate only node 2; keep node 3 aligned as the stationary guard",
    )
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
    if not 0 < args.speed <= MAX_TEST_SPEED_DEG_S:
        parser.error(
            f"speed must be greater than 0 and no more than "
            f"{MAX_TEST_SPEED_DEG_S:g} deg/s"
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


def finish_node2_at_slot(node, slot_target: float, reference: int, cpr: int) -> dict:
    """Anchor the endpoint at the physical slot, after the LUT-assisted move.

    The LUT-adjusted target is useful for the measured stopping bias, but is
    not the taught physical reference. A short PID finish sets that reference
    explicitly. No SET_ZERO, flash save, or reference reteaching occurs here.
    """
    started = time.monotonic()
    initial_counts = node.get_encoder_counts()
    delta = signed_modular_delta(reference, initial_counts, cpr)
    initial_error = -delta * 360.0 / cpr
    corrected = abs(delta) > NODE2_PRECISION_COUNTS
    attempts = 0
    for attempt in range(NODE2_MAX_FINISH_ATTEMPTS + 1):
        if corrected or attempt > 0:
            if attempt == NODE2_MAX_FINISH_ATTEMPTS:
                break
            node.set_param(Param.MICROSTEPS, NODE2_FINAL_MICROSTEPS)
            node.set_pid(
                *NODE2_FINAL_PID_GAINS, tolerance_deg=NODE2_FINAL_PID_TOLERANCE_DEG
            )
            set_motion_limits(node, STARTUP_ALIGNMENT_SPEED)
            node.set_param(Param.ACCELERATION, NODE2_FINAL_ACCEL_DEG_S2)
            node.set_param(Param.CL_MAX_ACCEL, NODE2_FINAL_ACCEL_DEG_S2)
            node.move_to(slot_target, blocking=True, timeout=6.0)
            attempts += 1
        # MOVE_DONE alone can precede a one-count drift. Let it settle and
        # require exact raw-reference agreement before releasing the other disc.
        time.sleep(0.15)
        counts = []
        for _ in range(3):
            counts.append(node.get_encoder_counts())
            time.sleep(0.02)
        errors = [-signed_modular_delta(reference, count, cpr) for count in counts]
        if all(abs(error) <= NODE2_PRECISION_COUNTS for error in errors):
            break
        corrected = True
        print(f"PRECISION_RETRY node=2 errors={errors}counts", flush=True)
    else:
        errors = []
    if not errors or any(abs(error) > NODE2_PRECISION_COUNTS for error in errors):
        raise RuntimeError(
            f"node 2 precision finish failed after {attempts} corrections: "
            f"raw slot errors {errors} counts; limit {NODE2_PRECISION_COUNTS} count"
        )
    final = counts[-1] * 360.0 / cpr
    print(
        f"PRECISION_FINISH node=2 initial_error={initial_error:+.6f} "
        f"final_error={errors[-1] * 360.0 / cpr:+.6f} "
        f"corrected={corrected} attempts={attempts} "
        f"extra_time={time.monotonic()-started:.3f}s",
        flush=True,
    )
    return {
        "position_deg": final,
        "slot_error_deg": final - slot_target,
        "reference_error_deg": -errors[-1] * 360.0 / cpr,
        "precision_finish_s": time.monotonic() - started,
        "precision_finish_applied": corrected,
    }


def move_one_revolution(
    node,
    slot_target: float,
    lut_offset: float,
    speed: float,
    reference: int,
    cpr: int,
) -> dict[str, float]:
    command_target = slot_target + lut_offset
    if node.node_id == 2:
        configure_node2_motion(node, speed)
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
    result = {
        "slot_target_deg": slot_target,
        "command_target_deg": command_target,
        "lut_offset_deg": lut_offset,
        "position_deg": final,
        "slot_error_deg": slot_error,
        "command_error_deg": command_error,
        "reference_error_deg": reference_error,
        "precision_warning": precision_warning,
        "initial_slot_error_deg": slot_error,
        "precision_finish_s": 0.0,
        "precision_finish_applied": False,
        "elapsed_s": time.monotonic() - started,
    }
    if node.node_id == 2:
        result.update(finish_node2_at_slot(node, slot_target, reference, cpr))
        result["command_error_deg"] = result["position_deg"] - command_target
        result["precision_warning"] = False
        result["elapsed_s"] = time.monotonic() - started
    return result


def configure_node2_motion(node, speed: float) -> None:
    """Never extrapolate the precision LUT to a different motion profile."""
    profile = NODE2_STOP_PROFILES.get(speed)
    if profile is not None:
        node.set_param(Param.MICROSTEPS, profile["microsteps"])
        node.set_pid(
            *profile["pid_gains"],
            tolerance_deg=profile["pid_tolerance_deg"],
        )
    else:
        node.set_param(Param.MICROSTEPS, 32)
        node.set_pid(*PID_GAINS[2], tolerance_deg=MOTION_TOLERANCE_DEG)


def print_settings(lut_offsets: dict[int, float], mode: str, speed: float) -> None:
    print("PID_LUT_SETTINGS", flush=True)
    print(
        f"mode={mode} speed={speed:g}deg/s "
        f"acceleration={speed * 4.0:g}deg/s^2 "
        f"default_node3_tolerance={MOTION_TOLERANCE_DEG:g}deg",
        flush=True,
    )
    speed_status = (
        "measured-speed"
        if speed == LUT_CALIBRATION_SPEED_DEG_S
        else "experimental-speed-extrapolation"
    )
    print(
        f"node3_lut_calibration_speed={LUT_CALIBRATION_SPEED_DEG_S:g}deg/s "
        f"node3_lut_speed_status={speed_status}",
        flush=True,
    )
    profile = NODE2_STOP_PROFILES.get(speed)
    if profile is not None:
        print(
            f"node2_stop_profile={profile} "
            f"applied_lut={lut_offsets[2]:+.9f}deg "
            "node3_uses_720deg_s_lut_if_requested",
            flush=True,
        )
    for node_id in (2, 3):
        kp, ki, kd = PID_GAINS[node_id]
        entry = LUT[node_id]
        tolerance = MOTION_TOLERANCE_DEG
        if node_id == 2 and profile is not None:
            kp, ki, kd = profile["pid_gains"]
            tolerance = profile["pid_tolerance_deg"]
            entry = {
                "offset_deg": profile["offset_deg"],
                "samples": 5,
                "stddev_deg": 0.0,
                "status": profile["status"],
                "warning": "5/5 zero-count final returns; one needed fine correction. Calibration stddev only; see arrival errors above.",
            }
        print(
            f"node={node_id} PID(kp={kp:g},ki={ki:g},kd={kd:g}) "
            f"LUT(stored={entry['offset_deg']:+.6f}deg,"
            f"applied={lut_offsets[node_id]:+.6f}deg,"
            f"samples={entry['samples']},stddev={entry['stddev_deg']:.6f}deg,"
            f"status={entry['status']})",
            flush=True,
        )
        print(
            f"node={node_id} pid_tolerance={tolerance:.3f}deg "
            f"physical_handoff_limit="
            f"{SLOT_SAFETY_TOLERANCE_DEG[node_id]:.3f}deg",
            flush=True,
        )
        print(f"node={node_id} note={entry['warning']}", flush=True)
    print("log_destination=terminal_only files_saved=no", flush=True)
    print(
        f"node2_final_slot_limit={NODE2_PRECISION_COUNTS} encoder_count "
        f"node2_final_pid_tolerance={NODE2_FINAL_PID_TOLERANCE_DEG:g}deg "
        f"node2_final_microsteps={NODE2_FINAL_MICROSTEPS} "
        f"node2_final_pid={NODE2_FINAL_PID_GAINS}",
        flush=True,
    )


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
        "initial_slot_error_deg",
        "precision_finish_s",
        "precision_finish_applied",
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
            (
                NODE2_SPEED_LUT.get(args.speed, float(LUT[2]["offset_deg"]))
                if node_id == 2
                else float(LUT[node_id]["offset_deg"])
            )
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
    saved_params = {}
    runtime_params = (
        Param.MICROSTEPS, Param.CLOSED_LOOP, Param.INVERT_DIR,
        Param.PID_KP, Param.PID_KI, Param.PID_KD, Param.PID_TOLERANCE,
        Param.MAX_SPEED, Param.ACCELERATION, Param.CL_MAX_SPEED, Param.CL_MAX_ACCEL,
    )
    bus = CANStepperBus.serial(args.port)
    time.sleep(2.5)
    nodes = {2: bus.node(2), 3: bus.node(3)}

    try:
        for node_id, node in nodes.items():
            saved_params[node_id] = {p: node.get_param(p) for p in runtime_params}
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

                for node_id in ((2,) if args.node2_only else (2, 3)):
                    if node_id == 3:
                        # Recheck the stationary disc immediately before handoff.
                        finish_node2_at_slot(
                            nodes[2], base_targets[2] - 360.0 * cycle,
                            references[2], cpr,
                        )
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

    except (Exception, KeyboardInterrupt) as exc:
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
        for node_id, values in saved_params.items():
            for param, value in values.items():
                try:
                    nodes[node_id].set_param(param, value)
                except Exception as exc:
                    failure = failure or f"restore failed node {node_id}: {exc}"
                    print(f"RESTORE_ERROR node={node_id} param={param}: {exc}", flush=True)
        try:
            disabled = {
                node_id: not node.get_status().enabled
                for node_id, node in nodes.items()
            }
            print(f"FINAL_DISABLED {disabled}", flush=True)
            for node_id, node in nodes.items():
                print(
                    f"AFTER_DISABLE node={node_id} "
                    f"slot_error={-absolute_error_deg(node, references[node_id], cpr):+.9f}deg",
                    flush=True,
                )
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
```


## Appendix B — Full 1440deg-s alternating runner

Snapshot of `pid_lut_alternating_1440.py`; it imports the Desktop baseline.

```python
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
```


## Appendix C — Full five-trial calibration program

Snapshot of `node2_lut_calibration.py`. Its workspace import dependencies are described in Sections 2 and 14; the printed candidate is not automatically installed.

```python
#!/usr/bin/env python3
"""Five node-2 LUT trials with printed stable raw-encoder measurements."""

import argparse
import json
import math
import statistics
import time

import pid_lut_speed_limit_probe as probe
import standalone_absolute_alignment_sequence_lut as precision_rig

rig = probe.rig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--offset', type=float, default=0.1373291015625)
    parser.add_argument('--microsteps', type=int, choices=(32, 64, 128, 256), default=32)
    parser.add_argument('--pid-tolerance', type=float, default=.5)
    parser.add_argument('--ki', type=float, default=rig.PID_GAINS[2][1])
    parser.add_argument('--precision-finish', action='store_true')
    parser.add_argument('--speed', type=float, choices=(360.0, 720.0), default=360.0)
    args = parser.parse_args()
    if not math.isfinite(args.offset) or abs(args.offset) > .35:
        parser.error('offset must be within +/-0.35 degree')
    if not math.isfinite(args.pid_tolerance) or not .012 <= args.pid_tolerance <= .5:
        parser.error('PID tolerance must be between 0.012 and 0.5 degree')
    if not math.isfinite(args.ki) or not 0 <= args.ki <= .05:
        parser.error('Ki must be between 0 and 0.05')
    node2_gains = (rig.PID_GAINS[2][0], args.ki, rig.PID_GAINS[2][2])
    speed = args.speed
    data = json.loads(rig.REFERENCE_PATH.read_text())
    cpr = data['encoder_counts_per_revolution']
    refs = {int(i): v['aligned_raw_count'] for i, v in data['nodes'].items()}
    bus = rig.CANStepperBus.serial(rig.PORT)
    nodes = {i: bus.node(i) for i in (2, 3)}
    saved = {}
    errors = []
    failure = ''
    try:
        time.sleep(2.5)
        for i, n in nodes.items():
            rig.healthy(n)
            status = n.get_status()
            error = rig.absolute_error_deg(n, refs[i], cpr)
            print(f'PREFLIGHT node={i} correction={error:+.6f} enabled={status.enabled}', flush=True)
            if status.enabled or abs(error) > 2.5:
                raise RuntimeError(f'node {i} alignment preflight failed')
            if n.get_param(rig.Param.ZERO_ON_BOOT) or n.get_param(rig.Param.ENABLE_ON_BOOT):
                raise RuntimeError('unexpected boot settings')
            saved[i] = {p: n.get_param(p) for p in probe.SNAPSHOT}
        for i, n in nodes.items():
            n.set_param(rig.Param.MICROSTEPS, 32)
            n.set_param(rig.Param.CLOSED_LOOP, 1)
            n.set_param(rig.Param.INVERT_DIR, 0)
            n.set_pid(*rig.PID_GAINS[i], tolerance_deg=.5)
        # Node 2 is already visually aligned; align the guard before rotating.
        rig.align_node(nodes[3], refs[3], cpr, 2.5)
        base = rig.align_node(nodes[2], refs[2], cpr, 2.5)
        nodes[2].set_param(rig.Param.MICROSTEPS, args.microsteps)
        nodes[2].set_pid(*node2_gains, tolerance_deg=args.pid_tolerance)
        print(f'PROFILE speed={speed:g} microsteps={args.microsteps} '
              f'PID={node2_gains} tolerance={args.pid_tolerance:g} '
              f'offset={args.offset:+.9f}', flush=True)
        for trial in range(1, 6):
            guard_error = rig.absolute_error_deg(nodes[3], refs[3], cpr)
            if abs(guard_error) > .5:
                raise RuntimeError('node 3 clearance guard shifted')
            rig.healthy(nodes[2])
            rig.set_motion_limits(nodes[2], speed)
            slot_target = base - 360.0 * trial
            start = time.monotonic()
            nodes[2].move_to(slot_target + args.offset, blocking=True, timeout=6.0)
            instant_counts = nodes[2].get_encoder_counts()
            time.sleep(.15)
            counts = []
            for _ in range(5):
                counts.append(nodes[2].get_encoder_counts())
                time.sleep(.02)
            signed_errors = [-rig.signed_modular_delta(refs[2], c, cpr) for c in counts]
            if max(abs(e) * 360 / cpr for e in signed_errors) > .65:
                raise RuntimeError(f'node 2 exceeded physical limit: {signed_errors} counts')
            median_error = statistics.median(signed_errors) * 360 / cpr
            errors.append(median_error)
            print(f'TRIAL {trial}/5 speed={speed:g} offset={args.offset:+.9f} '
                  f'instant_error={-rig.signed_modular_delta(refs[2], instant_counts, cpr)*360/cpr:+.9f} '
                  f'stable_counts={signed_errors} stable_error={median_error:+.9f} '
                  f'elapsed={time.monotonic()-start:.3f}s', flush=True)
            if args.precision_finish:
                finish = precision_rig.finish_node2_at_slot(nodes[2], slot_target, refs[2], cpr)
                print(f'EXACT_FINISH trial={trial} {finish}', flush=True)
                # The finishing stage uses its own settings; restore the
                # measured motion profile before the next calibration trial.
                nodes[2].set_param(rig.Param.MICROSTEPS, args.microsteps)
                nodes[2].set_pid(*node2_gains, tolerance_deg=args.pid_tolerance)
    except (Exception, KeyboardInterrupt) as exc:
        failure = f'{type(exc).__name__}: {exc}'
        print(f'ABORT {failure}', flush=True)
        try:
            bus.estop_all()
        except Exception:
            pass
    finally:
        for _ in range(2):
            for n in nodes.values():
                try:
                    n.disable()
                except Exception:
                    pass
            time.sleep(.2)
        for i, params in saved.items():
            for p, value in params.items():
                try:
                    nodes[i].set_param(p, value)
                except Exception as exc:
                    failure = failure or f'restore failed node {i}: {exc}'
        disabled = {}
        for i, n in nodes.items():
            try:
                disabled[i] = not n.get_status().enabled
                print(f'AFTER_DISABLE node={i} slot_error={-rig.absolute_error_deg(n, refs[i], cpr):+.9f}', flush=True)
            except Exception:
                disabled[i] = False
        print(f'FINAL_DISABLED {disabled}', flush=True)
        bus.close()
    if len(errors) == 5:
        mean = statistics.mean(errors)
        deviation = statistics.stdev(errors)
        candidate = args.offset - mean
        print(f'CALIBRATION_ERRORS {errors}', flush=True)
        print(f'LUT_RESULT speed={speed:g} current={args.offset:+.9f} mean_error={mean:+.9f} '
              f'stddev={deviation:.9f} candidate={candidate:+.9f} '
              f'max_abs_error={max(abs(e) for e in errors):.9f}', flush=True)
    print('FILES_SAVED no; FLASH_SAVE no', flush=True)
    return 1 if failure or len(errors) != 5 or not all(disabled.values()) else 0


if __name__ == '__main__':
    raise SystemExit(main())
```
