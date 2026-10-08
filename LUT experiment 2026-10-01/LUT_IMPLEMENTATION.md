# Experimental alignment LUT

The proven controller remains `absolute_alignment_sequence.py`. Its exact
pre-LUT copy and reference data are stored under
`backups/20261001_pre_lut/`, with SHA-256 checksums.

The experimental controller is `absolute_alignment_sequence_lut.py`. It uses
`alignment_lut.json` and never calls `SET_ZERO` or `SAVE_CONFIG`.

## Modes

- `--lut-mode off`: ignore the table completely.
- `--lut-mode monitor` (default): validate and report a candidate correction,
  but apply zero correction.
- `--lut-mode apply`: shift the motion command by an eligible correction.
- `--allow-experimental-lut`: explicitly permit an experimental entry. This
  is required for node 2 and is never implied by `--lut-mode apply`.

Only an exact node, direction and speed match is accepted. There is no speed
interpolation or extrapolation. An entry also needs at least five samples,
standard deviation no greater than 0.10 degrees, and magnitude no greater
than 0.35 degrees.

The final motion gate checks the physical slot target and raw absolute encoder
reference. It does not treat reaching the LUT-shifted command as sufficient.

## Current candidates

- Node 2 at 720 deg/s: experimental +0.137329-degree command offset computed
  from 12 uncompensated moves. Its errors change sign and have a 0.198-degree
  sample standard deviation, so the entry requires explicit opt-in and a
  physical A/B test before it can be promoted.
- Node 3 at 720 deg/s: eligible candidate of -0.259277 degrees, based on five
  runs with a 0.036-degree sample standard deviation.
- No 360 deg/s entry is enabled because those measurements used a different
  microstep configuration from the alternating controller.

## Safe test order

1. Start from manually confirmed slot alignment.
2. Run the copied controller in `monitor` mode and confirm it behaves exactly
   like the proven controller.
3. Reconfirm alignment.
4. Run one 720 deg/s alternating cycle in `apply` mode.
5. Compare node 3 slot error with the non-LUT baseline before doing five
   cycles or enabling any node 2 correction.

The controller writes the LUT candidate, applied correction, command target,
physical slot target, command error, slot error and raw-reference error into
every CSV row.

## Separate dual-node experimental launcher

`dual_node_pid_lut_experimental.py` and
`alignment_lut_both_nodes_experimental.json` provide an isolated way to run
PID+LUT on both nodes at the measured 720 deg/s point. They do not alter the
validated LUT table. The launcher limits each invocation to five cycles and
requires `--accept-node2-experimental-risk` before it can apply the rejected
node 2 correction. Monitor mode applies no LUT correction.

The dedicated launcher always passes `--log-mode stdout`. It prints the LUT
and PID settings, readable summary, and complete CSV rows directly in the
terminal. It does not create timestamped log files. The underlying controller
retains `files` as its default when invoked directly so existing validated
workflows are unchanged.

### Node 2 physical result

The experimental +0.137329-degree node 2 entry initially improved a fresh
one-cycle baseline from -0.307617 degrees to 0.000000 degrees. It was not
repeatable: the subsequent validation measured -0.197754, -0.021973,
-0.065918 and then +0.593262 degrees. The physical-slot safety gate aborted
cycle 4 before node 3 moved and disabled both drivers. The node 2 entry is
therefore disabled and rejected; the validated node 3 LUT remains unchanged.

## Physical validation on 2026-10-01

The 720 deg/s monitor cycle measured a node 3 slot error of +0.285645 degrees
with no correction applied. The one-cycle A/B run applied -0.259277 degrees
and reduced the error to +0.021973 degrees.

A subsequent five-cycle apply-mode run passed all 10 alternating moves. Node
3 errors were +0.021973, +0.021973, +0.021973, 0.000000 and +0.021973
degrees: a mean of +0.017578 degrees and maximum absolute error of 0.021973
degrees. Node 2 remained uncompensated because its error continues to change
sign. Both drivers were verified disabled after every run.
