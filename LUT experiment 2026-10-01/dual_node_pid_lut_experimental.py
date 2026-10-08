#!/usr/bin/env python3
"""Run the two-disc rig with PID and LUT corrections on both nodes.

Node 2's fixed LUT previously exceeded the 0.5-degree slot tolerance during
physical validation.  This separate experimental launcher therefore requires
an explicit acknowledgement before it enables that entry.  The underlying
controller still checks the true raw slot reference after every move and
aborts/disables both drivers on any failure.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import absolute_alignment_sequence_lut as controller


SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENTAL_LUT_PATH = SCRIPT_DIR / "alignment_lut_both_nodes_experimental.json"
SPEED_DEG_S = 7200.0
MAX_CYCLES = 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Experimental 720 deg/s PID+LUT runner for nodes 2 and 3"
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="report both LUT entries but apply zero compensation",
    )
    parser.add_argument(
        "--accept-node2-experimental-risk",
        action="store_true",
        help=(
            "required for motion with both LUTs; node 2 previously produced "
            "a +0.593262 degree error and safety abort"
        ),
    )
    args = parser.parse_args()
    if not 1 <= args.cycles <= MAX_CYCLES:
        parser.error(f"cycles must be between 1 and {MAX_CYCLES}")
    if not args.monitor and not args.accept_node2_experimental_risk:
        parser.error(
            "--accept-node2-experimental-risk is required; use --monitor "
            "for a no-compensation baseline"
        )
    return args


def controller_argv(args: argparse.Namespace) -> list[str]:
    argv = [
        str(Path(controller.__file__).resolve()),
        "--port",
        args.port,
        "--speed",
        str(SPEED_DEG_S),
        "--cycles",
        str(args.cycles),
        "--lut-path",
        str(EXPERIMENTAL_LUT_PATH),
        "--lut-mode",
        "monitor" if args.monitor else "apply",
        "--log-mode",
        "stdout",
    ]
    if not args.monitor:
        argv.append("--allow-experimental-lut")
    return argv


def print_settings() -> None:
    data = json.loads(EXPERIMENTAL_LUT_PATH.read_text(encoding="utf-8"))
    print("DUAL_NODE_PID_LUT_SETTINGS", flush=True)
    print(
        f"speed={SPEED_DEG_S:g}deg/s acceleration={SPEED_DEG_S * 4:g}deg/s^2 "
        f"tolerance={controller.MOTION_TOLERANCE_DEG:g}deg",
        flush=True,
    )
    for entry in data["entries"]:
        node_id = int(entry["node_id"])
        kp, ki, kd = controller.PID_GAINS[node_id]
        print(
            f"node={node_id} PID(kp={kp:g},ki={ki:g},kd={kd:g}) "
            f"LUT(offset={float(entry['command_offset_deg']):+.6f}deg," 
            f"samples={int(entry['samples'])},"
            f"stddev={float(entry['residual_stddev_deg']):.6f}deg,"
            f"status={entry['validation_status']},"
            f"enabled={bool(entry['enabled'])})",
            flush=True,
        )
    print("log_destination=terminal_only files_saved=no", flush=True)


def main() -> int:
    args = parse_args()
    print_settings()
    if args.monitor:
        print("DUAL_NODE_LUT monitor mode: no LUT offsets will be applied")
    else:
        print(
            "WARNING: enabling experimental node-2 LUT and validated "
            "node-3 LUT; physical safety gates remain active",
            flush=True,
        )

    original_argv = sys.argv
    try:
        sys.argv = controller_argv(args)
        return controller.main()
    finally:
        sys.argv = original_argv


if __name__ == "__main__":
    raise SystemExit(main())
