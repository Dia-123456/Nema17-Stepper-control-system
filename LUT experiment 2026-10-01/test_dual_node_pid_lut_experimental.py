import argparse
import json
from pathlib import Path

import dual_node_pid_lut_experimental as runner


def test_dedicated_table_enables_both_exact_speed_entries():
    data = json.loads(runner.EXPERIMENTAL_LUT_PATH.read_text(encoding="utf-8"))
    entries = {entry["node_id"]: entry for entry in data["entries"]}

    assert set(entries) == {2, 3}
    assert entries[2]["enabled"] is True
    assert entries[2]["validation_status"] == "experimental"
    assert entries[3]["enabled"] is True
    assert entries[3]["validation_status"] == "validated"


def test_apply_invocation_explicitly_enables_experimental_entry():
    args = argparse.Namespace(
        port="/dev/ttyACM0",
        cycles=1,
        monitor=False,
        accept_node2_experimental_risk=True,
    )

    argv = runner.controller_argv(args)

    assert "--lut-mode" in argv
    assert argv[argv.index("--lut-mode") + 1] == "apply"
    assert "--allow-experimental-lut" in argv
    assert str(runner.EXPERIMENTAL_LUT_PATH) in argv
    assert argv[argv.index("--log-mode") + 1] == "stdout"


def test_monitor_invocation_never_enables_experimental_entry():
    args = argparse.Namespace(
        port="/dev/ttyACM0",
        cycles=5,
        monitor=True,
        accept_node2_experimental_risk=False,
    )

    argv = runner.controller_argv(args)

    assert argv[argv.index("--lut-mode") + 1] == "monitor"
    assert "--allow-experimental-lut" not in argv
    assert argv[argv.index("--log-mode") + 1] == "stdout"
    assert Path(argv[0]).name == "absolute_alignment_sequence_lut.py"
