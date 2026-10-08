from pathlib import Path
from io import StringIO

import pytest

import absolute_alignment_sequence_lut as controller


LUT_PATH = Path(__file__).resolve().parents[1] / "alignment_lut.json"


def test_only_validated_exact_speed_entry_is_eligible_by_default():
    lut = controller.load_lut(LUT_PATH)

    node2, reason2 = controller.select_lut_correction(
        lut, 2, 720.0, "clockwise"
    )
    node3, reason3 = controller.select_lut_correction(
        lut, 3, 720.0, "clockwise"
    )

    assert node2 == 0.0
    assert reason2 == "entry disabled"
    assert node3 == pytest.approx(-0.25927734375)
    assert reason3 == "eligible validated exact-speed entry"


def test_rejected_node2_entry_stays_disabled_even_with_explicit_opt_in():
    lut = controller.load_lut(LUT_PATH)

    correction, reason = controller.select_lut_correction(
        lut, 2, 720.0, "clockwise", allow_experimental=True
    )

    assert correction == 0.0
    assert reason == "entry disabled"


def test_lut_does_not_extrapolate_to_an_unmeasured_speed():
    lut = controller.load_lut(LUT_PATH)

    correction, reason = controller.select_lut_correction(
        lut, 3, 360.0, "clockwise"
    )

    assert correction == 0.0
    assert "found 0" in reason


@pytest.mark.parametrize(
    ("field", "value", "expected_reason"),
    [
        ("samples", 4, "only 4 samples"),
        ("residual_stddev_deg", 0.11, "stddev"),
        ("command_offset_deg", 0.36, "cap"),
    ],
)
def test_lut_rejects_unsafe_statistics(field, value, expected_reason):
    entry = {
        "node_id": 3,
        "direction": "clockwise",
        "speed_deg_s": 720.0,
        "command_offset_deg": -0.25,
        "samples": 5,
        "residual_stddev_deg": 0.05,
        "enabled": True,
        "validation_status": "validated",
    }
    entry[field] = value

    correction, reason = controller.select_lut_correction(
        {"entries": [entry]}, 3, 720.0, "clockwise"
    )

    assert correction == 0.0
    assert expected_reason in reason


class FakeNode:
    node_id = 3

    def __init__(self):
        self.params = {}
        self.command_target = None
        self.position = 0.0

    def set_param(self, param, value):
        self.params[param] = value

    def move_to(self, target, blocking, timeout):
        assert blocking is True
        assert timeout > 0
        self.command_target = target
        # Model the repeatable +0.25-degree stopping bias the LUT cancels.
        self.position = target + 0.25

    def get_position(self):
        return self.position

    def get_encoder_counts(self):
        # The compensated physical position is exactly the raw slot reference.
        return 0


def test_move_gates_on_physical_slot_not_shifted_command():
    node = FakeNode()

    result = controller.move_one_revolution(
        node,
        slot_target=-360.0,
        speed=720.0,
        lut_candidate=-0.25,
        lut_applied=-0.25,
        reference=0,
        cpr=16384,
    )

    assert node.command_target == pytest.approx(-360.25)
    assert result["position_deg"] == pytest.approx(-360.0)
    assert result["slot_error_deg"] == pytest.approx(0.0)
    assert result["command_error_deg"] == pytest.approx(0.25)
    assert result["reference_error_deg"] == pytest.approx(0.0)


def test_stdout_log_mode_prints_complete_logs_without_creating_files(tmp_path):
    csv_path = tmp_path / "must_not_exist.csv"
    summary_path = tmp_path / "must_not_exist.txt"
    output = StringIO()

    controller.emit_logs(
        "stdout",
        rows=[{"node_id": 2, "slot_error_deg": 0.125}],
        fields=["node_id", "slot_error_deg"],
        lines=["Result: PASSED", "LUT mode: apply"],
        csv_path=csv_path,
        summary_path=summary_path,
        stream=output,
    )

    text = output.getvalue()
    assert "SUMMARY_LOG_BEGIN" in text
    assert "Result: PASSED" in text
    assert "CSV_LOG_BEGIN" in text
    assert "2,0.125" in text
    assert "LOG_FILES_SAVED no" in text
    assert not csv_path.exists()
    assert not summary_path.exists()
