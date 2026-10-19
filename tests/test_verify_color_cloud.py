import numpy as np
import pytest

from scripts.verify_color_cloud import verify_complete_ply, verify_strict_ply
from survey_mapping.color_output import DualColorPlyWriter


def write_pair(tmp_path):
    complete = tmp_path / "complete.ply"
    strict = tmp_path / "strict.ply"
    with DualColorPlyWriter(complete, strict) as writer:
        writer.write(
            np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, 6.0]]),
            np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8),
            np.array([True, False]),
            np.array([7, 8], dtype=np.uint8),
        )
    return complete, strict


def test_verifier_checks_valid_complete_and_strict_pair(tmp_path):
    complete, strict = write_pair(tmp_path)

    complete_result = verify_complete_ply(complete)
    strict_result = verify_strict_ply(strict)

    assert complete_result["declared_vertex_count"] == 2
    assert complete_result["actual_vertex_count"] == 2
    assert complete_result["finite_vertex_count"] == 2
    assert complete_result["true_color_count"] == 1
    assert complete_result["minimum_xyz"] == [-4.0, 2.0, 3.0]
    assert complete_result["maximum_xyz"] == [1.0, 5.0, 6.0]
    assert strict_result["actual_vertex_count"] == 1


def test_verifier_rejects_truncated_binary_record(tmp_path):
    _complete, strict = write_pair(tmp_path)
    strict.write_bytes(strict.read_bytes()[:-1])

    with pytest.raises(ValueError, match="PLY declared vertex count does not match"):
        verify_strict_ply(strict)

