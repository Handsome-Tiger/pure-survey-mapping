import numpy as np
import pytest

from survey_mapping.color_output import DualColorPlyWriter, read_ply_vertex_count


COMPLETE_DTYPE = np.dtype(
    [("xyz", "<f4", (3,)), ("rgb", "u1", (3,)), ("color_valid", "u1")]
)
STRICT_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])


def _read_records(path, dtype):
    with path.open("rb") as handle:
        while handle.readline() != b"end_header\n":
            pass
        return np.fromfile(handle, dtype=dtype)


def test_dual_writer_preserves_all_points_and_filters_strict_points(tmp_path):
    complete = tmp_path / "complete.ply"
    strict = tmp_path / "strict.ply"
    xyz = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], dtype=float)
    rgb = np.array([[10, 20, 30], [40, 50, 60], [70, 80, 90]], dtype=np.uint8)
    valid = np.array([True, False, True])
    intensity = np.array([1, 120, 3], dtype=np.uint8)

    with DualColorPlyWriter(complete, strict) as writer:
        writer.write(xyz, rgb, valid, intensity)
        assert writer.complete_count == 3
        assert writer.strict_count == 2
        assert writer.true_color_count == 2

    assert read_ply_vertex_count(complete) == 3
    assert read_ply_vertex_count(strict) == 2
    assert b"property uchar color_valid" in complete.read_bytes()[:512]
    complete_records = _read_records(complete, COMPLETE_DTYPE)
    strict_records = _read_records(strict, STRICT_DTYPE)
    assert complete_records[1]["rgb"].tolist() == [120, 120, 120]
    assert complete_records["color_valid"].tolist() == [1, 0, 1]
    assert strict_records["xyz"].tolist() == [[1.0, 2.0, 3.0], [7.0, 8.0, 9.0]]
    assert strict_records["rgb"].tolist() == [[10, 20, 30], [70, 80, 90]]


@pytest.mark.parametrize(
    "xyz,rgb,valid,intensity,match",
    [
        (np.zeros((2, 2)), np.zeros((2, 3)), [True, True], [1, 2], "points_xyz"),
        (np.zeros((2, 3)), np.zeros((2, 2)), [True, True], [1, 2], "rgb"),
        (np.zeros((2, 3)), np.zeros((2, 3)), [True], [1, 2], "length"),
        (np.zeros((2, 3)), np.zeros((2, 3)), [True, True], [1], "length"),
        (
            np.array([[np.nan, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            np.zeros((2, 3)),
            [True, True],
            [1, 2],
            "finite",
        ),
    ],
)
def test_dual_writer_rejects_invalid_arrays(tmp_path, xyz, rgb, valid, intensity, match):
    with DualColorPlyWriter(tmp_path / "complete.ply", tmp_path / "strict.ply") as writer:
        with pytest.raises(ValueError, match=match):
            writer.write(xyz, rgb, valid, intensity)


def test_dual_writer_does_not_patch_counts_after_exception(tmp_path):
    complete = tmp_path / "complete.ply"
    strict = tmp_path / "strict.ply"

    with pytest.raises(RuntimeError, match="stop"):
        with DualColorPlyWriter(complete, strict) as writer:
            writer.write(
                np.array([[1.0, 2.0, 3.0]]),
                np.array([[10, 20, 30]], dtype=np.uint8),
                np.array([True]),
                np.array([5], dtype=np.uint8),
            )
            raise RuntimeError("stop")

    assert read_ply_vertex_count(complete) == 0
    assert read_ply_vertex_count(strict) == 0
