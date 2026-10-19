import numpy as np

from scripts.colorize_cloud_from_camera import colorized_ply_header, format_colorized_vertex


def test_colorized_ply_output_contains_rgb_without_intensity_scalar():
    header = colorized_ply_header(1)
    vertex = format_colorized_vertex(
        np.array([1.0, 2.0, 3.0]),
        np.array([10, 20, 30], dtype=np.uint8),
    )

    assert "property uchar red" in header
    assert "property uchar green" in header
    assert "property uchar blue" in header
    assert "intensity" not in header
    assert len(vertex.split()) == 6
