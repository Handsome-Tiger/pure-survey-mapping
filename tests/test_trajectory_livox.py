import numpy as np

from survey_mapping.livox import georeference_points, write_ply
from survey_mapping.trajectory import Pose, PoseSeries


def test_pose_series_interpolates_position_and_quaternion():
    series = PoseSeries([
        Pose(0.0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
        Pose(2.0, np.array([2.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    ])
    pose = series.sample(1.0)
    assert np.allclose(pose.position, [1.0, 0.0, 0.0])
    assert np.allclose(pose.quaternion, [0.0, 0.0, 0.0, 1.0])


def test_georeference_points_applies_pose_translation():
    series = PoseSeries([
        Pose(1.0, np.array([10.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    ])
    points = np.array([[1.0, 1.0, 2.0, 3.0, 30.0, 0.0, 0.0]])
    mapped = georeference_points(points, 1.0, series)
    assert np.allclose(mapped[:, :3], [[11.0, 2.0, 3.0]])
    assert mapped[0, 3] == 30.0


def test_write_ply_creates_ascii_point_file(tmp_path):
    out = tmp_path / "cloud.ply"
    write_ply(out, np.array([[1.0, 2.0, 3.0, 12.0]]))
    text = out.read_text()
    assert "element vertex 1" in text
    assert "1.000000 2.000000 3.000000 12" in text


def test_filter_valid_livox_points_drops_zero_returns():
    from survey_mapping.livox import filter_valid_points

    points = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0, 0.0, 7.0, 0.0, 0.0],
    ])
    filtered = filter_valid_points(points)
    assert filtered.shape == (1, 7)
    assert filtered[0, 1] == 1.0