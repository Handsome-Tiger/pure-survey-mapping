import numpy as np

from survey_mapping.calibrated_camera import project_lidar_points, visibility_mask


def test_project_lidar_points_applies_radial_distortion():
    points = np.array([[1.0, 0.0, 2.0], [0.0, 0.0, -1.0]])
    intrinsics = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]])

    uv, depth, valid = project_lidar_points(
        points,
        np.eye(3),
        np.zeros(3),
        intrinsics,
        np.array([0.1, 0.0, 0.0, 0.0]),
        (100, 120, 3),
    )

    assert valid.tolist() == [True, False]
    assert np.allclose(uv[0], [101.25, 40.0])
    assert np.allclose(depth, [2.0, -1.0])


def test_visibility_mask_rejects_points_hidden_in_same_pixel():
    uv = np.array([[10.1, 20.1], [10.2, 20.2], [11.0, 20.0]])
    depth = np.array([2.0, 3.0, 4.0])
    valid = np.ones(3, dtype=bool)

    visible = visibility_mask(uv, depth, valid, (40, 50, 3), depth_tolerance_m=0.2)

    assert visible.tolist() == [True, False, True]
