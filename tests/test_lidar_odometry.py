import numpy as np

from survey_mapping.lidar_odometry import (
    deskew_points,
    register_point_to_plane,
    voxel_centroids,
)
from survey_mapping.trajectory import Pose, PoseSeries


def test_deskew_points_compensates_platform_translation():
    poses = PoseSeries(
        [
            Pose(0.0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
            Pose(1.0, np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
        ]
    )
    points = np.array(
        [
            [0.0, 10.0, 0.0, 0.0, 20.0, 0.0, 0.0],
            [1.0e9, 9.0, 0.0, 0.0, 30.0, 0.0, 0.0],
        ]
    )

    deskewed = deskew_points(points, 0.0, poses, 1.0, np.eye(3), np.zeros(3))

    assert np.allclose(deskewed[:, :3], [[9.0, 0.0, 0.0], [9.0, 0.0, 0.0]])
    assert np.allclose(deskewed[:, 3], [20.0, 30.0])


def test_voxel_centroids_is_deterministic():
    points = np.array([[0.01, 0.01, 0.01], [0.09, 0.03, 0.05], [1.0, 0.0, 0.0]])

    result = voxel_centroids(points, 0.1)

    assert np.allclose(result, [[0.05, 0.02, 0.03], [1.0, 0.0, 0.0]])


def test_point_to_plane_registration_recovers_small_rigid_transform():
    rng = np.random.default_rng(4)
    base = rng.normal(size=(500, 3))
    base /= np.linalg.norm(base, axis=1, keepdims=True)
    base *= 10.0 + rng.normal(scale=0.2, size=(500, 1))
    angle = np.deg2rad(2.0)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]]
    )
    translation = np.array([0.35, -0.2, 0.15])
    source = (rotation.T @ (base - translation).T).T

    result = register_point_to_plane(source, base, np.eye(4), max_distance=1.5, iterations=30)

    assert result.converged
    assert result.inlier_fraction > 0.9
    assert result.rmse < 0.08
    assert np.allclose(result.transform[:3, :3], rotation, atol=0.02)
    assert np.allclose(result.transform[:3, 3], translation, atol=0.05)
