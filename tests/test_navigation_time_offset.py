import numpy as np

from survey_mapping.lidar_odometry import deskew_points
from survey_mapping.trajectory import Pose, PoseSeries


def test_deskew_uses_navigation_time_offset_for_nonlinear_motion():
    poses = PoseSeries(
        [
            Pose(-1.0, np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
            Pose(0.0, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
            Pose(1.0, np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
        ]
    )
    # A static world point at x=10 is observed while navigation time runs -1 -> 0.
    points = np.array(
        [
            [0.0, 9.0, 0.0, 0.0, 20.0, 0.0, 0.0],
            [1.0e9, 10.0, 0.0, 0.0, 30.0, 0.0, 0.0],
        ]
    )

    deskewed = deskew_points(
        points,
        frame_time=0.0,
        pose_series=poses,
        reference_time=1.0,
        rotation_body_lidar=np.eye(3),
        translation_body_lidar=np.zeros(3),
        navigation_time_offset_s=-1.0,
    )

    assert np.allclose(deskewed[:, :3], [[10.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
