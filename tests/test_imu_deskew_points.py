import numpy as np
from scipy.spatial.transform import Rotation

from survey_mapping.imu_deskew import ImuRotationSeries, deskew_points_with_imu
from survey_mapping.trajectory import Pose, PoseSeries


def test_deskew_points_with_imu_collapses_rotating_scan():
    times = np.linspace(0.0, 1.0, 101)
    gyro = np.zeros((len(times), 3), dtype=float)
    gyro[:, 2] = np.pi / 2.0
    imu_rotations = ImuRotationSeries.from_gyro(times, gyro)
    reference_quaternion = Rotation.from_euler("z", 90.0, degrees=True).as_quat()
    poses = PoseSeries(
        [
            Pose(0.0, np.zeros(3), reference_quaternion),
            Pose(1.0, np.zeros(3), reference_quaternion),
        ]
    )
    points = np.array(
        [
            [0.0, 10.0, 0.0, 0.0, 20.0, 0.0, 0.0],
            [1.0e9, 0.0, -10.0, 0.0, 30.0, 0.0, 0.0],
        ]
    )

    deskewed = deskew_points_with_imu(
        points,
        frame_time=0.0,
        reference_time=1.0,
        imu_rotations=imu_rotations,
        pose_series=poses,
    )

    assert np.allclose(
        deskewed[:, :3],
        [[0.0, -10.0, 0.0], [0.0, -10.0, 0.0]],
        atol=1e-6,
    )
    assert np.allclose(deskewed[:, 3], [20.0, 30.0])


def test_deskew_points_with_imu_collapses_translating_scan():
    times = np.linspace(0.0, 1.0, 101)
    imu_rotations = ImuRotationSeries.from_gyro(times, np.zeros((len(times), 3)))
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    poses = PoseSeries(
        [
            Pose(0.0, np.array([0.0, 0.0, 0.0]), identity),
            Pose(1.0, np.array([1.0, 0.0, 0.0]), identity),
        ]
    )
    points = np.array(
        [
            [0.0, 10.0, 0.0, 0.0, 20.0, 0.0, 0.0],
            [1.0e9, 9.0, 0.0, 0.0, 30.0, 0.0, 0.0],
        ]
    )

    deskewed = deskew_points_with_imu(
        points,
        frame_time=0.0,
        reference_time=1.0,
        imu_rotations=imu_rotations,
        pose_series=poses,
    )

    assert np.allclose(
        deskewed[:, :3],
        [[9.0, 0.0, 0.0], [9.0, 0.0, 0.0]],
        atol=1e-6,
    )
