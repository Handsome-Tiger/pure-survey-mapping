import numpy as np
import pytest

from survey_mapping.imu_deskew import ImuRotationSeries


def test_imu_rotation_series_integrates_constant_angular_rate():
    times = np.linspace(0.0, 1.0, 101)
    gyro = np.zeros((len(times), 3), dtype=float)
    gyro[:, 2] = np.pi / 2.0

    series = ImuRotationSeries.from_gyro(times, gyro)
    relative = series.relative_rotations(np.array([0.0, 1.0]), reference_time=1.0)
    rotated = relative.apply(np.array([[10.0, 0.0, 0.0], [0.0, -10.0, 0.0]]))

    assert np.allclose(rotated, [[0.0, -10.0, 0.0], [0.0, -10.0, 0.0]], atol=1e-6)


def test_imu_rotation_series_rejects_non_increasing_timestamps():
    with pytest.raises(ValueError, match="strictly increasing"):
        ImuRotationSeries.from_gyro(
            np.array([0.0, 0.1, 0.1]),
            np.zeros((3, 3), dtype=float),
        )
