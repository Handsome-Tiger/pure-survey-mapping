import numpy as np
import pytest

from survey_mapping.time_alignment import estimate_gyro_time_offset


def _angular_rate(physical_time):
    physical_time = np.asarray(physical_time, dtype=float)
    return np.column_stack(
        [
            0.4 * np.sin(0.71 * physical_time) + 0.2 * np.sin(2.13 * physical_time),
            0.3 * np.cos(0.43 * physical_time + 0.2),
            0.25 * np.sin(1.37 * physical_time - 0.4),
        ]
    )


def test_estimate_gyro_time_offset_recovers_sensor_clock_advance():
    reference_times = np.arange(0.0, 30.0, 0.005)
    sensor_times = np.arange(0.4, 30.4, 0.01)
    reference_gyro = _angular_rate(reference_times)
    sensor_gyro = _angular_rate(sensor_times - 0.4)

    estimate = estimate_gyro_time_offset(
        reference_times,
        reference_gyro,
        sensor_times,
        sensor_gyro,
        search_min_s=-0.8,
        search_max_s=0.0,
        step_s=0.01,
        sample_period_s=0.01,
    )

    assert abs(estimate.offset_s + 0.4) <= 0.01
    assert estimate.correlation > 0.99
    assert estimate.sample_count > 2500


def test_estimate_gyro_time_offset_rejects_stationary_data():
    times = np.arange(0.0, 10.0, 0.01)
    gyro = np.zeros((len(times), 3), dtype=float)

    with pytest.raises(ValueError, match="angular motion"):
        estimate_gyro_time_offset(
            times,
            gyro,
            times,
            gyro,
            search_min_s=-0.5,
            search_max_s=0.5,
            step_s=0.01,
            sample_period_s=0.01,
        )
