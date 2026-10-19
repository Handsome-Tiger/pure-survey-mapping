import numpy as np

from survey_mapping.pipeline_alignment import resolve_time_alignment


def _gyro_signal(times):
    return np.column_stack(
        [
            np.sin(0.8 * times) + 0.2 * np.sin(2.7 * times),
            0.5 * np.cos(0.6 * times),
            0.3 * np.sin(1.9 * times),
        ]
    )


def test_resolve_time_alignment_uses_fixed_mode_without_imu_data():
    config = {
        "mapping": {"navigation_time_offset_s": -0.6},
        "time_alignment": {"method": "fixed"},
    }

    result = resolve_time_alignment(config, np.zeros((0, 7)), np.zeros((0, 7)))

    assert result.offset_s == -0.6
    assert result.method == "fixed"
    assert result.used_fallback is False


def test_resolve_time_alignment_estimates_dual_imu_offset():
    reference_times = np.arange(0.0, 20.0, 0.005)
    sensor_times = np.arange(0.35, 20.35, 0.01)
    reference = np.column_stack([reference_times, _gyro_signal(reference_times), np.zeros((len(reference_times), 3))])
    sensor = np.column_stack([sensor_times, _gyro_signal(sensor_times - 0.35), np.zeros((len(sensor_times), 3))])
    config = {
        "mapping": {"navigation_time_offset_s": -0.6},
        "time_alignment": {
            "method": "gyro_correlation",
            "search_min_s": -0.7,
            "search_max_s": -0.1,
            "step_s": 0.01,
            "sample_period_s": 0.01,
            "minimum_correlation": 0.8,
        },
    }

    result = resolve_time_alignment(config, reference, sensor)

    assert abs(result.offset_s + 0.35) <= 0.01
    assert result.method == "gyro_correlation"
    assert result.used_fallback is False
    assert result.correlation > 0.99
