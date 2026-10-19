from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TimeOffsetEstimate:
    offset_s: float
    correlation: float
    margin: float
    sample_count: int


def _validate_gyro_series(times, gyro, name):
    times = np.asarray(times, dtype=float)
    gyro = np.asarray(gyro, dtype=float)
    if times.ndim != 1 or gyro.shape != (len(times), 3):
        raise ValueError(f"{name} times and gyro must have shapes (N,) and (N, 3)")
    if len(times) < 2 or not np.all(np.diff(times) > 0.0):
        raise ValueError(f"{name} timestamps must be strictly increasing")
    if not np.all(np.isfinite(times)) or not np.all(np.isfinite(gyro)):
        raise ValueError(f"{name} IMU data must be finite")
    return times, gyro


def estimate_gyro_time_offset(
    reference_times,
    reference_gyro,
    sensor_times,
    sensor_gyro,
    search_min_s=-1.0,
    search_max_s=1.0,
    step_s=0.005,
    sample_period_s=0.02,
):
    """Estimate the offset satisfying t_reference = t_sensor + offset."""
    reference_times, reference_gyro = _validate_gyro_series(
        reference_times, reference_gyro, "reference"
    )
    sensor_times, sensor_gyro = _validate_gyro_series(sensor_times, sensor_gyro, "sensor")
    search_min_s = float(search_min_s)
    search_max_s = float(search_max_s)
    step_s = float(step_s)
    sample_period_s = float(sample_period_s)
    if search_max_s < search_min_s or step_s <= 0.0 or sample_period_s <= 0.0:
        raise ValueError("time-offset search bounds and sampling periods are invalid")

    grid_start = max(sensor_times[0], reference_times[0] - search_min_s)
    grid_end = min(sensor_times[-1], reference_times[-1] - search_max_s)
    sample_times = np.arange(grid_start, grid_end, sample_period_s, dtype=float)
    if len(sample_times) < 100:
        raise ValueError("insufficient overlapping IMU samples for time alignment")

    reference_speed = np.linalg.norm(reference_gyro, axis=1)
    sensor_speed = np.linalg.norm(sensor_gyro, axis=1)
    sampled_sensor = np.interp(sample_times, sensor_times, sensor_speed)
    sampled_sensor -= sampled_sensor.mean()
    sensor_scale = np.linalg.norm(sampled_sensor)
    if sensor_scale <= 1e-10:
        raise ValueError("insufficient angular motion for time alignment")
    sampled_sensor /= sensor_scale

    candidate_count = int(np.floor((search_max_s - search_min_s) / step_s + 0.5)) + 1
    offsets = search_min_s + np.arange(candidate_count, dtype=float) * step_s
    offsets = offsets[offsets <= search_max_s + step_s * 1e-6]
    correlations = np.empty(len(offsets), dtype=float)
    for index, offset in enumerate(offsets):
        sampled_reference = np.interp(
            sample_times + offset, reference_times, reference_speed
        )
        sampled_reference -= sampled_reference.mean()
        reference_scale = np.linalg.norm(sampled_reference)
        if reference_scale <= 1e-10:
            raise ValueError("insufficient angular motion for time alignment")
        correlations[index] = float(np.dot(sampled_sensor, sampled_reference / reference_scale))

    best_index = int(np.argmax(correlations))
    exclusion = np.abs(np.arange(len(offsets)) - best_index) <= 2
    alternatives = correlations[~exclusion]
    margin = float(correlations[best_index] - np.max(alternatives)) if len(alternatives) else 0.0
    return TimeOffsetEstimate(
        offset_s=float(offsets[best_index]),
        correlation=float(correlations[best_index]),
        margin=margin,
        sample_count=int(len(sample_times)),
    )
