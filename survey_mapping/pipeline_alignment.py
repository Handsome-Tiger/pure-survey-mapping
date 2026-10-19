from dataclasses import dataclass

import numpy as np

from survey_mapping.time_alignment import estimate_gyro_time_offset


@dataclass(frozen=True)
class ResolvedTimeAlignment:
    offset_s: float
    correlation: float
    margin: float
    sample_count: int
    method: str
    used_fallback: bool


def resolve_time_alignment(config, reference_imu, sensor_imu):
    alignment = config.get("time_alignment", {})
    mapping = config.get("mapping", {})
    method = str(alignment.get("method", "fixed")).lower()
    fixed_offset = float(mapping.get("navigation_time_offset_s", 0.0))
    if method == "fixed":
        return ResolvedTimeAlignment(
            offset_s=fixed_offset,
            correlation=float("nan"),
            margin=float("nan"),
            sample_count=0,
            method="fixed",
            used_fallback=False,
        )
    if method != "gyro_correlation":
        raise ValueError("time_alignment.method must be 'fixed' or 'gyro_correlation'")

    reference_imu = np.asarray(reference_imu, dtype=float)
    sensor_imu = np.asarray(sensor_imu, dtype=float)
    if reference_imu.ndim != 2 or reference_imu.shape[1] < 4:
        raise ValueError("reference IMU rows must contain time and three gyroscope columns")
    if sensor_imu.ndim != 2 or sensor_imu.shape[1] < 4:
        raise ValueError("sensor IMU rows must contain time and three gyroscope columns")
    estimate = estimate_gyro_time_offset(
        reference_imu[:, 0],
        reference_imu[:, 1:4],
        sensor_imu[:, 0],
        sensor_imu[:, 1:4],
        search_min_s=float(alignment.get("search_min_s", -1.0)),
        search_max_s=float(alignment.get("search_max_s", 1.0)),
        step_s=float(alignment.get("step_s", 0.005)),
        sample_period_s=float(alignment.get("sample_period_s", 0.02)),
    )
    minimum_correlation = float(alignment.get("minimum_correlation", 0.8))
    if estimate.correlation < minimum_correlation:
        if bool(alignment.get("allow_fixed_fallback", False)):
            return ResolvedTimeAlignment(
                offset_s=fixed_offset,
                correlation=estimate.correlation,
                margin=estimate.margin,
                sample_count=estimate.sample_count,
                method="fixed_fallback",
                used_fallback=True,
            )
        raise ValueError(
            f"dual-IMU time alignment correlation {estimate.correlation:.3f} "
            f"is below {minimum_correlation:.3f}"
        )
    return ResolvedTimeAlignment(
        offset_s=estimate.offset_s,
        correlation=estimate.correlation,
        margin=estimate.margin,
        sample_count=estimate.sample_count,
        method="gyro_correlation",
        used_fallback=False,
    )
