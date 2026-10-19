from dataclasses import dataclass

import numpy as np


@dataclass
class FilterState:
    position: np.ndarray
    velocity: np.ndarray
    covariance: np.ndarray


class GnssInsFilter:
    """Six-state loosely coupled position/velocity filter."""

    def __init__(self, accel_noise=0.5, gnss_sigma=2.0):
        self.x = np.zeros(6, dtype=float)
        self.p = np.eye(6, dtype=float) * 10.0
        self.accel_noise = float(accel_noise)
        self.gnss_sigma = float(gnss_sigma)

    def reset(self, position, velocity=None):
        self.x[:3] = np.asarray(position, dtype=float)
        self.x[3:] = np.zeros(3) if velocity is None else np.asarray(velocity, dtype=float)
        self.p = np.eye(6, dtype=float)

    def predict(self, dt, accel_map):
        dt = float(dt)
        if dt <= 0.0:
            return
        accel = np.asarray(accel_map, dtype=float)
        f = np.eye(6, dtype=float)
        f[:3, 3:] = np.eye(3) * dt
        b = np.zeros((6, 3), dtype=float)
        b[:3, :] = 0.5 * dt * dt * np.eye(3)
        b[3:, :] = dt * np.eye(3)
        self.x = f @ self.x + b @ accel
        q = (self.accel_noise**2) * (b @ b.T)
        self.p = f @ self.p @ f.T + q

    def update_gnss(self, position, sigma=None):
        z = np.asarray(position, dtype=float)
        h = np.zeros((3, 6), dtype=float)
        h[:, :3] = np.eye(3)
        r_sigma = self.gnss_sigma if sigma is None else float(sigma)
        r = np.eye(3, dtype=float) * (r_sigma**2)
        innovation = z - h @ self.x
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ innovation
        i = np.eye(6, dtype=float)
        self.p = (i - k @ h) @ self.p @ (i - k @ h).T + k @ r @ k.T
        return innovation

    def state(self):
        return FilterState(
            position=self.x[:3].copy(),
            velocity=self.x[3:].copy(),
            covariance=self.p.copy(),
        )


def fuse_gnss_imu(gnss_times, gnss_positions, imu_times, imu_accels, gnss_sigma=2.0):
    gnss_times = np.asarray(gnss_times, dtype=float)
    gnss_positions = np.asarray(gnss_positions, dtype=float)
    imu_times = np.asarray(imu_times, dtype=float)
    imu_accels = np.asarray(imu_accels, dtype=float)
    if len(gnss_times) == 0:
        raise ValueError("At least one GNSS observation is required")
    filt = GnssInsFilter(gnss_sigma=gnss_sigma)
    filt.reset(gnss_positions[0])
    out_times = []
    out_positions = []
    out_velocities = []
    gnss_idx = 0
    last_t = float(gnss_times[0])
    for t, accel in zip(imu_times, imu_accels):
        if t < gnss_times[0]:
            continue
        filt.predict(t - last_t, accel)
        last_t = float(t)
        while gnss_idx < len(gnss_times) and gnss_times[gnss_idx] <= t:
            filt.update_gnss(gnss_positions[gnss_idx])
            gnss_idx += 1
        state = filt.state()
        out_times.append(float(t))
        out_positions.append(state.position)
        out_velocities.append(state.velocity)
    return np.asarray(out_times), np.asarray(out_positions), np.asarray(out_velocities)
