from dataclasses import dataclass, field

import numpy as np

from survey_mapping.geo import normalize_quat, quat_multiply, quat_to_matrix, rotvec_to_quat, skew


@dataclass
class ErrorStateConfig:
    accel_noise: float = 0.2
    gyro_noise: float = 0.02
    accel_bias_noise: float = 1e-4
    gyro_bias_noise: float = 1e-5
    gnss_sigma: float = 1.0
    gravity: float = 9.80665
    outlier_threshold: float = 20.0
    robust_threshold: float = 4.0
    robust_sigma_scale: float = 5.0
    lever_arm_body: np.ndarray = field(default_factory=lambda: np.zeros(3))
    initial_position_sigma: float = 5.0
    initial_velocity_sigma: float = 1.0
    initial_attitude_sigma: float = 0.1
    initial_gyro_bias_sigma: float = 0.02
    initial_accel_bias_sigma: float = 0.5


@dataclass
class ErrorStateInsState:
    position: np.ndarray
    velocity: np.ndarray
    quaternion: np.ndarray
    gyro_bias: np.ndarray
    accel_bias: np.ndarray
    covariance: np.ndarray


class ErrorStateInsFilter:
    """KF-GINS-style loosely coupled 15-state GNSS/INS error-state filter."""

    def __init__(self, config=None):
        self.config = config or ErrorStateConfig()
        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.gyro_bias = np.zeros(3, dtype=float)
        self.accel_bias = np.zeros(3, dtype=float)
        self.p = np.eye(15, dtype=float)
        self.reset(np.zeros(3), np.zeros(3), self.quaternion)

    def reset(self, position, velocity=None, quaternion=None, gyro_bias=None, accel_bias=None):
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = np.zeros(3, dtype=float) if velocity is None else np.asarray(velocity, dtype=float).copy()
        self.quaternion = normalize_quat(
            np.array([0.0, 0.0, 0.0, 1.0]) if quaternion is None else np.asarray(quaternion, dtype=float)
        )
        self.gyro_bias = np.zeros(3, dtype=float) if gyro_bias is None else np.asarray(gyro_bias, dtype=float).copy()
        self.accel_bias = np.zeros(3, dtype=float) if accel_bias is None else np.asarray(accel_bias, dtype=float).copy()
        cfg = self.config
        sigmas = np.array(
            [
                cfg.initial_position_sigma,
                cfg.initial_position_sigma,
                cfg.initial_position_sigma,
                cfg.initial_velocity_sigma,
                cfg.initial_velocity_sigma,
                cfg.initial_velocity_sigma,
                cfg.initial_attitude_sigma,
                cfg.initial_attitude_sigma,
                cfg.initial_attitude_sigma,
                cfg.initial_gyro_bias_sigma,
                cfg.initial_gyro_bias_sigma,
                cfg.initial_gyro_bias_sigma,
                cfg.initial_accel_bias_sigma,
                cfg.initial_accel_bias_sigma,
                cfg.initial_accel_bias_sigma,
            ],
            dtype=float,
        )
        self.p = np.diag(sigmas * sigmas)

    def propagate(self, dt, gyro_body, accel_body):
        dt = float(dt)
        if dt <= 0.0:
            return
        gyro = np.asarray(gyro_body, dtype=float) - self.gyro_bias
        accel = np.asarray(accel_body, dtype=float) - self.accel_bias
        dq = rotvec_to_quat(gyro * dt)
        self.quaternion = quat_multiply(self.quaternion, dq)
        r_nb = quat_to_matrix(self.quaternion)
        specific_force_nav = r_nb @ accel
        accel_nav = specific_force_nav - np.array([0.0, 0.0, self.config.gravity])
        self.position = self.position + self.velocity * dt + 0.5 * accel_nav * dt * dt
        self.velocity = self.velocity + accel_nav * dt

        f = np.eye(15, dtype=float)
        f[0:3, 3:6] = np.eye(3) * dt
        f[3:6, 6:9] = -r_nb @ skew(accel) * dt
        f[3:6, 12:15] = -r_nb * dt
        f[6:9, 9:12] = -np.eye(3) * dt
        q = np.zeros((15, 15), dtype=float)
        q[3:6, 3:6] = np.eye(3) * (self.config.accel_noise**2) * dt * dt
        q[6:9, 6:9] = np.eye(3) * (self.config.gyro_noise**2) * dt * dt
        q[9:12, 9:12] = np.eye(3) * (self.config.gyro_bias_noise**2) * dt
        q[12:15, 12:15] = np.eye(3) * (self.config.accel_bias_noise**2) * dt
        self.p = f @ self.p @ f.T + q

    def update_gnss(self, gnss_position, sigma=None):
        z = np.asarray(gnss_position, dtype=float)
        r_nb = quat_to_matrix(self.quaternion)
        lever = np.asarray(self.config.lever_arm_body, dtype=float)
        predicted = self.position + r_nb @ lever
        residual = z - predicted
        residual_norm = float(np.linalg.norm(residual))
        if residual_norm > self.config.outlier_threshold:
            return False, residual

        h = np.zeros((3, 15), dtype=float)
        h[:, 0:3] = np.eye(3)
        h[:, 6:9] = -r_nb @ skew(lever)
        obs_sigma = self.config.gnss_sigma if sigma is None else float(sigma)
        if residual_norm > self.config.robust_threshold:
            obs_sigma *= self.config.robust_sigma_scale
        r = np.eye(3, dtype=float) * (obs_sigma * obs_sigma)
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        dx = k @ residual
        self._inject_error(dx)
        i = np.eye(15, dtype=float)
        self.p = (i - k @ h) @ self.p @ (i - k @ h).T + k @ r @ k.T
        return True, residual

    def update_velocity(self, velocity_nav, sigma=0.1):
        z = np.asarray(velocity_nav, dtype=float)
        residual = z - self.velocity
        h = np.zeros((3, 15), dtype=float)
        h[:, 3:6] = np.eye(3)
        r = np.eye(3, dtype=float) * float(sigma) ** 2
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self._inject_error(k @ residual)
        i = np.eye(15, dtype=float)
        self.p = (i - k @ h) @ self.p @ (i - k @ h).T + k @ r @ k.T
        return True, residual

    def update_yaw(self, yaw_rad, sigma_rad=0.02):
        r_nb = quat_to_matrix(self.quaternion)
        current_yaw = np.arctan2(r_nb[1, 0], r_nb[0, 0])
        residual = (float(yaw_rad) - current_yaw + np.pi) % (2.0 * np.pi) - np.pi
        h = np.zeros((1, 15), dtype=float)
        h[0, 8] = 1.0
        measurement_cov = np.array([[float(sigma_rad) ** 2]], dtype=float)
        s = h @ self.p @ h.T + measurement_cov
        k = self.p @ h.T @ np.linalg.inv(s)
        self._inject_error(k[:, 0] * residual)
        i = np.eye(15, dtype=float)
        self.p = (i - k @ h) @ self.p @ (i - k @ h).T + k @ measurement_cov @ k.T
        return True, residual

    def _inject_error(self, dx):
        self.position += dx[0:3]
        self.velocity += dx[3:6]
        self.quaternion = quat_multiply(rotvec_to_quat(dx[6:9]), self.quaternion)
        self.gyro_bias += dx[9:12]
        self.accel_bias += dx[12:15]

    def state(self):
        return ErrorStateInsState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            quaternion=self.quaternion.copy(),
            gyro_bias=self.gyro_bias.copy(),
            accel_bias=self.accel_bias.copy(),
            covariance=self.p.copy(),
        )
