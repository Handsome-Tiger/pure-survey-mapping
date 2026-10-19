import numpy as np

from survey_mapping.error_state_ins import ErrorStateConfig, ErrorStateInsFilter
from survey_mapping.geo import quat_to_matrix


def test_error_state_filter_propagates_constant_acceleration_in_navigation_frame():
    cfg = ErrorStateConfig(
        accel_noise=0.01,
        gyro_noise=0.01,
        accel_bias_noise=1e-5,
        gyro_bias_noise=1e-5,
        gnss_sigma=0.5,
        gravity=0.0,
    )
    filt = ErrorStateInsFilter(cfg)
    filt.reset(
        position=np.zeros(3),
        velocity=np.zeros(3),
        quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    filt.propagate(1.0, gyro_body=np.zeros(3), accel_body=np.array([1.0, 0.0, 0.0]))

    state = filt.state()
    assert np.allclose(state.position, [0.5, 0.0, 0.0], atol=1e-9)
    assert np.allclose(state.velocity, [1.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(quat_to_matrix(state.quaternion), np.eye(3), atol=1e-9)


def test_error_state_gnss_update_uses_lever_arm_and_reduces_position_error():
    cfg = ErrorStateConfig(
        accel_noise=0.01,
        gyro_noise=0.01,
        accel_bias_noise=1e-5,
        gyro_bias_noise=1e-5,
        gnss_sigma=0.5,
        lever_arm_body=np.array([1.0, 0.0, 0.0]),
        gravity=0.0,
    )
    filt = ErrorStateInsFilter(cfg)
    filt.reset(
        position=np.zeros(3),
        velocity=np.zeros(3),
        quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    accepted, residual = filt.update_gnss(np.array([11.0, 0.0, 0.0]))

    assert accepted
    assert np.allclose(residual, [10.0, 0.0, 0.0])
    assert 0.0 < filt.state().position[0] < 10.0


def test_error_state_gnss_update_rejects_outlier():
    cfg = ErrorStateConfig(gnss_sigma=0.5, outlier_threshold=5.0, gravity=0.0)
    filt = ErrorStateInsFilter(cfg)
    filt.reset(
        position=np.zeros(3),
        velocity=np.zeros(3),
        quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    accepted, residual = filt.update_gnss(np.array([100.0, 0.0, 0.0]))

    assert not accepted
    assert np.allclose(residual, [100.0, 0.0, 0.0])
    assert np.allclose(filt.state().position, np.zeros(3))


def test_velocity_update_reduces_velocity_error():
    filt = ErrorStateInsFilter(ErrorStateConfig(gravity=0.0))
    filt.reset(np.zeros(3), velocity=np.array([5.0, -2.0, 1.0]))
    before = np.linalg.norm(filt.velocity - np.array([1.0, 0.0, 0.0]))

    accepted, residual = filt.update_velocity(np.array([1.0, 0.0, 0.0]), sigma=0.05)

    assert accepted
    assert np.allclose(residual, [-4.0, 2.0, -1.0])
    assert np.linalg.norm(filt.velocity - np.array([1.0, 0.0, 0.0])) < before


def test_yaw_update_wraps_across_pi_and_reduces_heading_error():
    filt = ErrorStateInsFilter(ErrorStateConfig(gravity=0.0))
    start_yaw = np.deg2rad(179.0)
    target_yaw = np.deg2rad(-179.0)
    filt.reset(
        np.zeros(3),
        quaternion=np.array([0.0, 0.0, np.sin(start_yaw / 2.0), np.cos(start_yaw / 2.0)]),
    )

    accepted, residual = filt.update_yaw(target_yaw, sigma_rad=np.deg2rad(0.5))

    assert accepted
    assert np.isclose(np.rad2deg(residual), 2.0, atol=1e-6)
    r = quat_to_matrix(filt.quaternion)
    updated_yaw = np.arctan2(r[1, 0], r[0, 0])
    wrapped_error = (target_yaw - updated_yaw + np.pi) % (2.0 * np.pi) - np.pi
    assert abs(wrapped_error) < np.deg2rad(2.0)
