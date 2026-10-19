import numpy as np

from survey_mapping.ekf import GnssInsFilter


def test_ekf_prediction_integrates_constant_acceleration():
    filt = GnssInsFilter(accel_noise=0.1, gnss_sigma=1.0)
    filt.predict(1.0, np.array([1.0, 0.0, 0.0]))
    state = filt.state()
    assert np.allclose(state.position, [0.5, 0.0, 0.0], atol=1e-9)
    assert np.allclose(state.velocity, [1.0, 0.0, 0.0], atol=1e-9)


def test_ekf_gnss_update_pulls_position_toward_measurement():
    filt = GnssInsFilter(accel_noise=0.1, gnss_sigma=0.5)
    filt.predict(1.0, np.array([0.0, 0.0, 0.0]))
    before = filt.state().position.copy()
    filt.update_gnss(np.array([10.0, 0.0, 0.0]), sigma=0.5)
    after = filt.state().position
    assert after[0] > before[0]
    assert after[0] < 10.0
