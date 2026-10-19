import math

import numpy as np

from survey_mapping.geo import geodetic_to_enu, quat_to_matrix, wgs84_to_ecef


def test_wgs84_to_ecef_equator_prime_meridian():
    xyz = wgs84_to_ecef(0.0, 0.0, 0.0)
    assert np.allclose(xyz, [6378137.0, 0.0, 0.0], atol=1e-6)


def test_geodetic_to_enu_small_east_offset():
    origin = (22.20785650776563, 114.26042957568009, 5.64432430267334)
    enu = geodetic_to_enu(origin[0], origin[1] + 0.00001, origin[2], origin)
    assert enu[0] > 1.0
    assert abs(enu[1]) < 0.05
    assert abs(enu[2]) < 0.05


def test_quaternion_rotates_x_to_y_for_ninety_degree_yaw():
    half = math.sqrt(0.5)
    rot = quat_to_matrix([0.0, 0.0, half, half])
    assert np.allclose(rot @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)
