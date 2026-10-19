import math

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def wgs84_to_ecef(lat_deg, lon_deg, height_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + height_m) * cos_lat * math.cos(lon)
    y = (n + height_m) * cos_lat * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + height_m) * sin_lat
    return np.array([x, y, z], dtype=float)


def ecef_to_enu_matrix(origin_lat_deg, origin_lon_deg):
    lat = math.radians(origin_lat_deg)
    lon = math.radians(origin_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    return np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=float,
    )


def geodetic_to_enu(lat_deg, lon_deg, height_m, origin):
    origin_lat, origin_lon, origin_h = origin
    delta = wgs84_to_ecef(lat_deg, lon_deg, height_m) - wgs84_to_ecef(
        origin_lat, origin_lon, origin_h
    )
    return ecef_to_enu_matrix(origin_lat, origin_lon) @ delta


def normalize_quat(quat_xyzw):
    quat = np.asarray(quat_xyzw, dtype=float)
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def quat_to_matrix(quat_xyzw):
    x, y, z, w = normalize_quat(quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_multiply(q1_xyzw, q2_xyzw):
    x1, y1, z1, w1 = normalize_quat(q1_xyzw)
    x2, y2, z2, w2 = normalize_quat(q2_xyzw)
    return normalize_quat(
        np.array(
            [
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ],
            dtype=float,
        )
    )


def rotvec_to_quat(rotvec):
    rotvec = np.asarray(rotvec, dtype=float)
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return normalize_quat(np.array([0.5 * rotvec[0], 0.5 * rotvec[1], 0.5 * rotvec[2], 1.0]))
    axis = rotvec / angle
    half = 0.5 * angle
    return normalize_quat(np.array([axis[0] * math.sin(half), axis[1] * math.sin(half), axis[2] * math.sin(half), math.cos(half)]))


def skew(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)

def slerp_quat(q0, q1, ratio):
    q0 = normalize_quat(q0)
    q1 = normalize_quat(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return normalize_quat(q0 + ratio * (q1 - q0))
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return normalize_quat((s0 * q0) + (s1 * q1))
