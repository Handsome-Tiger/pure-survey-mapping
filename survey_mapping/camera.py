import itertools
import math

import cv2
import numpy as np


def intrinsics_from_fov(width, height, fov_x_deg, fov_y_deg):
    fx = (0.5 * float(width)) / math.tan(0.5 * math.radians(float(fov_x_deg)))
    fy = (0.5 * float(height)) / math.tan(0.5 * math.radians(float(fov_y_deg)))
    cx = (float(width) - 1.0) * 0.5
    cy = (float(height) - 1.0) * 0.5
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=float)


def decode_compressed_bgr(image_bytes):
    encoded = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Could not decode compressed camera image")
    return image


def signed_permutation_rotations():
    rotations = []
    for perm in itertools.permutations(range(3)):
        p = np.zeros((3, 3), dtype=float)
        for row, col in enumerate(perm):
            p[row, col] = 1.0
        for signs in itertools.product([-1.0, 1.0], repeat=3):
            r = np.diag(signs) @ p
            if round(float(np.linalg.det(r))) == 1:
                rotations.append(r)
    return rotations


def project_body_points(points_body, rotation_camera_body, translation_body_camera, intrinsics, image_shape):
    points_body = np.asarray(points_body, dtype=float)
    r_cb = np.asarray(rotation_camera_body, dtype=float)
    t_bc = np.asarray(translation_body_camera, dtype=float)
    k = np.asarray(intrinsics, dtype=float)
    if len(points_body) == 0:
        return np.zeros((0, 2), dtype=int), np.zeros(0, dtype=bool)
    points_camera = (r_cb @ (points_body - t_bc).T).T
    z = points_camera[:, 2]
    valid = z > 0.2
    uv = np.zeros((len(points_body), 2), dtype=float)
    if np.any(valid):
        x = points_camera[valid, 0] / z[valid]
        y = points_camera[valid, 1] / z[valid]
        u = k[0, 0] * x + k[0, 2]
        v = k[1, 1] * y + k[1, 2]
        h, w = image_shape[:2]
        inside = (u >= 0.0) & (u <= w - 1.0) & (v >= 0.0) & (v <= h - 1.0)
        valid_indices = np.flatnonzero(valid)
        valid[:] = False
        keep = valid_indices[inside]
        valid[keep] = True
        uv[keep, 0] = u[inside]
        uv[keep, 1] = v[inside]
    return uv, valid


def sample_bgr_bilinear(image_bgr, uv):
    image = np.asarray(image_bgr)
    uv = np.asarray(uv, dtype=float)
    if len(uv) == 0:
        return np.zeros((0, 3), dtype=np.uint8)
    h, w = image.shape[:2]
    u = np.clip(uv[:, 0], 0.0, w - 1.0)
    v = np.clip(uv[:, 1], 0.0, h - 1.0)
    u0 = np.floor(u).astype(int)
    v0 = np.floor(v).astype(int)
    u1 = np.clip(u0 + 1, 0, w - 1)
    v1 = np.clip(v0 + 1, 0, h - 1)
    du = (u - u0)[:, None]
    dv = (v - v0)[:, None]
    top = image[v0, u0].astype(float) * (1.0 - du) + image[v0, u1].astype(float) * du
    bottom = image[v1, u0].astype(float) * (1.0 - du) + image[v1, u1].astype(float) * du
    sampled = top * (1.0 - dv) + bottom * dv
    return np.clip(np.rint(sampled), 0, 255).astype(np.uint8)


def colorize_body_points(points_body, intensities, image_bgr, rotation_camera_body, translation_body_camera, intrinsics):
    uv, valid = project_body_points(
        points_body, rotation_camera_body, translation_body_camera, intrinsics, image_bgr.shape
    )
    gray = np.clip(np.rint(np.asarray(intensities, dtype=float)), 0, 255).astype(np.uint8)
    rgb = np.repeat(gray[:, None], 3, axis=1)
    if np.any(valid):
        sampled_bgr = sample_bgr_bilinear(image_bgr, uv[valid])
        rgb[valid] = sampled_bgr[:, ::-1]
    return rgb, valid
