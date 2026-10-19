from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation


def _numbers(values):
    return " ".join(f"{float(value):.12g}" for value in values)


def _pose_values(transform):
    transform = np.asarray(transform, dtype=float)
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return [*transform[:3, 3].tolist(), *quaternion.tolist()]


def _information_values(translation_weight, rotation_weight):
    matrix = np.diag([translation_weight] * 3 + [rotation_weight] * 3)
    return [matrix[row, column] for row in range(6) for column in range(row, 6)]


def pose_correction_metrics(before, after):
    before = np.asarray(before, dtype=float)
    after = np.asarray(after, dtype=float)
    translation_m = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
    rotation_delta = after[:3, :3] @ before[:3, :3].T
    rotation_deg = float(np.degrees(Rotation.from_matrix(rotation_delta).magnitude()))
    return translation_m, rotation_deg


def write_g2o_graph(
    initial_poses,
    lidar_edges,
    path,
    prior_translation_weight=400.0,
    prior_rotation_weight=0.01,
    prior_vertex_offset=100000,
):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    poses = [np.asarray(pose, dtype=float) for pose in initial_poses]
    lines = []
    for vertex_id, pose in enumerate(poses):
        lines.append(f"VERTEX_SE3:QUAT {vertex_id} {_numbers(_pose_values(pose))}")
    for edge in lidar_edges:
        if len(edge) == 4:
            first, second, measurement, translation_weight = edge
            rotation_weight = translation_weight
        elif len(edge) == 5:
            first, second, measurement, translation_weight, rotation_weight = edge
        else:
            raise ValueError("LiDAR edges must contain four or five values")
        information = _information_values(
            float(translation_weight), float(rotation_weight)
        )
        lines.append(
            f"EDGE_SE3:QUAT {int(first)} {int(second)} "
            f"{_numbers(_pose_values(measurement))} {_numbers(information)}"
        )
    identity = np.eye(4, dtype=float)
    prior_information = _information_values(
        prior_translation_weight, prior_rotation_weight
    )
    for vertex_id, pose in enumerate(poses):
        prior_id = int(prior_vertex_offset + vertex_id)
        lines.append(f"VERTEX_SE3:QUAT {prior_id} {_numbers(_pose_values(pose))}")
        lines.append(f"FIX {prior_id}")
        lines.append(
            f"EDGE_SE3:QUAT {prior_id} {vertex_id} "
            f"{_numbers(_pose_values(identity))} {_numbers(prior_information)}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def read_g2o_poses(path, maximum_vertex_id=None):
    poses = {}
    for line in Path(path).read_text(encoding="ascii").splitlines():
        fields = line.split()
        if not fields or fields[0] != "VERTEX_SE3:QUAT":
            continue
        vertex_id = int(fields[1])
        if maximum_vertex_id is not None and vertex_id > int(maximum_vertex_id):
            continue
        values = np.asarray([float(value) for value in fields[2:9]], dtype=float)
        transform = np.eye(4, dtype=float)
        transform[:3, 3] = values[:3]
        transform[:3, :3] = Rotation.from_quat(values[3:7]).as_matrix()
        poses[vertex_id] = transform
    return poses


def run_g2o(
    input_path,
    output_path,
    iterations=30,
    robust_kernel="Huber",
    robust_width=1.0,
):
    command = [
        "/usr/local/bin/g2o",
        "-solver",
        "lm_fix6_3",
        "-i",
        str(int(iterations)),
    ]
    if robust_kernel:
        command.extend(
            [
                "-robustKernel",
                str(robust_kernel),
                "-robustKernelWidth",
                str(float(robust_width)),
            ]
        )
    command.extend(["-o", str(output_path), str(input_path)])
    return subprocess.run(command, check=True, capture_output=True, text=True)

