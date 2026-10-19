import csv

import numpy as np
import pytest

from scripts.optimize_lidar_pose_graph import load_edges


def test_load_edges_scales_rotation_information_independently(tmp_path):
    np.savez_compressed(
        tmp_path / "lidar_constraint_transforms.npz",
        transforms=np.asarray([np.eye(4)]),
    )
    with (tmp_path / "lidar_constraints.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "first",
                "second",
                "point_to_plane_rmse_m",
                "inlier_fraction",
                "transform_index",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "first": 0,
                "second": 1,
                "point_to_plane_rmse_m": 0.2,
                "inlier_fraction": 0.8,
                "transform_index": 0,
            }
        )

    edge = load_edges(tmp_path, rotation_weight_scale=100.0)[0]

    assert edge[3] == pytest.approx(20.0)
    assert edge[4] == pytest.approx(2000.0)

