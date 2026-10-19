from pathlib import Path

import numpy as np

from survey_mapping.pose_graph import pose_correction_metrics, read_g2o_poses, write_g2o_graph


def test_write_and_read_g2o_se3_graph(tmp_path):
    first = np.eye(4)
    second = np.eye(4)
    second[:3, 3] = [1.0, 2.0, 3.0]
    edge = np.linalg.inv(first) @ second
    graph_path = tmp_path / "graph.g2o"

    write_g2o_graph([first, second], [(0, 1, edge, 1.0)], graph_path)

    text = graph_path.read_text(encoding="ascii")
    assert "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1" in text
    assert "VERTEX_SE3:QUAT 1 1 2 3 0 0 0 1" in text
    assert "EDGE_SE3:QUAT 0 1 1 2 3 0 0 0 1" in text
    poses = read_g2o_poses(graph_path, maximum_vertex_id=1)
    assert np.allclose(poses[0], first)
    assert np.allclose(poses[1], second)


def test_pose_correction_metrics_do_not_turn_rotation_into_translation():
    before = np.eye(4)
    before[:3, 3] = [100.0, 200.0, 50.0]
    after = before.copy()
    angle = np.deg2rad(1.0)
    after[:3, :3] = np.array(
        [[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]]
    )

    translation_m, rotation_deg = pose_correction_metrics(before, after)

    assert translation_m == 0.0
    assert np.isclose(rotation_deg, 1.0)
