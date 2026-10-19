import numpy as np

from survey_mapping.pose_graph import write_g2o_graph


def information_diagonal(edge_line):
    fields = edge_line.split()
    information = [float(value) for value in fields[10:]]
    return [information[index] for index in (0, 6, 11, 15, 18, 20)]


def test_lidar_edge_accepts_distinct_translation_and_rotation_weights(tmp_path):
    first = np.eye(4)
    second = np.eye(4)
    second[0, 3] = 1.0
    measurement = np.linalg.inv(first) @ second
    graph = tmp_path / "weighted.g2o"

    write_g2o_graph(
        [first, second],
        [(0, 1, measurement, 2.0, 200.0)],
        graph,
        prior_translation_weight=10.0,
        prior_rotation_weight=1000.0,
    )

    edge = next(
        line
        for line in graph.read_text(encoding="ascii").splitlines()
        if line.startswith("EDGE_SE3:QUAT 0 1 ")
    )
    assert information_diagonal(edge) == [2.0, 2.0, 2.0, 200.0, 200.0, 200.0]

