from dataclasses import dataclass

import numpy as np

from survey_mapping.geo import normalize_quat, slerp_quat


@dataclass
class Pose:
    time: float
    position: np.ndarray
    quaternion: np.ndarray


class PoseSeries:
    def __init__(self, poses):
        if not poses:
            raise ValueError("PoseSeries requires at least one pose")
        self.poses = sorted(poses, key=lambda pose: pose.time)
        self.times = np.asarray([pose.time for pose in self.poses], dtype=float)

    def sample(self, time):
        time = float(time)
        if time <= self.times[0]:
            first = self.poses[0]
            return Pose(time, first.position.copy(), normalize_quat(first.quaternion))
        if time >= self.times[-1]:
            last = self.poses[-1]
            return Pose(time, last.position.copy(), normalize_quat(last.quaternion))
        idx = int(np.searchsorted(self.times, time))
        a = self.poses[idx - 1]
        b = self.poses[idx]
        ratio = (time - a.time) / (b.time - a.time)
        position = a.position + ratio * (b.position - a.position)
        quat = slerp_quat(a.quaternion, b.quaternion, ratio)
        return Pose(time, position, quat)


def build_pose_series(times, positions, attitude_times=None, quaternions=None):
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if attitude_times is None or quaternions is None or len(quaternions) == 0:
        return PoseSeries(
            [Pose(float(t), p.copy(), np.array([0.0, 0.0, 0.0, 1.0])) for t, p in zip(times, positions)]
        )
    attitude = PoseSeries(
        [
            Pose(float(t), np.zeros(3), normalize_quat(q))
            for t, q in zip(np.asarray(attitude_times, dtype=float), np.asarray(quaternions, dtype=float))
        ]
    )
    poses = []
    for t, p in zip(times, positions):
        poses.append(Pose(float(t), p.copy(), attitude.sample(float(t)).quaternion))
    return PoseSeries(poses)
