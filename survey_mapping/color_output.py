from pathlib import Path

import numpy as np


_COUNT_WIDTH = 20
_COMPLETE_DTYPE = np.dtype(
    [("xyz", "<f4", (3,)), ("rgb", "u1", (3,)), ("color_valid", "u1")]
)
_STRICT_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])


def read_ply_vertex_count(path):
    with Path(path).open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("PLY header does not contain an element vertex line")
            if line.startswith(b"element vertex "):
                return int(line.split()[-1])


class _BinaryPlyStream:
    def __init__(self, path, complete):
        self.path = Path(path)
        self.complete = bool(complete)
        self.handle = None
        self.count_offset = None
        self.count = 0

    def open(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("w+b")
        self.handle.write(b"ply\nformat binary_little_endian 1.0\nelement vertex ")
        self.count_offset = self.handle.tell()
        self.handle.write(b"0" * _COUNT_WIDTH)
        self.handle.write(
            b"\nproperty float x\nproperty float y\nproperty float z\n"
            b"property uchar red\nproperty uchar green\nproperty uchar blue\n"
        )
        if self.complete:
            self.handle.write(b"property uchar color_valid\n")
        self.handle.write(b"end_header\n")

    def patch_count(self):
        self.handle.seek(self.count_offset)
        self.handle.write(f"{self.count:0{_COUNT_WIDTH}d}".encode("ascii"))

    def close(self):
        if self.handle is not None:
            self.handle.close()
            self.handle = None


class DualColorPlyWriter:
    def __init__(self, complete_path, strict_path):
        complete_path = Path(complete_path)
        strict_path = Path(strict_path)
        if complete_path.resolve() == strict_path.resolve():
            raise ValueError("complete and strict PLY paths must be different")
        self._complete = _BinaryPlyStream(complete_path, complete=True)
        self._strict = _BinaryPlyStream(strict_path, complete=False)

    @property
    def complete_count(self):
        return self._complete.count

    @property
    def strict_count(self):
        return self._strict.count

    @property
    def true_color_count(self):
        return self._strict.count

    def __enter__(self):
        self._complete.open()
        try:
            self._strict.open()
        except Exception:
            self._complete.close()
            raise
        return self

    def write(self, points_xyz, rgb, color_valid, intensity):
        if self._complete.handle is None or self._strict.handle is None:
            raise RuntimeError("DualColorPlyWriter must be used as a context manager")
        xyz = np.asarray(points_xyz, dtype=float)
        colors = np.asarray(rgb)
        valid = np.asarray(color_valid, dtype=bool).reshape(-1)
        gray = np.asarray(intensity).reshape(-1)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError("points_xyz must have shape (N, 3)")
        if colors.ndim != 2 or colors.shape[1] != 3:
            raise ValueError("rgb must have shape (N, 3)")
        if len(colors) != len(xyz) or len(valid) != len(xyz) or len(gray) != len(xyz):
            raise ValueError("point, rgb, validity, and intensity lengths must match")
        if not np.all(np.isfinite(xyz)):
            raise ValueError("points_xyz must contain only finite values")

        colors = np.clip(np.rint(colors), 0, 255).astype(np.uint8)
        gray = np.clip(np.rint(gray), 0, 255).astype(np.uint8)
        colors = colors.copy()
        colors[~valid] = gray[~valid, None]

        complete_records = np.empty(len(xyz), dtype=_COMPLETE_DTYPE)
        complete_records["xyz"] = xyz.astype(np.float32)
        complete_records["rgb"] = colors
        complete_records["color_valid"] = valid.astype(np.uint8)
        self._complete.handle.write(complete_records.tobytes())
        self._complete.count += len(complete_records)

        strict_records = np.empty(int(np.count_nonzero(valid)), dtype=_STRICT_DTYPE)
        strict_records["xyz"] = xyz[valid].astype(np.float32)
        strict_records["rgb"] = colors[valid]
        self._strict.handle.write(strict_records.tobytes())
        self._strict.count += len(strict_records)

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            if exc_type is None:
                self._complete.patch_count()
                self._strict.patch_count()
        finally:
            self._complete.close()
            self._strict.close()
