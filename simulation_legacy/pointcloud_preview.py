"""Lightweight XYZ extraction from a Dimos PointCloud2 LCM file."""

from __future__ import annotations

from pathlib import Path
import struct

import numpy as np
from dimos_lcm.sensor_msgs.PointCloud2 import PointCloud2 as LCMPointCloud2


def read_xy(path: Path, max_points: int = 30_000) -> np.ndarray:
    """Decode and evenly sample XY coordinates without importing Open3D."""
    message = LCMPointCloud2.lcm_decode(path.read_bytes())
    if message.width <= 0 or message.height <= 0:
        return np.empty((0, 2), dtype=np.float32)

    offsets: dict[str, int] = {}
    for field in message.fields:
        if field.name in {"x", "y", "z"}:
            offsets[field.name] = int(field.offset)
    if not {"x", "y"}.issubset(offsets):
        raise ValueError("PointCloud2 map is missing x/y fields")

    count = int(message.width * message.height)
    step = int(message.point_step)
    raw = message.data
    if step <= 0 or len(raw) < count * step:
        raise ValueError("PointCloud2 data length does not match its point layout")

    stride = max(1, count // max_points)
    indices = np.arange(0, count, stride, dtype=np.int64)
    if offsets.get("x") == 0 and offsets.get("y") == 4 and step >= 8:
        if step == 8:
            layout = np.dtype([("x", "<f4"), ("y", "<f4")])
        else:
            layout = np.dtype(
                [("x", "<f4"), ("y", "<f4"), ("_rest", f"V{step - 8}")]
            )
        structured = np.frombuffer(raw, dtype=layout, count=count)
        xy = np.column_stack((structured["x"][indices], structured["y"][indices]))
    else:
        xy = np.empty((len(indices), 2), dtype=np.float32)
        for output_index, point_index in enumerate(indices):
            base = int(point_index) * step
            xy[output_index, 0] = struct.unpack_from("<f", raw, base + offsets["x"])[0]
            xy[output_index, 1] = struct.unpack_from("<f", raw, base + offsets["y"])[0]

    return xy[np.isfinite(xy).all(axis=1)]