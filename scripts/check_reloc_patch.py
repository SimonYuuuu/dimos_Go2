#!/usr/bin/env python3
"""Fail if the patched relocalization cannot work, so a patrol never starts blind.

1. relocalize.py must not read or write a RelocalizeConfig field that does not exist.
   A truncated name (cfg.icp_dist_f) made every relocalize() call raise, the error was
   only logged, and the patrol ran on odometry alone.
2. The wall tracker must be installed, hooked in, and pull a knocked pose back onto a
   small made-up room. Its failures are also only logged while the patrol keeps walking.

About a second: no DimOS import, only numpy and scipy for the second check.
"""

from __future__ import annotations

import ast
import importlib.util
import math
import sys
from pathlib import Path


def check_config_fields(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    fields: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "RelocalizeConfig":
            for item in node.body:
                if isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
                    fields.add(item.target.id)
    if not fields:
        return [f"{path}: RelocalizeConfig fields not found"]
    bad = sorted(
        {
            (node.attr, node.lineno)
            for node in ast.walk(tree)
            if isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name)
            and node.value.id == "cfg"
            and node.attr not in fields
            and not node.attr.startswith("model_")
        }
    )
    return [f"{path.name}:{line}: cfg.{attr} is not a RelocalizeConfig field" for attr, line in bad]


def check_field_tracker(lidar: Path, source: Path) -> list[str]:
    installed = lidar / "field_track.py"
    if not installed.is_file():
        return [f"{installed} is missing; run apply_reloc_field_track_patch.py"]
    if source.is_file() and installed.read_text(encoding="utf-8") != source.read_text(
        encoding="utf-8"
    ):
        return [f"{installed} differs from {source}"]
    if "DIMOS_RELOC_FIELD_TRACK_V1" not in (lidar / "module.py").read_text(encoding="utf-8"):
        return ["module.py does not call the wall tracker (DIMOS_RELOC_FIELD_TRACK_V1)"]

    import numpy as np

    spec = importlib.util.spec_from_file_location("field_track_check", installed)
    track = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(track)

    # A 6 x 4 m room with a cabinet, floor at z = 0, walls up to 1.1 m.
    side = np.arange(0.0, 6.0, 0.02)
    end = np.arange(0.0, 4.0, 0.02)
    walls = [np.column_stack([side, np.zeros_like(side)]), np.column_stack([side, np.full_like(side, 4.0)])]
    walls += [np.column_stack([np.zeros_like(end), end]), np.column_stack([np.full_like(end, 6.0), end])]
    walls += [np.column_stack([np.full(40, 4.0), np.linspace(0.0, 0.8, 40)])]
    walls += [np.column_stack([np.linspace(4.0, 5.0, 50), np.full(50, 0.8)])]
    wall_xy = np.vstack(walls)
    wall = np.vstack([np.column_stack([wall_xy, np.full(len(wall_xy), z)]) for z in (0.4, 0.7, 1.0)])
    gx, gy = np.meshgrid(np.arange(0.1, 6.0, 0.1), np.arange(0.1, 4.0, 0.1))
    floor = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
    room = np.vstack([wall, floor])

    def pose(x: float, y: float, yaw_deg: float) -> np.ndarray:
        mat = np.eye(4)
        a = math.radians(yaw_deg)
        mat[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
        mat[:2, 3] = (x, y)
        return mat

    truth = pose(1.5, -0.7, 35.0)  # world<-map
    cloud = room @ truth[:3, :3].T + truth[:3, 3]
    dog = truth @ np.array([2.5, 2.0, 0.0, 1.0])
    odom = (float(dog[0]), float(dog[1]), 0.3)
    wm = truth @ pose(0.12, -0.08, 4.0)
    tracker = track.Tracker(room)
    tracker.trust(0.10, 2.0)
    for _ in range(15):
        res = tracker.update(cloud, wm, odom)
        if res.wm is not None:
            wm = res.wm
    off = np.linalg.inv(truth) @ wm
    yaw_err = abs(math.degrees(math.atan2(off[1, 0], off[0, 0])))
    here = np.linalg.inv(wm) @ dog
    xy_err = math.hypot(here[0] - 2.5, here[1] - 2.0)
    if yaw_err > 0.5 or xy_err > 0.04:
        return [
            f"wall tracker left a knocked pose {yaw_err:.2f} deg / {xy_err:.3f} m off "
            "(limit 0.5 deg / 0.04 m)"
        ]
    return []


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    site = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else repo / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    lidar = site / "dimos/mapping/relocalization/lidar"
    problems = check_config_fields(lidar / "relocalize.py")
    try:
        problems += check_field_tracker(lidar, Path(__file__).with_name("reloc_field_track.py"))
    except Exception as exc:
        problems.append(f"wall tracker self-check raised {type(exc).__name__}: {exc}")
    for line in problems:
        print(line, file=sys.stderr)
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
