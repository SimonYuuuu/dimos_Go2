#!/usr/bin/env python3
"""Draw big clouds as screen dots, not a sphere per voxel.

A warehouse cloud as spheres freezes the web viewer even on Ethernet.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RERUN_POINTS_V2"
OLD = '''        points = self.points_f32()
        if len(points) == 0:
            return rr.Points3D([]) if mode != "boxes" else rr.Boxes3D(centers=[])
'''
NEW = '''        points = self.points_f32()
        if len(points) == 0:
            return rr.Points3D([]) if mode != "boxes" else rr.Boxes3D(centers=[])
        # DIMOS_RERUN_POINTS_V1 — spheres of the full map stall the browser.
        # max_points=0 keeps a cloud whole. The live scan stays capped.
        step = 1
        cap = int(kwargs.pop("max_points", 12000) or 0)
        if mode == "spheres":
            mode = "points"
        if cap and len(points) > cap:
            step = max(1, len(points) // cap)
            points = points[::step]
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/msgs/sensor_msgs/PointCloud2.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        old_cap = """        if len(points) > 12000:
            step = max(1, len(points) // 12000)
            points = points[::step]
"""
        new_cap = """        cap = int(kwargs.pop("max_points", 12000) or 0)
        if cap and len(points) > cap:
            step = max(1, len(points) // cap)
            points = points[::step]
"""
        if old_cap in text:
            target.write_text(text.replace(old_cap, new_cap, 1), encoding="utf-8")
            print(f"patched {target} -> global map can keep every point")
            return 0
        print(f"already patched: {target}")
        return 0
    v1 = """        if mode == \"spheres\":
            mode = \"points\"
        if len(points) > 80000:
            points = points[:: max(1, len(points) // 80000)]
"""
    v2 = """        step = 1
        cap = int(kwargs.pop(\"max_points\", 12000) or 0)
        if mode == \"spheres\":
            mode = \"points\"
        if cap and len(points) > cap:
            step = max(1, len(points) // cap)
            points = points[::step]
"""
    color = "            own = np.asarray(self.pointcloud.colors)\n"
    color_step = color + "            if step > 1:\n                own = own[::step]\n"
    if "DIMOS_RERUN_POINTS_V1" in text and v1 in text and color in text:
        text = text.replace(v1, v2, 1).replace(color, color_step, 1).replace(
            "DIMOS_RERUN_POINTS_V1", "DIMOS_RERUN_POINTS_V2", 1
        )
        target.write_text(text, encoding="utf-8")
        print(f"patched {target} -> {MARKER}")
        return 0
    if OLD not in text:
        print("could not find to_rerun point load", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
