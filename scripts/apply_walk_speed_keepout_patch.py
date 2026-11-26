#!/usr/bin/env python3
"""Walk speed from the console, and quadrilateral keepouts on the costmap.

Speed is read from PATROL_SPEED_FILE on every command, so the value locked at
pose confirm applies without restarting. Keepout quads are in the loaded-map
frame; each costmap update paints their interior occupied in the world frame.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_WALK_SPEED_KEEPOUT_V1"

SPEED_FN = '''
def _walk_mps() -> float:
    # DIMOS_WALK_SPEED_KEEPOUT_V1 — locked when the operator confirms the pose.
    import json
    import os

    value = 0.35
    path = os.environ.get("PATROL_SPEED_FILE", "").strip()
    try:
        if path:
            value = float(json.loads(open(path, encoding="utf-8").read()).get("mps", value))
    except (OSError, ValueError, TypeError):
        value = 0.35
    if value != value:
        value = 0.35
    return min(1.0, max(0.05, value))


'''

KEEPOUT_FN = '''
def _paint_keepouts(grid) -> None:
    # DIMOS_WALK_SPEED_KEEPOUT_V1 — map-frame quads become occupied cells.
    import json
    import os

    zone_path = os.environ.get("PATROL_KEEPOUT_FILE", "").strip()
    tf_path = os.environ.get("PATROL_TRANSFORM_FILE", "").strip()
    if not zone_path or not tf_path or grid is None or getattr(grid, "grid", None) is None:
        return
    try:
        zones = json.loads(open(zone_path, encoding="utf-8").read())
        transform = json.loads(open(tf_path, encoding="utf-8").read())
        tx, ty, _tz = transform["translation"]
        qx, qy, qz, qw = transform["rotation_xyzw"]
    except (OSError, ValueError, TypeError, KeyError, IndexError):
        return
    try:
        _paint_keepouts_draw(grid, zones, tx, ty, qx, qy, qz, qw)
    except Exception:
        return


def _paint_keepouts_draw(grid, zones, tx, ty, qx, qy, qz, qw) -> None:
    import math

    import cv2
    import numpy as np

    from dimos.msgs.nav_msgs.OccupancyGrid import CostValues

    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    origin_x = float(grid.origin.position.x)
    origin_y = float(grid.origin.position.y)
    resolution = float(grid.resolution) or 0.05
    mask = np.zeros(grid.grid.shape, np.uint8)
    for zone in zones if isinstance(zones, list) else []:
        points = zone.get("points") if isinstance(zone, dict) else zone
        if not isinstance(points, list) or len(points) < 3:
            continue
        pixels = []
        for point in points:
            x, y = float(point["x"]), float(point["y"])
            world_x = tx + cos_yaw * x - sin_yaw * y
            world_y = ty + sin_yaw * x + cos_yaw * y
            pixels.append([int((world_x - origin_x) / resolution), int((world_y - origin_y) / resolution)])
        cv2.fillPoly(mask, [np.asarray(pixels, np.int32)], 255)
    grid.grid[mask > 0] = CostValues.OCCUPIED


'''


def edit(path: Path, pairs: list[tuple[str, str]], prefix: str = "") -> None:
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path.name} {MARKER}")
        return
    for old, new in pairs:
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"{path.name}: anchor found {count} times: {old[:60]!r}")
        text = text.replace(old, new, 1)
    if prefix:
        text = prefix + text
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} -> {MARKER}")


def main() -> None:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    edit(
        site / "dimos/robot/unitree/connection.py",
        [
            (
                "        x = float(np.clip(x, -0.15, 0.40))  # DIMOS_CMD_WALK_V1\n",
                "        x = float(np.clip(x, -0.15, _walk_mps()))  # DIMOS_CMD_WALK_V1\n",
            )
        ],
        prefix=SPEED_FN,
    )
    edit(
        site / "dimos/navigation/go2/replanning_a_star/controllers.py",
        [
            (
                "            linear=Vector3(0.35, 0.0, 0.0),\n",
                "            linear=Vector3(_walk_mps(), 0.0, 0.0),\n",
            )
        ],
        prefix=SPEED_FN,
    )
    edit(
        site / "dimos/navigation/go2/replanning_a_star/navigation_map.py",
        [
            (
                "        if fused is None:\n"
                "            return static\n"
                "        if time.monotonic() - float(getattr(self, \"_fused_at\", 0.0)) > 0.6:\n"
                "            return static\n"
                "        return fused\n",
                "        chosen = static\n"
                "        if fused is not None and time.monotonic() - float(getattr(self, \"_fused_at\", 0.0)) <= 0.6:\n"
                "            chosen = fused\n"
                "        if chosen is None:\n"
                "            return None\n"
                "        painted = chosen.copy()\n"
                "        _paint_keepouts(painted)\n"
                "        return painted\n",
            ),
            (
                "                    fused.grid[keep // width, keep % width] = CostValues.OCCUPIED\n"
                "        with self._lock:\n",
                "                    fused.grid[keep // width, keep % width] = CostValues.OCCUPIED\n"
                "        _paint_keepouts(fused)\n"
                "        with self._lock:\n",
            ),
        ],
        prefix=KEEPOUT_FN,
    )


if __name__ == "__main__":
    main()
