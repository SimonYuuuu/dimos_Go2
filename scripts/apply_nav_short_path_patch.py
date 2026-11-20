#!/usr/bin/env python3
"""Replace the cost-valley polyline with straight free-space segments.

A* on the inflated gradient walks the cheap cells, so the drawn path
weaves even when the room is open. Pull each stretch tight when the
straight line does not cross a real obstacle.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_SHORT_PATH_V1"
OLD_IMPORT = "from dimos.mapping.occupancy.path_resampling import smooth_resample_path\n"
NEW_IMPORT = (
    "from dimos.mapping.occupancy.path_resampling import simple_resample_path, smooth_resample_path\n"
)
OLD_CALL = "        resampled_path = smooth_resample_path(path, current_goal, 0.1)\n"
NEW_CALL = "        resampled_path = simple_resample_path(self._shortcut_path(path), current_goal, 0.45)  # DIMOS_NAV_SHORT_PATH_V1\n"
HELPER = '''
    def _shortcut_path(self, path: Path) -> Path:
        """Drop bends whose straight line stays off occupied cells."""
        poses = path.poses
        if len(poses) < 3:
            return path
        try:
            grid = self._navigation_map.binary_costmap
        except Exception:
            return path
        kept = [0]
        i = 0
        last = len(poses) - 1
        while i < last:
            best = i + 1
            for j in range(last, i, -1):
                if self._segment_clear(grid, poses[i].position, poses[j].position):
                    best = j
                    break
            kept.append(best)
            i = best
        return Path(frame_id=path.frame_id, poses=[poses[k] for k in kept])

    def _segment_clear(self, grid: OccupancyGrid, a: Vector3, b: Vector3) -> bool:
        import math

        dx, dy = float(b.x - a.x), float(b.y - a.y)
        dist = math.hypot(dx, dy)
        steps = max(1, int(dist / 0.15))
        # A hand's width of clearance so the shortcut does not shave a wall.
        nx, ny = -dy, dx
        norm = math.hypot(nx, ny) or 1.0
        ox, oy = 0.2 * nx / norm, 0.2 * ny / norm
        for s in range(steps + 1):
            t = s / steps
            x = float(a.x) + dx * t
            y = float(a.y) + dy * t
            for sx, sy in ((0.0, 0.0), (ox, oy), (-ox, -oy)):
                if grid.cell_value(Vector3(x + sx, y + sy, 0.0)) >= CostValues.OCCUPIED:
                    return False
        return True

'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star/global_planner.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if OLD_IMPORT not in text or OLD_CALL not in text or "    def _find_wide_path(" not in text:
        print("planner block not found", file=sys.stderr)
        return 1
    text = text.replace(OLD_IMPORT, NEW_IMPORT, 1).replace(OLD_CALL, NEW_CALL, 1)
    text = text.replace("    def _find_wide_path(", HELPER + "    def _find_wide_path(", 1)
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
