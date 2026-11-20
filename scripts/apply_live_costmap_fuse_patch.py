#!/usr/bin/env python3
"""Draw live obstacles into the existing costmap, and nowhere else.

Points outside the prior grid are dropped, so a drifted scan cannot
enlarge the map. Points inside, near the dog and between the floor and
1 m, mark cells until the next scan. The prior walls stay.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_LIVE_COSTMAP_FUSE_V1"


def _root() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages/dimos"
    )


OLD_UPDATE = '''    def update(self, occupancy_grid: OccupancyGrid) -> None:
        with self._lock:
            self._binary = occupancy_grid
'''

NEW_UPDATE = '''    def update(self, occupancy_grid: OccupancyGrid) -> None:
        with self._lock:
            # DIMOS_LIVE_COSTMAP_FUSE_V1 — keep the prior room. A larger grid is drift.
            shape = getattr(occupancy_grid, "grid", None)
            if shape is not None and getattr(shape, "size", 0):
                fixed = getattr(self, "_fixed_shape", None)
                if fixed is None:
                    self._fixed_shape = tuple(shape.shape)
                elif shape.shape[0] > fixed[0] + 4 or shape.shape[1] > fixed[1] + 4:
                    if not getattr(self, "_grew_logged", False):
                        self._grew_logged = True
                        logger.info(
                            "costmap grew past the prior room (%s -> %s); keeping the original",
                            fixed,
                            tuple(shape.shape),
                        )
                    return
            self._binary = occupancy_grid
            self._fused = None
'''

OLD_BINARY = '''        with self._lock:
            if self._binary is None:
                raise ValueError("No current global costmap available")

            return self._binary
'''

NEW_BINARY = '''        with self._lock:
            grid = self._grid_for_planning()
            if grid is None:
                raise ValueError("No current global costmap available")

            return grid
'''

OLD_GRADIENT = '''        with self._lock:
            binary = self._binary
            if binary is None:
                raise ValueError("No current global costmap available")
'''

NEW_GRADIENT = '''        with self._lock:
            binary = self._grid_for_planning()
            if binary is None:
                raise ValueError("No current global costmap available")
'''

HELPER = '''
    def _grid_for_planning(self) -> OccupancyGrid | None:
        """Prior costmap, with live obstacles stamped inside the same cells."""
        import time

        static = self._binary
        fused = getattr(self, "_fused", None)
        if fused is None:
            return static
        if time.monotonic() - float(getattr(self, "_fused_at", 0.0)) > 0.6:
            return static
        return fused

    def stamp_live(self, world, rx: float, ry: float) -> None:
        """DIMOS_LIVE_COSTMAP_FUSE_V1 — mark nearby obstacles that land inside the grid."""
        import time

        import numpy as np

        from dimos.msgs.nav_msgs.OccupancyGrid import CostValues

        with self._lock:
            static = self._binary
        if static is None or static.grid.size == 0 or world is None or len(world) < 30:
            return
        pts = np.asarray(world, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] < 3:
            return
        near = np.hypot(pts[:, 0] - rx, pts[:, 1] - ry)
        nearby = pts[near < 3.5]
        if len(nearby) < 30:
            with self._lock:
                self._fused = None
            return
        ground = float(np.percentile(nearby[:, 2], 15))
        band = nearby[(nearby[:, 2] > ground + 0.25) & (nearby[:, 2] <= ground + 1.0)]
        if len(band):
            dist = np.hypot(band[:, 0] - rx, band[:, 1] - ry)
            band = band[(dist > 0.45) & (dist <= 3.5)]
        fused = static.copy()
        if len(band):
            ox = float(static.origin.position.x)
            oy = float(static.origin.position.y)
            res = float(static.resolution) or 1.0
            gx = np.floor((band[:, 0] - ox) / res).astype(np.int32)
            gy = np.floor((band[:, 1] - oy) / res).astype(np.int32)
            height, width = fused.grid.shape
            inside = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
            gx = gx[inside]
            gy = gy[inside]
            if len(gx):
                flat = gy.astype(np.int64) * width + gx.astype(np.int64)
                cells, counts = np.unique(flat, return_counts=True)
                keep = cells[counts >= 3]
                if len(keep):
                    fused.grid[keep // width, keep % width] = CostValues.OCCUPIED
        with self._lock:
            if self._binary is static:
                self._fused = fused
                self._fused_at = time.monotonic()

'''

NOTE_OLD = '''            self._live_pts = pts
            self._live_recv = time.monotonic()
'''

NOTE_NEW = '''            self._live_pts = pts
            self._live_recv = time.monotonic()
            self._stamp_costmaps(pts)
'''

STAMP_FN = '''
    def _stamp_costmaps(self, pts) -> None:
        """DIMOS_LIVE_COSTMAP_FUSE_V1 — live obstacles inside the prior costmap only."""
        odom = self._current_odom
        if odom is None or pts is None:
            return
        try:
            rx = float(odom.position.x)
            ry = float(odom.position.y)
            yaw = float(odom.orientation.euler.z)
            world = _scan_in_world(np.asarray(pts, dtype=np.float32), rx, ry, yaw)
        except Exception:
            return
        maps = getattr(self, "_live_maps", None)
        if not maps:
            maps = (self._navigation_map,)
        for nav in maps:
            stamp = getattr(nav, "stamp_live", None)
            if stamp is not None:
                try:
                    stamp(world, rx, ry)
                except Exception:
                    if not getattr(self, "_stamp_err", False):
                        self._stamp_err = True
                        logger.exception("Live costmap stamp failed")

'''

PLANNER_OLD = '''            simulation=simulation,
        )
'''

PLANNER_NEW = '''            simulation=simulation,
        )
        # DIMOS_LIVE_COSTMAP_FUSE_V1 — both distance bands share the live obstacles.
        self._local_planner._live_maps = (self._navigation_map, self._navigation_map_near)
'''


def _patch(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text(encoding="utf-8")
    if old not in text:
        raise SystemExit(f"{label} anchor not found in {path}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def main() -> int:
    root = _root()
    nav = root / "navigation/go2/replanning_a_star/navigation_map.py"
    local = root / "navigation/go2/replanning_a_star/local_planner.py"
    planner = root / "navigation/go2/replanning_a_star/global_planner.py"
    for path in (nav, local, planner):
        if not path.is_file():
            print(f"not found: {path}", file=sys.stderr)
            return 1
    nav_text = nav.read_text(encoding="utf-8")
    local_text = local.read_text(encoding="utf-8")
    planner_text = planner.read_text(encoding="utf-8")
    if (
        MARKER in nav_text
        and "_stamp_costmaps" in local_text
        and "_live_maps" in planner_text
    ):
        print(f"already patched: {nav}")
        return 0
    if MARKER not in nav_text:
        if "import logging" not in nav_text and "setup_logger" not in nav_text:
            nav_text = nav_text.replace(
                "from threading import RLock\n",
                "from threading import RLock\n\n"
                "from dimos.utils.logging_config import setup_logger\n\n"
                "logger = setup_logger()\n",
                1,
            )
        if OLD_UPDATE not in nav_text:
            print("navigation map update() not found", file=sys.stderr)
            return 1
        nav_text = nav_text.replace(OLD_UPDATE, NEW_UPDATE, 1)
        nav_text = nav_text.replace(OLD_BINARY, NEW_BINARY, 1)
        nav_text = nav_text.replace(OLD_GRADIENT, NEW_GRADIENT, 1)
        tail = '''        return make_navigation_map(
            binary,
            self._robot_width * robot_increase,
            strategy="simple",
            gradient_strategy=self._gradient_strategy,
        )
'''
        if tail not in nav_text:
            print("gradient return not found", file=sys.stderr)
            return 1
        nav_text = nav_text.replace(tail, tail + HELPER, 1)
        nav.write_text(nav_text, encoding="utf-8")
    local_text = local.read_text(encoding="utf-8")
    if "_stamp_costmaps" not in local_text:
        if NOTE_OLD not in local_text:
            print("note_lidar store not found", file=sys.stderr)
            return 1
        local_text = local_text.replace(NOTE_OLD, NOTE_NEW, 1)
        anchor = "    def _live_path_blocked(self) -> bool:\n"
        if anchor not in local_text:
            print("live pause method not found", file=sys.stderr)
            return 1
        local_text = local_text.replace(anchor, STAMP_FN + anchor, 1)
        local.write_text(local_text, encoding="utf-8")
    planner_text = planner.read_text(encoding="utf-8")
    if "_live_maps" not in planner_text:
        if planner_text.count(PLANNER_OLD) != 1:
            print("local planner construction is not unique", file=sys.stderr)
            return 1
        planner.write_text(planner_text.replace(PLANNER_OLD, PLANNER_NEW, 1), encoding="utf-8")
    if (
        MARKER in nav.read_text(encoding="utf-8")
        and "_stamp_costmaps" in local.read_text(encoding="utf-8")
        and "_live_maps" in planner.read_text(encoding="utf-8")
    ):
        print(f"patched costmap fusion under {root}")
        return 0
    print("costmap fusion incomplete", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
