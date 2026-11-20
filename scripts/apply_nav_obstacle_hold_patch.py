#!/usr/bin/env python3
"""Stay stopped when the mapped path is blocked.

A detected obstacle used to replan and resume in the same control cycle,
so the dog never actually braked.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_OBSTACLE_HOLD_V1"

OLD_START = """        self.path.on_next(resampled_path)

        self._local_planner.start_planning(resampled_path)
"""

NEW_START = """        # DIMOS_NAV_OBSTACLE_HOLD_V1 — do not resume into a blocked path.
        from dimos.navigation.go2.replanning_a_star.path_clearance import PathClearance

        blocked = False
        try:
            grid = self._navigation_map.binary_costmap
        except Exception:
            grid = None
        if grid is not None and getattr(grid, "grid", None) is not None and grid.grid.size:
            clearance = PathClearance(self._local_planner._robot_width, resampled_path)
            clearance.update_costmap(grid)
            blocked = clearance.is_obstacle_ahead()
        holding = time.perf_counter() < getattr(self, "_obstacle_hold_until", 0.0)
        if blocked or holding:
            if blocked:
                self._obstacle_hold_until = time.perf_counter() + 1.5
            logger.info(
                "Holding: mapped obstacle on the path."
                if blocked
                else "Obstacle hold: staying stopped."
            )
            self.path.on_next(Path())
            return

        self.path.on_next(resampled_path)

        self._local_planner.start_planning(resampled_path)
"""

OLD_STOP = """        elif stop_message == "obstacle_found":
            logger.info("Replanning path due to obstacle found.")
            self._replan_path()
"""

NEW_STOP = """        elif stop_message == "obstacle_found":
            logger.info("Replanning path due to obstacle found.")
            # DIMOS_NAV_OBSTACLE_HOLD_V1
            self._obstacle_hold_until = time.perf_counter() + 1.5
            self._replan_path()
"""

OLD_LOOP = """                if reason is not None:
                    self._handle_stop_message(reason)
                    last_stuck_check = time.perf_counter()
                    continue
"""

NEW_LOOP = """                if reason is not None:
                    self._handle_stop_message(reason)
                    last_stuck_check = time.perf_counter()
                    continue

            # DIMOS_NAV_OBSTACLE_HOLD_V1
            hold_until = getattr(self, "_obstacle_hold_until", 0.0)
            if hold_until and time.perf_counter() >= hold_until:
                self._obstacle_hold_until = 0.0
                logger.info("Obstacle hold ended. Planning again.")
                self._replan_path()
                last_stuck_check = time.perf_counter()
                continue
"""


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
    if OLD_START not in text or OLD_STOP not in text or OLD_LOOP not in text:
        print("planner blocks not found", file=sys.stderr)
        return 1
    text = text.replace(OLD_START, NEW_START, 1).replace(OLD_STOP, NEW_STOP, 1).replace(
        OLD_LOOP, NEW_LOOP, 1
    )
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
