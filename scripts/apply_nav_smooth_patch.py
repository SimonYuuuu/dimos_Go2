#!/usr/bin/env python3
"""Stop for obstacles only when they are close and persist; resume at once.

The local planner stopped on a single costmap frame that put an occupied cell
anywhere in the next 3 m of path. The global planner then waited a fixed 1.5 s
even when the replanned path was clear, so the dog walked in bursts.

- Obstacle check looks DIMOS_PATH_LOOKAHEAD_M ahead (default 1.2 m).
- It stops after DIMOS_OBSTACLE_CONFIRM_N consecutive hits (default 3).
- A clear replanned path starts immediately; the hold only applies when the
  new path is still blocked.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_SMOOTH_V1"


def edit(path: Path, pairs: list[tuple[str, str]]) -> None:
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path.name} {MARKER}")
        return
    for old, new in pairs:
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"{path.name}: anchor found {count} times: {old[:60]!r}")
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} -> {MARKER}")


def main() -> None:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    nav = site / "dimos/navigation/go2/replanning_a_star"
    edit(
        nav / "path_clearance.py",
        [
            (
                "    _path_lookup_distance: float = 3.0\n",
                f"    _path_lookup_distance: float = float(  # {MARKER}\n"
                '        __import__("os").environ.get("DIMOS_PATH_LOOKAHEAD_M", "1.2")\n'
                "    )\n",
            )
        ],
    )
    edit(
        nav / "local_planner.py",
        [
            (
                "            if path_clearance.is_obstacle_ahead():\n"
                '                logger.info("Obstacle detected ahead, stopping local planner.")\n',
                f"            # {MARKER} — one noisy costmap frame must not stop the dog.\n"
                "            if path_clearance.is_obstacle_ahead():\n"
                '                self._obstacle_hits = getattr(self, "_obstacle_hits", 0) + 1\n'
                "            else:\n"
                "                self._obstacle_hits = 0\n"
                '            confirm_n = int(__import__("os").environ.get("DIMOS_OBSTACLE_CONFIRM_N", "3"))\n'
                "            if self._obstacle_hits >= confirm_n:\n"
                "                self._obstacle_hits = 0\n"
                '                logger.info("Obstacle detected ahead, stopping local planner.")\n',
            )
        ],
    )
    edit(
        nav / "global_planner.py",
        [
            (
                "            self._obstacle_hold_until = time.perf_counter() + 1.5\n"
                "            self._replan_path()\n",
                f"            self._replan_path()  # {MARKER}: hold only if the new path is blocked\n",
            ),
            (
                "        if blocked or holding:\n",
                "        if blocked:\n",
            ),
        ],
    )


if __name__ == "__main__":
    main()
