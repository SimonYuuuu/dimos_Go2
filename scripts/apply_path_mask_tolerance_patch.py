#!/usr/bin/env python3
"""Let a planned path graze wall cells instead of aborting the plan.

make_path_mask raised when more than 5% of the robot-width band touched
occupied cells, which killed planning next to walls. The limit comes from
DIMOS_PATH_MASK_OCCUPIED_MAX (default 0.25), and the global planner's
obstacle hold no longer aborts on that error.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_PATH_MASK_TOLERANCE_V1"


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
    edit(
        site / "dimos/mapping/occupancy/path_mask.py",
        [
            (
                "    if occupied_count / total_points > 0.05:\n",
                f"    import os  # {MARKER}\n"
                "\n"
                '    limit = float(os.environ.get("DIMOS_PATH_MASK_OCCUPIED_MAX", "0.25"))\n'
                "    if occupied_count / total_points > limit:\n",
            ),
            (
                '            f"More than 5% of path points are occupied: "\n',
                '            f"More than {100 * limit:.0f}% of path points are occupied: "\n',
            ),
        ],
    )
    edit(
        site / "dimos/navigation/go2/replanning_a_star/global_planner.py",
        [
            (
                "            clearance.update_costmap(grid)\n"
                "            blocked = clearance.is_obstacle_ahead()\n",
                "            clearance.update_costmap(grid)\n"
                "            try:\n"
                "                blocked = clearance.is_obstacle_ahead()\n"
                f"            except ValueError as exc:  # {MARKER}\n"
                '                logger.info(f"Path brushes mapped walls: {exc}")\n'
                "                blocked = True\n",
            )
        ],
    )


if __name__ == "__main__":
    main()
