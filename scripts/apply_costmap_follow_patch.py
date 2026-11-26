#!/usr/bin/env python3
"""Let the planning costmap follow a pose nudge.

The first grid was frozen for the whole patrol. A 5-cell growth, which is what
a yaw nudge does to the room's bounding box, was thrown away. Goals then used
the nudged frame while obstacles stayed in the old one, so the path bent and
later points had no connection.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_COSTMAP_FOLLOW_V1"

OLD = """                elif shape.shape[0] > fixed[0] + 4 or shape.shape[1] > fixed[1] + 4:
                    if not getattr(self, "_grew_logged", False):
                        self._grew_logged = True
                        logger.info(
                            "costmap grew past the prior room (%s -> %s); keeping the original",
                            fixed,
                            tuple(shape.shape),
                        )
                    return
"""

NEW = """                elif shape.shape[0] > fixed[0] + 4 or shape.shape[1] > fixed[1] + 4:
                    # DIMOS_COSTMAP_FOLLOW_V1 — a nudge moves the room's corner.
                    # Keep that grid. Only a corner that stayed put is drift.
                    import math

                    old = self._binary
                    moved = True
                    try:
                        dx = float(occupancy_grid.origin.position.x) - float(old.origin.position.x)
                        dy = float(occupancy_grid.origin.position.y) - float(old.origin.position.y)
                        dyaw = float(occupancy_grid.origin.orientation.euler.z) - float(
                            old.origin.orientation.euler.z
                        )
                        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
                        moved = math.hypot(dx, dy) > 0.15 or abs(dyaw) > math.radians(1.5)
                    except (AttributeError, TypeError, ValueError):
                        moved = True
                    first = getattr(self, "_first_shape", fixed)
                    huge = shape.shape[0] > first[0] + 80 or shape.shape[1] > first[1] + 80
                    if not moved or huge:
                        if not getattr(self, "_grew_logged", False):
                            self._grew_logged = True
                            logger.info(
                                "costmap grew past the prior room (%s -> %s); keeping the original",
                                fixed,
                                tuple(shape.shape),
                            )
                        return
                    logger.info(
                        "costmap origin moved; accepting the new room (%s -> %s)",
                        fixed,
                        tuple(shape.shape),
                    )
                    self._fixed_shape = tuple(shape.shape)
                    self._grew_logged = False
"""


def main() -> int:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star/navigation_map.py"
    )
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if text.count(OLD) != 1:
        print(f"anchor found {text.count(OLD)} times", file=sys.stderr)
        return 1
    if "self._first_shape = tuple(shape.shape)" not in text:
        text = text.replace(
            "                    self._fixed_shape = tuple(shape.shape)\n",
            "                    self._fixed_shape = tuple(shape.shape)\n"
            "                    self._first_shape = tuple(shape.shape)\n",
            1,
        )
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
