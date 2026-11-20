#!/usr/bin/env python3
"""Keep the global planner thread alive after an obstacle hold.

The hold timer replanned even when the goal was already cancelled, which hit
`assert current_goal is not None` and killed the planner thread; every later
obstacle stop then left the dog idle.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_HOLD_GUARD_V1"

OLD = (
    '                logger.info("Obstacle hold ended. Planning again.")\n'
    "                self._replan_path()\n"
)
NEW = (
    f"                with self._lock:  # {MARKER}\n"
    "                    held_goal = self._current_goal\n"
    "                    held_odom = self._current_odom\n"
    "                if held_goal is not None and held_odom is not None:\n"
    '                    logger.info("Obstacle hold ended. Planning again.")\n'
    "                    try:\n"
    "                        self._replan_path()\n"
    "                    except Exception:\n"
    '                        logger.exception("Replan after obstacle hold failed")\n'
)

OLD_STOP = (
    "                if reason is not None:\n"
    "                    self._handle_stop_message(reason)\n"
)
NEW_STOP = (
    "                if reason is not None:\n"
    "                    try:\n"
    "                        self._handle_stop_message(reason)\n"
    "                    except Exception:\n"
    '                        logger.exception("Handling planner stop failed")\n'
)


def main() -> None:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    path = site / "dimos/navigation/go2/replanning_a_star/global_planner.py"
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path.name} {MARKER}")
        return
    for old, new in ((OLD, NEW), (OLD_STOP, NEW_STOP)):
        if text.count(old) != 1:
            raise SystemExit(f"{path.name}: anchor found {text.count(old)} times: {old[:50]!r}")
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} -> {MARKER}")


if __name__ == "__main__":
    main()
