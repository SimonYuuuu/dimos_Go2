#!/usr/bin/env python3
"""Keep the global planner thread alive when a replan races a cancel.

"Veered off track" and "stuck" call _replan_path directly. _plan_path then
asserts the goal is still set, but a concurrent cancel can clear it. The
assertion killed the thread, so later obstacle stops cleared the path and
nothing planned the next one.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_PLANNER_SURVIVE_V1"


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
    pairs = [
        (
            "                self._replan_path()\n"
            "                last_stuck_check = time.perf_counter()\n"
            "                continue\n"
            "\n"
            "            _, new_id = self._local_planner.get_unique_state()\n",
            "                try:\n"
            "                    self._replan_path()\n"
            "                except Exception:\n"
            f'                    logger.exception("Replan after leaving the path failed")  # {MARKER}\n'
            "                last_stuck_check = time.perf_counter()\n"
            "                continue\n"
            "\n"
            "            _, new_id = self._local_planner.get_unique_state()\n",
        ),
        (
            '                logger.info("Robot is stuck. Replanning.")\n'
            "                self._replan_path()\n",
            '                logger.info("Robot is stuck. Replanning.")\n'
            "                try:\n"
            "                    self._replan_path()\n"
            "                except Exception:\n"
            '                    logger.exception("Replan after stuck failed")\n',
        ),
        (
            "        assert current_odom is not None\n"
            "        assert current_goal is not None\n",
            "        if current_odom is None or current_goal is None:\n"
            '            logger.info("Replan skipped: goal already cancelled.")\n'
            "            return\n",
        ),
        (
            "        assert current_goal is not None\n"
            "\n"
            "        if current_odom is None:\n",
            "        if current_goal is None:\n"
            '            logger.info("Plan skipped: goal already cancelled.")\n'
            "            return\n"
            "\n"
            "        if current_odom is None:\n",
        ),
    ]
    for old, new in pairs:
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"{path.name}: anchor found {count} times: {old[:50]!r}")
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} -> {MARKER}")


if __name__ == "__main__":
    main()
