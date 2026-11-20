#!/usr/bin/env python3
"""Do not treat in-place rotation as 'stuck'.

ReplanningAStar declares stuck when XY moves <0.4 m in 8 s. initial_rotation
and final_rotation are pure yaw, so the dog never starts path_following.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_ROTATION_STUCK_V1"

OLD = '''            if (
                time.perf_counter() - last_stuck_check > self._stuck_time_window
                and self._position_tracker.is_stuck()
            ):
                logger.info("Robot is stuck. Replanning.")
                self._replan_path()
                last_stuck_check = time.perf_counter()
'''

NEW = '''            if (
                time.perf_counter() - last_stuck_check > self._stuck_time_window
                and self._position_tracker.is_stuck()
            ):
                # DIMOS_NAV_ROTATION_STUCK_V1
                state, _sid = self._local_planner.get_unique_state()
                if state in ("initial_rotation", "final_rotation"):
                    last_stuck_check = time.perf_counter()
                    continue
                logger.info("Robot is stuck. Replanning.")
                self._replan_path()
                last_stuck_check = time.perf_counter()
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
    if OLD not in text:
        print("stuck block not found", file=sys.stderr)
        return 1
    backup = target.with_suffix(target.suffix + ".pre_rot_stuck")
    if not backup.exists():
        backup.write_text(text, encoding="utf-8")
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
