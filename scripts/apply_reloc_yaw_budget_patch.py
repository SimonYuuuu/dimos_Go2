#!/usr/bin/env python3
"""Bound how far lidar tracking may rotate the map away from the lock.

Tracking ICP returned sub-degree yaw corrections that leaned one way. Over
nine minutes they summed to 21 degrees, the map turned under the dog, and the
dog walked into a wall. Odometry yaw is trusted; lidar may only bend it by
DIMOS_RELOC_YAW_BUDGET_DEG from the lock (or the last operator nudge).
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_YAW_BUDGET_V1"

OLD = """                    if abs(step_yaw) < 0.25 and (lock_xy if recover else err_xy) < 0.08:
"""

NEW = """                    # DIMOS_RELOC_YAW_BUDGET_V1 — lidar may bend odometry yaw only a few degrees.
                    _anchor = getattr(self, "_settle_anchor", None)
                    if _anchor is not None:
                        _budget = float(os.environ.get("DIMOS_RELOC_YAW_BUDGET_DEG", "5"))
                        _off = (yaw_deg(last) - yaw_deg(_anchor) + 180.0) % 360.0 - 180.0
                        _new = _off - step_yaw
                        if abs(_new) > _budget:
                            if abs(_off) > _budget:
                                step_yaw = step_yaw if abs(_new) < abs(_off) else 0.0
                            else:
                                step_yaw = _off - math.copysign(_budget, _new)
                    if abs(step_yaw) < 0.25 and (lock_xy if recover else err_xy) < 0.08:
"""


def main() -> int:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/mapping/relocalization/lidar/module.py"
    )
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if text.count(OLD) != 1:
        print(f"anchor found {text.count(OLD)} times in {target}", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
