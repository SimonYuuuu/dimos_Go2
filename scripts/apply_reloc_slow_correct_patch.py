#!/usr/bin/env python3
"""Stop a small lidar bias from walking the map off over a long patrol.

Each scan was allowed to rotate the map 0.8 deg. The error sat near 2 deg, so
the correction ran at full speed until it hit the 5 deg rail, then ran back.
Over one loop the room swung about 10 deg and a metre, and by loop 4 the live
cloud no longer sat on the costmap.

Small mismatches are now ignored. A correction that keeps the same sign for
four scans moves the map by at most 0.15 deg and 2 cm.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_SLOW_CORRECT_V1"

OLD = """                    self._recover_step = recover
                    if recover:
                        step_yaw = max(-4.0, min(4.0, 0.5 * lock_yaw))
                        scale = min(0.08, 0.5 * lock_xy) / lock_xy if lock_xy > 1e-4 else 0.0
                        base_x, base_y, base_yaw = exp_x, exp_y, exp_yaw
                    else:
                        step_yaw = max(-0.8, min(0.8, 0.45 * err_yaw))
                        scale = min(0.05, 0.45 * err_xy) / err_xy if err_xy > 1e-4 else 0.0
                        base_x, base_y, base_yaw = exp_x, exp_y, exp_yaw
                    # DIMOS_RELOC_YAW_BUDGET_V1 — lidar may bend odometry yaw only a few degrees.
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
"""

NEW = """                    self._recover_step = recover
                    # DIMOS_RELOC_SLOW_CORRECT_V1
                    _ey = lock_yaw if recover else err_yaw
                    _ex = lock_xy if recover else err_xy
                    _sign = 1 if _ey > 2.5 else (-1 if _ey < -2.5 else 0)
                    _agree = list(getattr(self, "_slow_agree", []))
                    _agree.append(_sign)
                    self._slow_agree = _agree[-4:]
                    _same = (
                        len(self._slow_agree) == 4
                        and len(set(self._slow_agree)) == 1
                        and self._slow_agree[0] != 0
                    )
                    if not _same and _ex < 0.20:
                        step_yaw = 0.0
                        scale = 0.0
                    else:
                        _cap = 0.4 if recover and abs(_ey) > 6.0 else 0.15
                        step_yaw = max(-_cap, min(_cap, 0.2 * _ey))
                        _move = min(0.04 if recover else 0.02, _ex)
                        scale = _move / _ex if _ex > 1e-4 else 0.0
                    base_x, base_y, base_yaw = exp_x, exp_y, exp_yaw
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
    hold_old = "                    if abs(step_yaw) < 0.25 and (lock_xy if recover else err_xy) < 0.08:\n"
    hold_new = "                    if abs(step_yaw) < 0.05 and scale < 1e-6:\n"
    if text.count(OLD) != 1 or text.count(hold_old) != 1:
        print(f"anchor found step={text.count(OLD)} hold={text.count(hold_old)}", file=sys.stderr)
        return 1
    text = text.replace(OLD, NEW, 1).replace(hold_old, hold_new, 1)
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
