#!/usr/bin/env python3
"""Last clamp on the velocity sent to the dog.

Planner bugs must not be able to command more than a slow walk.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_CMD_SPEED_CAP_V1"
TURN = "DIMOS_CMD_TURN_RATE_V1"
WALK = "DIMOS_CMD_WALK_V1"

OLD = '''    def _publish_movement(self, x: float, y: float, yaw: float) -> None:
        # DIMOS_AVOID_MOVE_V2 — Sport Move drives; SwitchAvoidMode steers it.
        if self._velocity_api:
'''

NEW = '''    def _publish_movement(self, x: float, y: float, yaw: float) -> None:
        # DIMOS_CMD_SPEED_CAP_V1 — slow walk. Turn rate is set separately.
        x = float(np.clip(x, -0.22, 0.22))
        y = float(np.clip(y, -0.08, 0.08))
        yaw = float(np.clip(yaw, -0.8, 0.8))  # DIMOS_CMD_TURN_RATE_V1
        # DIMOS_AVOID_MOVE_V2 — Sport Move drives; SwitchAvoidMode steers it.
        if self._velocity_api:
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/robot/unitree/connection.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if WALK in text:
        print(f"already patched: {target}")
        return 0
    old_x = "        x = float(np.clip(x, -0.22, 0.22))\n"
    new_x = "        x = float(np.clip(x, -0.15, 0.40))  # DIMOS_CMD_WALK_V1\n"
    if old_x in text and TURN in text:
        target.write_text(text.replace(old_x, new_x, 1), encoding="utf-8")
        print(f"patched {target}")
        return 0
    if TURN in text:
        print("linear clip not found", file=sys.stderr)
        return 1
    slow_yaw = """        yaw = float(np.clip(yaw, -0.22, 0.22))\n"""
    fast_yaw = """        yaw = float(np.clip(yaw, -0.8, 0.8))  # DIMOS_CMD_TURN_RATE_V1\n"""
    if slow_yaw in text:
        text = text.replace(slow_yaw, fast_yaw, 1)
        target.write_text(text, encoding="utf-8")
        print(f"patched {target}")
        return 0
    if MARKER in text:
        print("yaw clip not found", file=sys.stderr)
        return 1
    if OLD not in text:
        print("publish block not found", file=sys.stderr)
        return 1
    if "import numpy as np" not in text:
        print("numpy import missing", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
