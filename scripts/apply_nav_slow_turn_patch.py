#!/usr/bin/env python3
"""Smaller, slower heading corrections on the real robot.

The tracker clipped yaw rate to the full drive speed (about 0.55 rad/s)
and would not command less than 0.2 rad/s, so every kink was a sharp twist.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_SLOW_TURN_V1"
OLD_GAINS = """    _min_angular_velocity: float = 0.2
    _k_angular: float = 0.5
    _max_angular_accel: float = 2.0
"""
NEW_GAINS = """    _min_angular_velocity: float = 0.08  # DIMOS_NAV_SLOW_TURN_V1
    _k_angular: float = 0.25
    _max_angular_accel: float = 0.4
"""
OLD_CLIP = """        angular_velocity = self._k_angular * yaw_error
        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
"""
NEW_CLIP = """        angular_velocity = self._k_angular * yaw_error
        # DIMOS_NAV_SLOW_TURN_V1 — about 12 deg/s, not the full drive speed.
        angular_velocity = np.clip(angular_velocity, -0.22, 0.22)
"""
OLD_LOOK = "    def __init__(self, path: Path, lookahead_dist: float = 0.5) -> None:\n"
NEW_LOOK = "    def __init__(self, path: Path, lookahead_dist: float = 0.9) -> None:  # DIMOS_NAV_SLOW_TURN_V1\n"


def main() -> int:
    root = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star"
    )
    controller = root / "controllers.py"
    distancer = root / "path_distancer.py"
    if not controller.is_file() or not distancer.is_file():
        print(f"not found under {root}", file=sys.stderr)
        return 1
    text = controller.read_text(encoding="utf-8")
    if MARKER not in text:
        if OLD_GAINS not in text or OLD_CLIP not in text:
            print("controller block not found", file=sys.stderr)
            return 1
        controller.write_text(
            text.replace(OLD_GAINS, NEW_GAINS, 1).replace(OLD_CLIP, NEW_CLIP, 1),
            encoding="utf-8",
        )
    look = distancer.read_text(encoding="utf-8")
    if MARKER not in look:
        if OLD_LOOK not in look:
            print("lookahead not found", file=sys.stderr)
            return 1
        distancer.write_text(look.replace(OLD_LOOK, NEW_LOOK, 1), encoding="utf-8")
    print(f"patched {controller.parent}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
