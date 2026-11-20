#!/usr/bin/env python3
"""Turn in place when the heading is off, then drive straight.

Creep-forward existed because Sport Move dropped a pure yaw command while
obstacle avoidance was switched on. Those commands now use the avoidance
Move API, which accepts yaw, so a heading change no longer has to arc.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_SPIN_FIRST_V1"

UPSTREAM_ADVANCE = """        # Rotate-then-drive: if heading error is large, rotate in place first
        if abs(yaw_error) > self._rotation_threshold:
            return self._angular_twist(angular_velocity)
"""

CREEP_ADVANCE = """        # DIMOS_NAV_CREEP_TURN_V1
        # Rotate-then-drive: simulation may spin in place. Real Go2 OA drops yaw-only Move.
        if abs(yaw_error) > self._rotation_threshold:
            if self._simulation:
                return self._angular_twist(angular_velocity)
            return Twist(
                linear=Vector3(self._min_linear_velocity, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, angular_velocity),
            )
"""

SPIN_ADVANCE = """        # DIMOS_NAV_SPIN_FIRST_V1
        # Heading off by more than ~20 deg: yaw in place, then drive straight.
        if abs(yaw_error) > self._rotation_threshold:
            return self._angular_twist(angular_velocity)
"""

UPSTREAM_ROTATE = """    def rotate(self, yaw_error: float) -> Twist:
        angular_velocity = self._compute_angular_velocity(yaw_error)
        return self._angular_twist(angular_velocity)
"""

CREEP_ROTATE = """    def rotate(self, yaw_error: float) -> Twist:
        angular_velocity = self._compute_angular_velocity(yaw_error)
        if self._simulation:
            return self._angular_twist(angular_velocity)
        # DIMOS_NAV_CREEP_TURN_V1: creep forward so Sport Move is not yaw-only.
        return Twist(
            linear=Vector3(self._min_linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )
"""

SPIN_ROTATE = """    def rotate(self, yaw_error: float) -> Twist:
        angular_velocity = self._compute_angular_velocity(yaw_error)
        # DIMOS_NAV_SPIN_FIRST_V1
        return self._angular_twist(angular_velocity)
"""

OLD_THRESHOLD = "    _rotation_threshold: float = 90 * (math.pi / 180)\n"
NEW_THRESHOLD = (
    "    _rotation_threshold: float = 20 * (math.pi / 180)  # DIMOS_NAV_SPIN_FIRST_V1\n"
)


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star/controllers.py"
    )
    text = target.read_text(encoding="utf-8")
    if MARKER in text and "CREEP_TURN_V1" not in text:
        print(f"already patched: {target}")
        return 0
    if CREEP_ADVANCE in text:
        text = text.replace(CREEP_ADVANCE, SPIN_ADVANCE, 1)
    elif UPSTREAM_ADVANCE in text:
        text = text.replace(UPSTREAM_ADVANCE, SPIN_ADVANCE, 1)
    elif MARKER not in text:
        print("advance block not found", file=sys.stderr)
        return 1
    if CREEP_ROTATE in text:
        text = text.replace(CREEP_ROTATE, SPIN_ROTATE, 1)
    elif UPSTREAM_ROTATE in text:
        text = text.replace(UPSTREAM_ROTATE, SPIN_ROTATE, 1)
    elif "def rotate(self, yaw_error: float)" not in text:
        print("rotate block not found", file=sys.stderr)
        return 1
    if OLD_THRESHOLD in text:
        text = text.replace(OLD_THRESHOLD, NEW_THRESHOLD, 1)
    elif "20 * (math.pi / 180)" not in text:
        print("rotation threshold not found", file=sys.stderr)
        return 1
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
