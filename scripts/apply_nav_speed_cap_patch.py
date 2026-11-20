#!/usr/bin/env python3
"""Cap drive speed and never reverse when the heading is still off.

The aligned-drive formula is only valid inside the turn threshold. Skipping
the in-place turn left a large heading error in that formula, which produced
a fast negative x and the dog backed into the wall.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_SPEED_CAP_V1"
TURN = "DIMOS_NAV_TURN_RATE_V1"
WALK = "DIMOS_NAV_WALK_GAIT_V1"

OLD = '''        # DIMOS_NAV_SPIN_FIRST_V1
        # Heading off by more than ~20 deg: yaw in place, then drive straight.
        # DIMOS_NAV_YAW_FALLBACK_V1
        if spin and abs(yaw_error) > self._rotation_threshold:
            return self._angular_twist(angular_velocity)

        # When aligned, drive forward with proportional angular correction
        linear_velocity = self._speed * (1.0 - abs(yaw_error) / self._rotation_threshold)
        linear_velocity = self._apply_min_velocity(linear_velocity, self._min_linear_velocity)
'''

NEW = '''        # DIMOS_NAV_SPIN_FIRST_V1
        # Heading off by more than ~20 deg: yaw in place, then drive straight.
        # DIMOS_NAV_YAW_FALLBACK_V1
        # DIMOS_NAV_SPEED_CAP_V1 — outside the threshold the old scale goes negative.
        # DIMOS_NAV_TURN_RATE_V1 — 0.8 rad/s is a normal in-place turn, about 46 deg/s.
        turn = 0.8 if yaw_error > 0.0 else -0.8
        if abs(yaw_error) > self._rotation_threshold:
            if spin:
                return self._angular_twist(turn)
            return Twist(
                linear=Vector3(0.12, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, turn),
            )

        # When aligned, drive forward. Never reverse, and stay at a slow walk.
        fraction = min(1.0, abs(yaw_error) / self._rotation_threshold)
        linear_velocity = min(self._speed, 0.22) * (1.0 - fraction)
        linear_velocity = self._apply_min_velocity(linear_velocity, min(self._min_linear_velocity, 0.12))
        linear_velocity = float(np.clip(linear_velocity, 0.0, 0.22))
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star/controllers.py"
    )
    text = target.read_text(encoding="utf-8")
    if WALK in text:
        print(f"already patched: {target}")
        return 0
    crawl = '''        # When aligned, drive forward. Never reverse, and stay at a slow walk.
        fraction = min(1.0, abs(yaw_error) / self._rotation_threshold)
        linear_velocity = min(self._speed, 0.22) * (1.0 - fraction)
        linear_velocity = self._apply_min_velocity(linear_velocity, min(self._min_linear_velocity, 0.12))
        linear_velocity = float(np.clip(linear_velocity, 0.0, 0.22))

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )
'''
    trot = '''        # DIMOS_NAV_WALK_GAIT_V1 — a steady 0.35 m/s trot once the turn is done.
        # Scaling speed with the leftover angle, and a minimum yaw, broke the gait.
        if abs(yaw_error) < math.radians(6):
            yaw_cmd = 0.0
        else:
            yaw_cmd = float(np.clip(0.5 * yaw_error, -0.2, 0.2))
        return Twist(
            linear=Vector3(0.35, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, yaw_cmd),
        )
'''
    if crawl in text:
        target.write_text(text.replace(crawl, trot, 1), encoding="utf-8")
        print(f"patched {target}")
        return 0
    if TURN in text:
        print("walk block not found", file=sys.stderr)
        return 1
    slow_turn = '''        # DIMOS_NAV_SPEED_CAP_V1 — outside the threshold the old scale goes negative.
        if abs(yaw_error) > self._rotation_threshold:
            if spin:
                return self._angular_twist(angular_velocity)
            return Twist(
                linear=Vector3(0.08, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, angular_velocity),
            )
'''
    fast_turn = '''        # DIMOS_NAV_SPEED_CAP_V1 — outside the threshold the old scale goes negative.
        # DIMOS_NAV_TURN_RATE_V1 — 0.8 rad/s is a normal in-place turn, about 46 deg/s.
        turn = 0.8 if yaw_error > 0.0 else -0.8
        if abs(yaw_error) > self._rotation_threshold:
            if spin:
                return self._angular_twist(turn)
            return Twist(
                linear=Vector3(0.12, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, turn),
            )
'''
    old_rotate = '''    def rotate(self, yaw_error: float) -> Twist:
        angular_velocity = self._compute_angular_velocity(yaw_error)
        # DIMOS_NAV_SPIN_FIRST_V1
        return self._angular_twist(angular_velocity)
'''
    new_rotate = '''    def rotate(self, yaw_error: float) -> Twist:
        # DIMOS_NAV_TURN_RATE_V1
        turn = 0.8 if yaw_error > 0.0 else -0.8
        return self._angular_twist(turn)
'''
    if slow_turn in text:
        text = text.replace(slow_turn, fast_turn, 1)
    elif OLD in text:
        text = text.replace(OLD, NEW, 1)
    elif MARKER not in text:
        print("speed block not found", file=sys.stderr)
        return 1
    if old_rotate in text:
        text = text.replace(old_rotate, new_rotate, 1)
    elif "def rotate(self, yaw_error: float)" not in text:
        print("rotate block not found", file=sys.stderr)
        return 1
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
