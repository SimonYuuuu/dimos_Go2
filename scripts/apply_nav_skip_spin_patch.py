#!/usr/bin/env python3
"""Face the path in place before driving.

An earlier workaround skipped that spin because Sport Move dropped yaw-only
commands. Velocity now goes through the avoidance service, which accepts yaw,
so the dog can turn on the spot and then drive the straight segment.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_SPIN_FIRST_V1"

UPSTREAM = '''        # Determine initial state: skip initial_rotation if already aligned.
        new_state: PlannerState = "initial_rotation"
        if current_odom is not None and len(path.poses) > 0:
            first_yaw = path.poses[0].orientation.euler[2]
            robot_yaw = current_odom.orientation.euler[2]
            initial_yaw_error = angle_diff(first_yaw, robot_yaw)
            self._controller.reset_yaw_error(initial_yaw_error)
            angle_in_tolerance = abs(initial_yaw_error) < self._orientation_tolerance
            if angle_in_tolerance:
                position_in_tolerance = (
                    path.poses[0].position.distance(current_odom.position) < 0.01
                )
                if position_in_tolerance:
                    new_state = "final_rotation"
                else:
                    new_state = "path_following"
'''

SKIP = '''        # DIMOS_NAV_SKIP_SPIN_V1
        # Go2 + obstacle avoidance ignores yaw-only Move(); drive and turn together.
        new_state: PlannerState = "path_following"
        if current_odom is not None and len(path.poses) > 0:
            first_yaw = path.poses[0].orientation.euler[2]
            robot_yaw = current_odom.orientation.euler[2]
            initial_yaw_error = angle_diff(first_yaw, robot_yaw)
            self._controller.reset_yaw_error(initial_yaw_error)
            angle_in_tolerance = abs(initial_yaw_error) < self._orientation_tolerance
            position_in_tolerance = (
                path.poses[0].position.distance(current_odom.position) < 0.15
            )
            if angle_in_tolerance and position_in_tolerance:
                new_state = "final_rotation"
'''

SPIN = '''        # DIMOS_NAV_SPIN_FIRST_V1
        # Turn on the spot until the first heading matches, then drive straight.
        new_state: PlannerState = "initial_rotation"
        if current_odom is not None and len(path.poses) > 0:
            first_yaw = path.poses[0].orientation.euler[2]
            robot_yaw = current_odom.orientation.euler[2]
            initial_yaw_error = angle_diff(first_yaw, robot_yaw)
            self._controller.reset_yaw_error(initial_yaw_error)
            angle_in_tolerance = abs(initial_yaw_error) < self._orientation_tolerance
            if angle_in_tolerance:
                position_in_tolerance = (
                    path.poses[0].position.distance(current_odom.position) < 0.15
                )
                if position_in_tolerance:
                    new_state = "final_rotation"
                else:
                    new_state = "path_following"
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star/local_planner.py"
    )
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if SKIP in text:
        text = text.replace(SKIP, SPIN, 1)
    elif UPSTREAM in text:
        text = text.replace(UPSTREAM, SPIN, 1)
    else:
        print("block not found", file=sys.stderr)
        return 1
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
