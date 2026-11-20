#!/usr/bin/env python3
"""Don't wait forever on a yaw-only command the body is ignoring.

The dog turns in place when the heading is off, then drives straight.
If that pure yaw does not change the heading within 1.5 s, drive anyway.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_YAW_FALLBACK_V1"

OLD_SIG = (
    "    def advance(self, lookahead_point: NDArray[np.float64], "
    "current_odom: PoseStamped) -> Twist:"
)
NEW_SIG = (
    "    def advance(self, lookahead_point: NDArray[np.float64], "
    "current_odom: PoseStamped, spin: bool = True) -> Twist:"
)
OLD_PROTO = (
    "    def advance(self, lookahead_point: NDArray[np.float64], "
    "current_odom: PoseStamped) -> Twist: ..."
)
NEW_PROTO = (
    "    def advance(self, lookahead_point: NDArray[np.float64], "
    "current_odom: PoseStamped, spin: bool = True) -> Twist: ..."
)
OLD_IF = """        if abs(yaw_error) > self._rotation_threshold:
            return self._angular_twist(angular_velocity)
"""
NEW_IF = """        # DIMOS_NAV_YAW_FALLBACK_V1
        if spin and abs(yaw_error) > self._rotation_threshold:
            return self._angular_twist(angular_velocity)
"""

OLD_INITIAL = '''        if abs(yaw_error) < self._orientation_tolerance:
            with self._lock:
                self._change_state("path_following")
            return self._compute_path_following()

        return self._controller.rotate(yaw_error)
'''

NEW_INITIAL = '''        if abs(yaw_error) < self._orientation_tolerance:
            self._yaw_only_ignored(robot_yaw, False)
            with self._lock:
                self._change_state("path_following")
            return self._compute_path_following()

        # DIMOS_NAV_YAW_FALLBACK_V1 — a dropped yaw-only command must not freeze the dog.
        if self._yaw_only_ignored(robot_yaw, True):
            logger.info("in-place turn had no effect; driving")
            self._yaw_drive_until = time.monotonic() + 3.0
            self._yaw_only_ignored(robot_yaw, False)
            with self._lock:
                self._change_state("path_following")
            return self._compute_path_following()

        return self._controller.rotate(yaw_error)
'''

OLD_FOLLOW = '''        lookahead_point = path_distancer.find_lookahead_point(closest_index)

        return self._controller.advance(lookahead_point, current_odom)
'''

NEW_FOLLOW = '''        lookahead_point = path_distancer.find_lookahead_point(closest_index)

        # DIMOS_NAV_YAW_FALLBACK_V1
        yaw = float(current_odom.orientation.euler[2])
        if time.monotonic() < getattr(self, "_yaw_drive_until", 0.0):
            self._yaw_only_ignored(yaw, False)
            return self._controller.advance(lookahead_point, current_odom, spin=False)
        cmd = self._controller.advance(lookahead_point, current_odom)
        pure = abs(float(cmd.linear.x)) < 1e-4 and abs(float(cmd.angular.z)) > 1e-4
        if self._yaw_only_ignored(yaw, pure):
            logger.info("in-place turn had no effect; driving the straight segment")
            self._yaw_drive_until = time.monotonic() + 3.0
            self._yaw_only_ignored(yaw, False)
            return self._controller.advance(lookahead_point, current_odom, spin=False)
        return cmd
'''

HELPER = '''
    def _yaw_only_ignored(self, odom_yaw: float, commanding: bool) -> bool:
        # DIMOS_NAV_YAW_FALLBACK_V1
        if not commanding:
            self._pure_yaw_t = None
            return False
        now = time.monotonic()
        started = getattr(self, "_pure_yaw_t", None)
        base = getattr(self, "_pure_yaw_base", None)
        if started is None or base is None:
            self._pure_yaw_t = now
            self._pure_yaw_base = odom_yaw
            return False
        if abs(angle_diff(odom_yaw, base)) > 0.07:
            self._pure_yaw_t = now
            self._pure_yaw_base = odom_yaw
            return False
        return (now - started) > 1.5

'''


def _planner_root(argv: list[str]) -> Path:
    if len(argv) > 1:
        return Path(argv[1])
    return (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star"
    )


def main() -> int:
    root = _planner_root(sys.argv)
    controller = root / "controllers.py"
    planner = root / "local_planner.py"
    if not controller.is_file() or not planner.is_file():
        print(f"not found under {root}", file=sys.stderr)
        return 1
    text = controller.read_text(encoding="utf-8")
    plan = planner.read_text(encoding="utf-8")
    if MARKER in plan and "spin: bool = True" in text:
        print(f"already patched: {root}")
        return 0
    if OLD_PROTO not in text or OLD_SIG not in text or OLD_IF not in text:
        print("controller block not found", file=sys.stderr)
        return 1
    if OLD_INITIAL not in plan or OLD_FOLLOW not in plan:
        print("planner block not found", file=sys.stderr)
        return 1
    if "def _compute_initial_rotation" not in plan:
        print("initial rotation not found", file=sys.stderr)
        return 1
    text = (
        text.replace(OLD_PROTO, NEW_PROTO, 1)
        .replace(OLD_SIG, NEW_SIG, 1)
        .replace(OLD_IF, NEW_IF, 1)
    )
    plan = plan.replace(OLD_INITIAL, NEW_INITIAL, 1).replace(OLD_FOLLOW, NEW_FOLLOW, 1)
    plan = plan.replace(
        "    def _compute_initial_rotation(self) -> Twist:\n",
        HELPER + "    def _compute_initial_rotation(self) -> Twist:\n",
        1,
    )
    controller.write_text(text, encoding="utf-8")
    planner.write_text(plan, encoding="utf-8")
    print(f"patched {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
