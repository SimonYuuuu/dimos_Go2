#!/usr/bin/env python3
"""Pull a drifted walking pose back when lidar keeps disagreeing.

Odometry drift of tens of degrees was rejected as "unstable" because the
check compared absolute poses (they move) and discarded the buffer once the
error passed 18 degrees. The same error then sat on the costmap and A* found
no path. A disagreement that repeats for three scans, while the dog is not
spinning, is applied from the published pose at 4 degrees and 8 cm per scan.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_RECOVER_V1"
MARKER_V2 = "DIMOS_RELOC_RECOVER_V2"

# V1 measured the recovery error from the pose at the last accepted scan. After
# a turn (scans rejected while spinning) that pose was a turn behind, so a
# lidar fix that agreed with the published pose still rotated the map. V2
# measures from the odometry-predicted published pose, and the jump limit
# checks the fused result instead of the raw lidar fix.
V1_TO_V2 = [
    (
        "                    lock_yaw = (placed_odom[2] - prev_on_map[2] + 180.0) % 360.0 - 180.0\n"
        "                    lock_xy = math.hypot(\n"
        "                        placed_odom[0] - prev_on_map[0], placed_odom[1] - prev_on_map[1]\n"
        "                    )\n",
        f"                    lock_yaw = err_yaw  # {MARKER_V2}\n"
        "                    lock_xy = err_xy\n",
    ),
    (
        "                        base_x, base_y, base_yaw = prev_on_map\n",
        "                        base_x, base_y, base_yaw = exp_x, exp_y, exp_yaw\n",
    ),
]
MARKER_V3 = "DIMOS_RELOC_RECOVER_V3"
# The odometry prediction started from the pose at the last accepted scan. A
# console nudge moved the published pose without moving that baseline, so every
# fused result sat a fixed offset from the published pose and the jump limit
# rejected tracking for the rest of the run. V3 predicts from the published pose.
PRED_OLD = "                    placed_odom = self._robot_on_map(chosen)\n"
PRED_NEW = (
    "                    placed_odom = self._robot_on_map(chosen)\n"
    f"                    published = self._robot_on_map(last)  # {MARKER_V3}\n"
    "                    if published is not None:\n"
    "                        exp_x, exp_y, exp_yaw = published\n"
)

JUMP_OLD = "            if abs(dyaw) > max_step_yaw or dist > max_step_xy:\n"
JUMP_NEW = (
    "            moved_new = self._robot_on_map(chosen)\n"
    "            moved_old = self._robot_on_map(last)\n"
    "            if moved_new is not None and moved_old is not None:\n"
    "                dyaw = (moved_new[2] - moved_old[2] + 180.0) % 360.0 - 180.0\n"
    "                dist = math.hypot(moved_new[0] - moved_old[0], moved_new[1] - moved_old[1])\n"
    "            if abs(dyaw) > max_step_yaw or dist > max_step_xy:\n"
)

OLD_FUSE = """                    spinning = abs(float(getattr(self, "_yaw_rate", 0.0))) > 0.35
                    believable = abs(err_yaw) <= 18.0 and err_xy <= 1.2
                    if spinning or not consistent or not believable:
                        why = "spin" if spinning else ("unstable" if not consistent else "far")
                        logger.info(
                            f"relocalize fuse: keep odom ({why}) "
                            f"err_yaw={err_yaw:.1f} err_xy={err_xy:.2f}m "
                            f"yaw_rate={math.degrees(float(getattr(self, '_yaw_rate', 0.0))):.0f}deg/s "
                            f"odom_dyaw={math.degrees(dyaw_odom):.1f}"
                        )
                        self._settle_buf = []
                        if not believable:
                            self._fuse_buf = []
                        return
                    step_yaw = max(-0.8, min(0.8, 0.45 * err_yaw))
                    scale = min(0.05, 0.45 * err_xy) / err_xy if err_xy > 1e-4 else 0.0
                    if abs(step_yaw) < 0.25 and err_xy < 0.08:
                        chosen = last
                        self._track_odom_note = (
                            f" fuse=hold err_yaw={err_yaw:.1f} err_xy={err_xy:.2f}m"
                        )
                    else:
                        corr_x = exp_x + (placed_odom[0] - exp_x) * scale
                        corr_y = exp_y + (placed_odom[1] - exp_y) * scale
                        corr_yaw = exp_yaw + step_yaw
"""

NEW_FUSE = f"""                    spinning = abs(float(getattr(self, "_yaw_rate", 0.0))) > 0.35
                    # {MARKER} — judge the repeated error, then step the published pose.
                    err_buf = list(getattr(self, "_fuse_err_buf", []))
                    err_buf.append((err_yaw, err_xy))
                    self._fuse_err_buf = err_buf[-3:]
                    err_stable = len(self._fuse_err_buf) >= 3 and all(
                        abs(item[0] - err_yaw) <= 4.0 and abs(item[1] - err_xy) <= 0.2
                        for item in self._fuse_err_buf
                    )
                    lock_yaw = err_yaw  # {MARKER_V2}
                    lock_xy = err_xy
                    recover = (
                        err_stable
                        and not spinning
                        and abs(lock_yaw) <= 90.0
                        and lock_xy <= 1.5
                        and (abs(lock_yaw) > 8.0 or lock_xy > 0.25)
                    )
                    believable = (abs(err_yaw) <= 18.0 and err_xy <= 1.2) or recover
                    if spinning or not (consistent or err_stable) or not believable:
                        why = "spin" if spinning else (
                            "unstable" if not (consistent or err_stable) else "far"
                        )
                        logger.info(
                            f"relocalize fuse: keep odom ({{why}}) "
                            f"err_yaw={{err_yaw:.1f}} err_xy={{err_xy:.2f}}m "
                            f"yaw_rate={{math.degrees(float(getattr(self, '_yaw_rate', 0.0))):.0f}}deg/s "
                            f"odom_dyaw={{math.degrees(dyaw_odom):.1f}}"
                        )
                        self._settle_buf = []
                        if not believable and not err_stable:
                            self._fuse_buf = []
                            self._fuse_err_buf = []
                        return
                    self._recover_step = recover
                    if recover:
                        step_yaw = max(-4.0, min(4.0, 0.5 * lock_yaw))
                        scale = min(0.08, 0.5 * lock_xy) / lock_xy if lock_xy > 1e-4 else 0.0
                        base_x, base_y, base_yaw = exp_x, exp_y, exp_yaw
                    else:
                        step_yaw = max(-0.8, min(0.8, 0.45 * err_yaw))
                        scale = min(0.05, 0.45 * err_xy) / err_xy if err_xy > 1e-4 else 0.0
                        base_x, base_y, base_yaw = exp_x, exp_y, exp_yaw
                    if abs(step_yaw) < 0.25 and (lock_xy if recover else err_xy) < 0.08:
                        chosen = last
                        self._track_odom_note = (
                            f" fuse=hold err_yaw={{err_yaw:.1f}} err_xy={{err_xy:.2f}}m"
                        )
                    else:
                        corr_x = base_x + (placed_odom[0] - base_x) * scale
                        corr_y = base_y + (placed_odom[1] - base_y) * scale
                        corr_yaw = base_yaw + step_yaw
"""

OLD_BOUND = "                if abs(ayaw) > 25.0 or adist > 2.0:\n"
NEW_BOUND = (
    "                if getattr(self, \"_recover_step\", False):\n"
    "                    max_step_xy = max(max_step_xy, 0.12)\n"
    "                    max_step_yaw = max(max_step_yaw, 5.0)\n"
    "                if abs(ayaw) > 90.0 or adist > 4.0:\n"
)


def main() -> None:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    path = site / "dimos/mapping/relocalization/lidar/module.py"
    text = path.read_text(encoding="utf-8")
    if MARKER_V3 in text:
        print(f"already patched: {path.name} {MARKER_V3}")
        return
    if MARKER_V2 in text:
        if text.count(PRED_OLD) != 1:
            raise SystemExit(f"{path.name}: prediction anchor found {text.count(PRED_OLD)} times")
        path.write_text(text.replace(PRED_OLD, PRED_NEW, 1), encoding="utf-8")
        print(f"patched {path.name} -> {MARKER_V3}")
        return
    if MARKER in text:
        for old, new in V1_TO_V2 + [(JUMP_OLD, JUMP_NEW)]:
            if text.count(old) != 1:
                raise SystemExit(f"{path.name}: V2 anchor found {text.count(old)} times: {old[:50]!r}")
            text = text.replace(old, new, 1)
        if text.count(PRED_OLD) != 1:
            raise SystemExit(f"{path.name}: prediction anchor found {text.count(PRED_OLD)} times")
        text = text.replace(PRED_OLD, PRED_NEW, 1)
        path.write_text(text, encoding="utf-8")
        print(f"patched {path.name} -> {MARKER_V3}")
        return
    if text.count(OLD_FUSE) != 1:
        raise SystemExit(f"{path.name}: fuse anchor found {text.count(OLD_FUSE)} times")
    if text.count(OLD_BOUND) != 1:
        raise SystemExit(f"{path.name}: bound anchor found {text.count(OLD_BOUND)} times")
    # The step limits are assigned just before the from-lock check.
    text = text.replace(OLD_FUSE, NEW_FUSE, 1)
    # NEW_BOUND uses max_step_*, which exist only after they are assigned.
    assigned = "            max_step_yaw = float(os.environ.get(\"DIMOS_RELOC_TRACK_STEP_YAW_DEG\", \"2.5\"))\n"
    if text.count(assigned) != 1:
        raise SystemExit(f"{path.name}: step assignment found {text.count(assigned)} times")
    text = text.replace(assigned, assigned + "\n", 1)
    text = text.replace(OLD_BOUND, NEW_BOUND, 1)
    if text.count(JUMP_OLD) != 1:
        raise SystemExit(f"{path.name}: jump check found {text.count(JUMP_OLD)} times")
    text = text.replace(JUMP_OLD, JUMP_NEW, 1)
    if text.count(PRED_OLD) != 1:
        raise SystemExit(f"{path.name}: prediction anchor found {text.count(PRED_OLD)} times")
    text = text.replace(PRED_OLD, PRED_NEW, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} -> {MARKER_V3}")


if __name__ == "__main__":
    main()
