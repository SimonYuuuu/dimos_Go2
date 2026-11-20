#!/usr/bin/env python3
"""Fuse odometry and lidar for continuous localization.

Odometry predicts how the dog moved, including its measured yaw rate.
Lidar is allowed to correct that prediction by a small step when several
matches agree and the dog is not spinning. A single bad match cannot
rotate the map.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_TRACK_BOUND_V1"
ODOM_MARKER = "DIMOS_RELOC_TRACK_ODOM_V1"
FUSE_MARKER = "DIMOS_RELOC_FUSE_V1"

OLD = '''            dyaw, dist = fix_error(chosen, last)
            max_step_xy = float(os.environ.get("DIMOS_RELOC_TRACK_STEP_M", "0.6"))
            max_step_yaw = float(os.environ.get("DIMOS_RELOC_TRACK_STEP_YAW_DEG", "20"))
            if abs(dyaw) > max_step_yaw or dist > max_step_xy:
'''

ODOM_CHECK = '''            # DIMOS_RELOC_TRACK_ODOM_V1 — map pose follows the measured yaw rate.
            self._track_odom_note = ""
            odom_now = getattr(self, "_latest_odom", None)
            odom_prev = getattr(self, "_track_odom", None)
            prev_on_map = getattr(self, "_track_on_map", None)
            if odom_now is not None and odom_prev is not None and prev_on_map is not None:
                placed_odom = None
                try:
                    dx = float(odom_now.x) - odom_prev[0]
                    dy = float(odom_now.y) - odom_prev[1]
                    dyaw_odom = (
                        float(odom_now.yaw) - odom_prev[2] + math.pi
                    ) % (2 * math.pi) - math.pi
                    map_yaw = math.radians(yaw_deg(last))
                    c, s = math.cos(map_yaw), math.sin(map_yaw)
                    exp_x = prev_on_map[0] + c * dx + s * dy
                    exp_y = prev_on_map[1] - s * dx + c * dy
                    exp_yaw = prev_on_map[2] + math.degrees(dyaw_odom)
                    placed_odom = self._robot_on_map(chosen)
                except (AttributeError, TypeError, ValueError):
                    placed_odom = None
                if placed_odom is not None:
                    err_yaw = (placed_odom[2] - exp_yaw + 180.0) % 360.0 - 180.0
                    err_xy = math.hypot(placed_odom[0] - exp_x, placed_odom[1] - exp_y)
                    if abs(err_yaw) > 2.5 or err_xy > 0.15:
                        logger.info(
                            f"relocalize track: reject vs odom yaw={yaw_deg(chosen):.1f} "
                            f"odom_dyaw={math.degrees(dyaw_odom):.1f} "
                            f"map=({placed_odom[0]:.2f},{placed_odom[1]:.2f},{placed_odom[2]:.1f}) "
                            f"expected=({exp_x:.2f},{exp_y:.2f},{exp_yaw:.1f}) "
                            f"err_yaw={err_yaw:.1f} err_xy={err_xy:.2f}m"
                        )
                        self._settle_buf = []
                        return
                    self._track_odom_note = (
                        f" odom_dyaw={math.degrees(dyaw_odom):.1f} "
                        f"map=({placed_odom[0]:.2f},{placed_odom[1]:.2f},{placed_odom[2]:.1f})"
                    )
'''

NEW = '''            dyaw, dist = fix_error(chosen, last)
''' + ODOM_CHECK + '''            max_step_xy = float(os.environ.get("DIMOS_RELOC_TRACK_STEP_M", "0.15"))
            max_step_yaw = float(os.environ.get("DIMOS_RELOC_TRACK_STEP_YAW_DEG", "2.5"))
            # DIMOS_RELOC_TRACK_BOUND_V1 — walking or spinning must not rotate the map.
            anchor = getattr(self, "_settle_anchor", None)
            if anchor is not None:
                ayaw, adist = fix_error(chosen, anchor)
                odom_now = getattr(self, "_latest_odom", None)
                odom_prev = getattr(self, "_track_odom", None)
                turning = False
                if odom_now is not None and odom_prev is not None:
                    try:
                        dyaw_odom = (
                            float(odom_now.yaw) - odom_prev[2] + math.pi
                        ) % (2 * math.pi) - math.pi
                        turning = abs(dyaw_odom) > math.radians(5)
                    except (AttributeError, TypeError, ValueError):
                        turning = False
                if turning:
                    max_step_yaw = min(max_step_yaw, 1.0)
                if abs(ayaw) > 6.0 or adist > 0.45:
                    logger.info(
                        f"relocalize track: reject drift yaw={yaw_deg(chosen):.1f} "
                        f"from_lock_dyaw={ayaw:.1f} from_lock_dist={adist:.2f}m "
                        "(max 0.45m / 6deg from the first lock)"
                    )
                    self._settle_buf = []
                    return
            if abs(dyaw) > max_step_yaw or dist > max_step_xy:
'''

TRACK_LOG_OLD = '''            logger.info(
                f"relocalize track update yaw={yaw_deg(chosen):.1f} "
                f"dyaw={dyaw:.1f} dist={dist:.2f}m t={chosen.translation}"
            )
'''

TRACK_LOG_NEW = '''            logger.info(
                f"relocalize track update yaw={yaw_deg(chosen):.1f} "
                f"dyaw={dyaw:.1f} dist={dist:.2f}m t={chosen.translation}"
                + getattr(self, "_track_odom_note", "")
            )
'''

ODOM_SAVE_OLD = '''                self._track_odom = (float(odom.x), float(odom.y), float(odom.yaw))
'''

ODOM_SAVE_NEW = '''                self._track_odom = (float(odom.x), float(odom.y), float(odom.yaw))
                on_map = self._robot_on_map(chosen)
                if on_map is not None:
                    self._track_on_map = on_map
'''


def _default_target() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/mapping/relocalization/lidar/module.py"
    )


def _insert_odom(text: str) -> str:
    if ODOM_MARKER not in text:
        anchor = "            dyaw, dist = fix_error(chosen, last)\n"
        at = text.find(anchor)
        if at < 0:
            raise SystemExit("track step line not found")
        at += len(anchor)
        text = text[:at] + ODOM_CHECK + text[at:]
    if "+ getattr(self, \"_track_odom_note\"" not in text:
        if TRACK_LOG_OLD not in text:
            raise SystemExit("track update log not found")
        text = text.replace(TRACK_LOG_OLD, TRACK_LOG_NEW, 1)
    if "_track_on_map = on_map" not in text:
        if ODOM_SAVE_OLD not in text:
            raise SystemExit("track odom save not found")
        text = text.replace(ODOM_SAVE_OLD, ODOM_SAVE_NEW, 1)
    return text


FUSE_BLOCK = '''            # DIMOS_RELOC_FUSE_V1 — odom predicts the motion, lidar applies a small correction.
            self._fusion_correcting = False
            self._track_odom_note = ""
            odom_now = getattr(self, "_latest_odom", None)
            odom_prev = getattr(self, "_track_odom", None)
            prev_on_map = getattr(self, "_track_on_map", None)
            if odom_now is not None and odom_prev is not None and prev_on_map is not None:
                placed_odom = None
                try:
                    dx = float(odom_now.x) - odom_prev[0]
                    dy = float(odom_now.y) - odom_prev[1]
                    dyaw_odom = (
                        float(odom_now.yaw) - odom_prev[2] + math.pi
                    ) % (2 * math.pi) - math.pi
                    map_yaw = math.radians(yaw_deg(last))
                    c, s = math.cos(map_yaw), math.sin(map_yaw)
                    exp_x = prev_on_map[0] + c * dx + s * dy
                    exp_y = prev_on_map[1] - s * dx + c * dy
                    exp_yaw = prev_on_map[2] + math.degrees(dyaw_odom)
                    placed_odom = self._robot_on_map(chosen)
                except (AttributeError, TypeError, ValueError):
                    placed_odom = None
                if placed_odom is not None:
                    err_yaw = (placed_odom[2] - exp_yaw + 180.0) % 360.0 - 180.0
                    err_xy = math.hypot(placed_odom[0] - exp_x, placed_odom[1] - exp_y)
                    buf = list(getattr(self, "_fuse_buf", []))
                    buf.append(placed_odom)
                    self._fuse_buf = buf[-3:]
                    consistent = len(self._fuse_buf) >= 3
                    if consistent:
                        for sample in self._fuse_buf:
                            span_yaw = (sample[2] - placed_odom[2] + 180.0) % 360.0 - 180.0
                            span_xy = math.hypot(
                                sample[0] - placed_odom[0], sample[1] - placed_odom[1]
                            )
                            if abs(span_yaw) > 3.0 or span_xy > 0.25:
                                consistent = False
                                break
                    spinning = abs(float(getattr(self, "_yaw_rate", 0.0))) > 0.35
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
                        try:
                            import numpy as np
                            from dimos.msgs.geometry_msgs.Transform import Transform

                            world_base = self._pose_matrix(
                                float(odom_now.x), float(odom_now.y), float(odom_now.yaw)
                            )
                            map_base = self._pose_matrix(
                                corr_x, corr_y, math.radians(corr_yaw)
                            )
                            fused = Transform.from_matrix(
                                world_base @ np.linalg.inv(map_base),
                                frame_id=self.config.world_frame,
                                child_frame_id=self.config.map_frame,
                            )
                        except (AttributeError, TypeError, ValueError):
                            self._settle_buf = []
                            return
                        chosen = fused
                        self._fusion_correcting = True
                        self._track_odom_note = (
                            f" fuse=step dyaw={step_yaw:.1f} dxy={err_xy * scale:.2f}m "
                            f"err_yaw={err_yaw:.1f} err_xy={err_xy:.2f}m"
                        )
'''

ODOM_ON = '''        import time

        self._latest_odom = msg
        now = time.monotonic()
'''

FUSE_ON = '''        import math
        import time

        self._latest_odom = msg
        now = time.monotonic()
        # DIMOS_RELOC_FUSE_V1 — yaw rate, so a spin is not treated as map drift.
        try:
            yaw_now = float(msg.yaw)
            tick = getattr(self, "_odom_tick", None)
            if tick is not None:
                dt = now - tick[0]
                if dt >= 0.02:
                    dyaw_tick = (yaw_now - tick[1] + math.pi) % (2 * math.pi) - math.pi
                    self._yaw_rate = dyaw_tick / dt
            self._odom_tick = (now, yaw_now)
        except (AttributeError, TypeError, ValueError):
            pass
'''

HOLD_OLD = '''                if moved < 0.08 and abs(dyaw_odom) < math.radians(4):
                    # DIMOS_RELOC_HOLD_STILL_V3 — keep the lock while standing.
'''

HOLD_NEW = '''                if (
                    moved < 0.08
                    and abs(dyaw_odom) < math.radians(4)
                    and not getattr(self, "_fusion_correcting", False)
                ):
                    # DIMOS_RELOC_HOLD_STILL_V3 — keep the lock while standing.
'''


def _apply_fuse(text: str) -> str:
    if FUSE_MARKER not in text:
        if ODOM_CHECK not in text:
            raise SystemExit("odom check block not found")
        text = text.replace(ODOM_CHECK, FUSE_BLOCK, 1)
        text = text.replace(
            "if abs(ayaw) > 6.0 or adist > 0.45:",
            "if abs(ayaw) > 25.0 or adist > 2.0:",
            1,
        )
        text = text.replace(
            '"(max 0.45m / 6deg from the first lock)"',
            '"(max 2.0m / 25deg from the first lock)"',
            1,
        )
    if "self._yaw_rate" not in text:
        if ODOM_ON not in text:
            raise SystemExit("odom handler not found")
        text = text.replace(ODOM_ON, FUSE_ON, 1)
    if "_fusion_correcting" not in text.split("moved < 0.08", 1)[-1][:400]:
        if HOLD_OLD not in text:
            raise SystemExit("hold-still block not found")
        text = text.replace(HOLD_OLD, HOLD_NEW, 1)
    return text


def main() -> int:
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else _default_target()
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if (
        FUSE_MARKER in text
        and "self._yaw_rate" in text
        and "not getattr(self, \"_fusion_correcting\", False)" in text
    ):
        print(f"already patched: {target}")
        return 0
    if MARKER not in text and ODOM_MARKER not in text and FUSE_MARKER not in text:
        if OLD not in text:
            print("track step block not found", file=sys.stderr)
            return 1
        text = text.replace(OLD, NEW, 1)
    text = _insert_odom(text)
    text = _apply_fuse(text)
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
