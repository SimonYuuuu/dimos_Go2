#!/usr/bin/env python3
"""Keep the locked pose while the dog is standing still.

Small ICP updates were accepted one after another and the map slid even
though the dog had not moved. Hint gates are unchanged.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_HOLD_STILL_V1"
MARKER_V2 = "DIMOS_RELOC_HOLD_STILL_V2"
MARKER_V3 = "DIMOS_RELOC_HOLD_STILL_V3"

OLD = '''        if getattr(self, "_settle_anchor", None) is None:
            self._settle_anchor = chosen
        self._settle_last = chosen
        self.submit(chosen, "lidar")
'''

NEW = '''        # DIMOS_RELOC_HOLD_STILL_V1 — do not walk the lock while odometry is still.
        odom = getattr(self, "_latest_odom", None)
        prev = getattr(self, "_track_odom", None)
        if last is not None and odom is not None and prev is not None:
            try:
                ox, oy, oyaw = float(odom.x), float(odom.y), float(odom.yaw)
            except (AttributeError, TypeError, ValueError):
                ox = oy = oyaw = None
            if ox is not None:
                moved = math.hypot(ox - prev[0], oy - prev[1])
                dyaw_odom = (oyaw - prev[2] + math.pi) % (2 * math.pi) - math.pi
                if moved < 0.08 and abs(dyaw_odom) < math.radians(4):
                    # DIMOS_RELOC_HOLD_STILL_V3 — keep the lock while standing.
                    if not getattr(self, "_track_held", False):
                        logger.info("relocalize track: dog is still, keep the locked pose")
                        self._track_held = True
                    self._settle_buf = []
                    return
                self._track_held = False
        if odom is not None:
            try:
                self._track_odom = (float(odom.x), float(odom.y), float(odom.yaw))
            except (AttributeError, TypeError, ValueError):
                pass
        if getattr(self, "_settle_anchor", None) is None:
            self._settle_anchor = chosen
        self._settle_last = chosen
        self.submit(chosen, "lidar")
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/mapping/relocalization/lidar/module.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER_V3 in text:
        print(f"already patched: {target}")
        return 0
    if MARKER_V2 in text:
        start = text.find("                if moved < 0.08 and abs(dyaw_odom) < math.radians(4):\n")
        end = text.find("        if odom is not None:", start)
        if start < 0 or end < 0 or "DIMOS_RELOC_HOLD_STILL_V2" not in text[start:end]:
            print("standing correction block not found", file=sys.stderr)
            return 1
        simple = '''                if moved < 0.08 and abs(dyaw_odom) < math.radians(4):
                    # DIMOS_RELOC_HOLD_STILL_V3 — keep the lock while standing.
                    if not getattr(self, "_track_held", False):
                        logger.info("relocalize track: dog is still, keep the locked pose")
                        self._track_held = True
                    self._settle_buf = []
                    return
                self._track_held = False
'''
        target.write_text(text[:start] + simple + text[end:], encoding="utf-8")
        print(f"reverted standing correction: {target}")
        return 0
    old_still = '''                if moved < 0.08 and abs(dyaw_odom) < math.radians(4):
                    if not getattr(self, "_track_held", False):
                        logger.info("relocalize track: dog is still, keep the locked pose")
                        self._track_held = True
                    self._settle_buf = []
                    return
                self._track_held = False
'''
    new_still = NEW.split("                if moved < 0.08", 1)[1]
    new_still = "                if moved < 0.08" + new_still.split("        if odom is not None:", 1)[0]
    if MARKER in text:
        if old_still not in text:
            print("standing-hold block not found", file=sys.stderr)
            return 1
        target.write_text(text.replace(old_still, new_still, 1), encoding="utf-8")
        print(f"upgraded {target}")
        return 0
    if OLD not in text:
        print("submit block not found", file=sys.stderr)
        return 1
    if text.count(OLD) != 1:
        print("submit block is not unique", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
