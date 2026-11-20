#!/usr/bin/env python3
"""Prefer a wider path before falling back to a door-width squeeze.

The installed planner only tried 1.1x robot width, so every path hugged
walls. A few tens of centimetres of pose error then clips a door frame.
Try a comfortable clearance first; keep 1.1x for genuine narrow gaps.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_WIDE_PATH_V2"
OLD = """        #        sizes_to_try: list[float] = [2.2, 1.7, 1.3, 1]
        sizes_to_try: list[float] = [1.1]
"""
NEW = """        # DIMOS_NAV_WIDE_PATH_V2 — a little clearance, then the narrow gap.
        # Fatter widths made A* detour through the cost valley.
        sizes_to_try: list[float] = [1.2, 1.1]
"""
V1 = "sizes_to_try: list[float] = [1.5, 1.25, 1.1]"
V2 = "sizes_to_try: list[float] = [1.2, 1.1]  # DIMOS_NAV_WIDE_PATH_V2"


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star/global_planner.py"
    )
    text = target.read_text(encoding="utf-8")
    if MARKER in text or V2 in text:
        print(f"already patched: {target}")
        return 0
    if V1 in text:
        target.write_text(text.replace(V1, V2, 1), encoding="utf-8")
        print(f"patched {target} -> shorter clearance")
        return 0
    if OLD not in text:
        print("block not found", file=sys.stderr)
        return 1
    backup = target.with_suffix(target.suffix + ".pre_wide_path")
    if not backup.exists():
        backup.write_text(text, encoding="utf-8")
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
