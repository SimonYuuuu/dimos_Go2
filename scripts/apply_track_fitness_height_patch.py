#!/usr/bin/env python3
"""Keep tracking corrections while walking, and ignore points above the body.

Walking scans score about 0.58-0.61 against the loaded map while a standing
scan scores about 0.75, so the 0.72 gate dropped nearly every in-motion
correction and odometry drift built up. Tracking now uses
DIMOS_RELOC_TRACK_FITNESS (default 0.55); the first lock keeps the config gate.

Live obstacle bands topped out at 1.0 m above the floor, well over the Go2's
body, so tables and door frames stopped the dog. The top is
DIMOS_OBSTACLE_MAX_Z (default 0.6 m).
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_TRACK_FITNESS_HEIGHT_V1"


def edit(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path.name} {MARKER}")
        return
    if text.count(old) != 1:
        raise SystemExit(f"{path.name}: anchor found {text.count(old)} times: {old[:60]!r}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")
    print(f"patched {path.name} -> {MARKER}")


def main() -> None:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    edit(
        site / "dimos/mapping/relocalization/lidar/relocalize.py",
        "        if result.fitness < self.config.fitness_threshold:\n"
        "            return RelocAttempt(None, result)\n",
        f"        import os  # {MARKER}\n"
        "\n"
        "        gate = self.config.fitness_threshold\n"
        '        if os.environ.get("DIMOS_RELOC_SEED_POSE", "").strip():\n'
        '            gate = min(gate, float(os.environ.get("DIMOS_RELOC_TRACK_FITNESS", "0.55")))\n'
        "        if result.fitness < gate:\n"
        "            return RelocAttempt(None, result)\n",
    )
    top = '__import__("os").environ.get("DIMOS_OBSTACLE_MAX_Z", "0.6")'
    for rel in (
        "dimos/navigation/go2/replanning_a_star/local_planner.py",
        "dimos/navigation/go2/replanning_a_star/navigation_map.py",
    ):
        path = site / rel
        text = path.read_text(encoding="utf-8")
        if MARKER in text:
            print(f"already patched: {path.name} {MARKER}")
            continue
        if text.count("ground + 1.0") != 1:
            raise SystemExit(f"{path.name}: band anchor found {text.count('ground + 1.0')} times")
        text = text.replace("ground + 1.0", f"ground + float({top})", 1)
        path.write_text(f"# {MARKER}\n" + text, encoding="utf-8")
        print(f"patched {path.name} -> {MARKER}")


if __name__ == "__main__":
    main()
