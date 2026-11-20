#!/usr/bin/env python3
"""Log the prior cloud once, as a static Rerun entity.

Repeating the full loaded_map on the timeline fills the gRPC buffer. Over the
phone hotspot the viewer then stays on the first frame.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RERUN_MAP_TF_V2"
OLD_MARKER = "DIMOS_RERUN_STATIC_MAP_V1"
OLD = """            rr.log(entity_path, cast("Archetype", rerun_data))
"""
NEW = """            # DIMOS_RERUN_MAP_TF_V2 — a static cloud ignores later yaw, so this one stays on the timeline.
            rr.log(entity_path, cast("Archetype", rerun_data))
"""


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/visualization/rerun/bridge.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    frozen = """            # DIMOS_RERUN_STATIC_MAP_V1 — prior cloud is static; TF moves it.
            rr.log(
                entity_path,
                cast("Archetype", rerun_data),
                static=entity_path == "world/loaded_map",
            )
"""
    moving = """            # DIMOS_RERUN_MAP_TF_V2 — a static cloud ignores later yaw, so this one stays on the timeline.
            rr.log(entity_path, cast("Archetype", rerun_data))
"""
    if frozen in text:
        target.write_text(text.replace(frozen, moving, 1), encoding="utf-8")
        print(f"patched {target} -> {MARKER}")
        return 0
    if OLD not in text:
        print("could not find rr.log for streamed entities", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
