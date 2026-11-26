#!/usr/bin/env python3
"""Keep Rerun live for a whole patrol, and draw the trail on the loaded map.

Every tf, scan, path and costmap was logged on the timeline, so the viewer kept
all of it and slowed down loop after loop. With DIMOS_RERUN_LATEST_ONLY=1
(default) they are logged static: the viewer holds only the latest of each.

The trail used odometry in the world frame. When localization moved the map,
old points stayed where odometry put them and no longer matched the costmap.
It now uses the dog pose on the loaded map (PATROL_DOG_POSE_FILE) and hangs
under the map frame, so it moves with the map.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RERUN_LATEST_ONLY_V1"
MARKER_V2 = "DIMOS_RERUN_LATEST_ONLY_V2"
TF_OLD = "                    rr.log(path, archetype, static=_rerun_latest_only())\n"
TF_NEW = (
    "                    # DIMOS_RERUN_LATEST_ONLY_V2 — frames stay on the clock so a nudge\n"
    "                    # moves the view immediately. Clouds and maps stay static.\n"
    "                    rr.log(path, archetype, static=False)\n"
)

HELPERS = '''

def _rerun_latest_only() -> bool:
    # DIMOS_RERUN_LATEST_ONLY_V1 — no history in the viewer; it stays fast for hours.
    return os.environ.get("DIMOS_RERUN_LATEST_ONLY", "1").lower() not in ("0", "false", "no")


def _map_trail_point(odom):
    """Dog on the loaded map from the localization pose file, or None if stale."""
    import json
    import time as _time

    path = os.environ.get("PATROL_DOG_POSE_FILE", "").strip()
    if not path:
        return None
    try:
        with open(path, encoding="utf-8") as f:
            pose = json.load(f)
        if _time.time() - float(pose.get("t", 0.0)) > 2.0:
            return None
        return (float(pose["x"]), float(pose["y"]), float(getattr(odom, "z", 0.0)) + 0.05)
    except (OSError, ValueError, TypeError, KeyError):
        return None
'''

PAIRS = [
    (
        '                _p = (float(msg.x), float(msg.y), float(getattr(msg, "z", 0.0)) + 0.05)\n',
        "                _p = _map_trail_point(msg)\n"
        "                if _p is None:\n"
        '                    raise LookupError("no dog pose on the map yet")\n',
    ),
    (
        '                    rr.log("world/trail", rr.Transform3D(parent_frame="tf#/world"), static=True)\n',
        '                    rr.log("world/trail", rr.Transform3D(parent_frame="tf#/map"), static=True)\n',
    ),
    (
        "                for path, archetype in msg.to_rerun(self._tf_tree):\n"
        "                    rr.log(path, archetype)\n",
        "                for path, archetype in msg.to_rerun(self._tf_tree):\n" + TF_NEW,
    ),
    (
        "                rr.log(entry[0], entry[1], static=isinstance(entry, RerunEntry) and entry.static)\n",
        "                rr.log(\n"
        "                    entry[0],\n"
        "                    entry[1],\n"
        "                    static=_rerun_latest_only()\n"
        "                    or (isinstance(entry, RerunEntry) and entry.static),\n"
        "                )\n",
    ),
    (
        '                rr.log(entity_path, rr.Transform3D(parent_frame=f"tf#/{frame_id}"))\n',
        "                rr.log(\n"
        "                    entity_path,\n"
        '                    rr.Transform3D(parent_frame=f"tf#/{frame_id}"),\n'
        "                    static=_rerun_latest_only(),\n"
        "                )\n",
    ),
    (
        '            rr.log(entity_path, cast("Archetype", rerun_data), static=one_map)\n',
        '            rr.log(\n'
        '                entity_path,\n'
        '                cast("Archetype", rerun_data),\n'
        '                static=one_map or _rerun_latest_only(),\n'
        '            )\n',
    ),
    (
        "                        rr.Transform3D(parent_frame=f\"tf#/{frame_id}\"),\n"
        "                        static=one_map,\n",
        "                        rr.Transform3D(parent_frame=f\"tf#/{frame_id}\"),\n"
        "                        static=one_map or _rerun_latest_only(),\n",
    ),
]

CLASS_ANCHOR = "\ndef region_entity(base: str, seq: int) -> str:\n"


def main() -> int:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/visualization/rerun/bridge.py"
    )
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        if MARKER_V2 in text:
            print(f"already patched: {target}")
            return 0
        if text.count(TF_OLD) != 1:
            print(f"tf anchor found {text.count(TF_OLD)} times", file=sys.stderr)
            return 1
        target.write_text(text.replace(TF_OLD, TF_NEW, 1), encoding="utf-8")
        print(f"patched {target} -> {MARKER_V2}")
        return 0
    for old, new in PAIRS:
        count = text.count(old)
        if count != 1:
            print(f"anchor found {count} times: {old[:70]!r}", file=sys.stderr)
            return 1
        text = text.replace(old, new, 1)
    if text.count(CLASS_ANCHOR) != 1:
        print("helper anchor not found", file=sys.stderr)
        return 1
    text = text.replace(CLASS_ANCHOR, HELPERS + CLASS_ANCHOR, 1)
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
