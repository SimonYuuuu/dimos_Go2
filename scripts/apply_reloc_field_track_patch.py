#!/usr/bin/env python3
"""After the first lock, track the walls on a likelihood field instead of ICP.

At 0.5 m/s the ICP tracker fell behind: it moved the map by at most 1 degree per
update, threw away any match further than 0.30 m / 8 deg from its own pose, and so
could not come back once the Go2 odometry had jumped. By lap 3 the live cloud sat
6 to 15 degrees and 0.5 to 1 m off the loaded map.

Installs scripts/reloc_field_track.py as lidar/field_track.py and hooks it into
CloudRelocalization._relocalize. DIMOS_RELOC_FIELD_TRACK=0 returns to the ICP tracker.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_FIELD_TRACK_V1"

ANCHOR = '''        msg = self._cloud_in_world(msg)  # DIMOS_RELOC_WORLD_CLOUD_V1
        import math
        import os
'''

HOOK = '''
        # DIMOS_RELOC_FIELD_TRACK_V1 — after the first lock, slide the scan over the map's walls.
        if (
            getattr(self, "_settle_last", None) is not None
            and os.environ.get("DIMOS_RELOC_FIELD_TRACK", "1") != "0"
        ):
            try:
                from dimos.mapping.relocalization.lidar.field_track import track_step

                track_step(self, msg, logger)
            except Exception:
                if not getattr(self, "_field_err", False):
                    self._field_err = True
                    logger.exception("relocalize field tracking failed; odometry carries the pose")
            return
'''


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    site = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else repo / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    lidar = site / "dimos/mapping/relocalization/lidar"
    module = lidar / "module.py"
    source = Path(__file__).with_name("reloc_field_track.py")
    if not module.is_file() or not source.is_file():
        print(f"not found: {module if not module.is_file() else source}", file=sys.stderr)
        return 1

    wanted = source.read_text(encoding="utf-8")
    installed = lidar / "field_track.py"
    if not installed.is_file() or installed.read_text(encoding="utf-8") != wanted:
        installed.write_text(wanted, encoding="utf-8")
        print(f"installed {installed}")

    text = module.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {module}")
        return 0
    if text.count(ANCHOR) != 1:
        print("world-cloud line not found once in _relocalize", file=sys.stderr)
        return 1
    module.write_text(text.replace(ANCHOR, ANCHOR + HOOK, 1), encoding="utf-8")
    print(f"patched {module}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
