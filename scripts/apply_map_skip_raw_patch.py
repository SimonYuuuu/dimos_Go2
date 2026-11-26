#!/usr/bin/env python3
"""`dimos map global`: skip the un-corrected "raw" map when DIMOS_MAP_SKIP_RAW=1.

The raw map is only drawn into the .rrd preview. The .pc2.lcm and .map.db
exports never read it, so skipping it saves its rebuild and denoise.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_MAP_SKIP_RAW_V1"

OLD = """    # Raw map: same dedup'd frames, no PGO correction.
    with progress(n_kept, "reconstructing global map") as bar:
        global_map = _accumulate(
            kept,
            voxel=voxel,
            block_count=block_count,
            device=device,
            register=register,
            carve_columns=carve,
            progress_cb=bar,
        )
"""
NEW = f"""    # Raw map: same dedup'd frames, no PGO correction.  # {MARKER}
    import os

    global_map = None
    if os.environ.get("DIMOS_MAP_SKIP_RAW") != "1":
        with progress(n_kept, "reconstructing global map") as bar:
            global_map = _accumulate(
                kept,
                voxel=voxel,
                block_count=block_count,
                device=device,
                register=register,
                carve_columns=carve,
                progress_cb=bar,
            )
"""


def main() -> int:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    path = site / "dimos/mapping/cli/map.py"
    if not path.is_file():
        print(f"not found: {path}", file=sys.stderr)
        return 1
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path.name} {MARKER}")
        return 0
    if text.count(OLD) != 1:
        print(f"{path}: raw-map block found {text.count(OLD)} times", file=sys.stderr)
        return 1
    path.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {path} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
