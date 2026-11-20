#!/usr/bin/env python3
"""Keep PGO from publishing world←map while lidar relocalization owns that edge.

Both write the same TF. The later one rotates the prior cloud away from the live scan.
DIMOS_PGO_PUBLISH_TF=0 skips the publish. The pose graph still builds the costmap.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_PGO_TF_GUARD_V1"
OLD = """            placement = self._map.pgo.world_to_map()
            if placement is not None:
                self.tf.publish(TFMessage(placement.now()))
"""
NEW = """            placement = self._map.pgo.world_to_map()
            # DIMOS_PGO_TF_GUARD_V1 — lidar relocalization owns world←map.
            if placement is not None and __import__("os").environ.get("DIMOS_PGO_PUBLISH_TF", "1") != "0":
                self.tf.publish(TFMessage(placement.now()))
"""


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/loop_closure/module.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if OLD not in text:
        print("could not find PGO tf.publish", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
