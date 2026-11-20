#!/usr/bin/env python3
"""Write map-to-world.json when a reloc fix is accepted (needed by waypoint_patrol).

Upstream DimOS 0.0.15 RelocalizationModule.submit() only publishes TF; this repo's
start_patrol.sh refuses to dispatch goals without PATROL_TRANSFORM_FILE.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_TRANSFORM_FILE_PATCH_V1"
ATOMIC = "DIMOS_TRANSFORM_FILE_ATOMIC_V1"

OLD = '''    def submit(self, tf: Transform, source: str = "") -> None:
        """Publish a ``world_frame -> map`` fix the implementation already decided to believe."""
        world, map_frame = self.config.world_frame, self.config.map_frame
        assert (tf.frame_id, tf.child_frame_id) == (world, map_frame), (
            f"relocalize {source}: expected {world!r} -> {map_frame!r}, "
            f"got {tf.frame_id!r} -> {tf.child_frame_id!r}"
        )
        logger.info(
            "relocalize fix",
            source=source,
            world=world,
            map_frame=map_frame,
            translation=str(tf.translation),
            yaw_deg=round(yaw_deg(tf), 1),
        )
        self.fixes.on_next(tf)
'''

NEW = '''    def submit(self, tf: Transform, source: str = "") -> None:
        """Publish a ``world_frame -> map`` fix the implementation already decided to believe."""
        # DIMOS_TRANSFORM_FILE_PATCH_V1
        import json
        import os
        from pathlib import Path as _Path

        world, map_frame = self.config.world_frame, self.config.map_frame
        assert (tf.frame_id, tf.child_frame_id) == (world, map_frame), (
            f"relocalize {source}: expected {world!r} -> {map_frame!r}, "
            f"got {tf.frame_id!r} -> {tf.child_frame_id!r}"
        )
        logger.info(
            "relocalize fix",
            source=source,
            world=world,
            map_frame=map_frame,
            translation=str(tf.translation),
            yaw_deg=round(yaw_deg(tf), 1),
        )
        self.fixes.on_next(tf)
        out = os.environ.get("PATROL_TRANSFORM_FILE", "").strip()
        if out:
            try:
                t = tf.translation
                r = tf.rotation
                payload = {
                    "frame_id": world,
                    "child_frame_id": map_frame,
                    "translation": [float(t[0]), float(t[1]), float(t[2])],
                    "rotation_xyzw": [float(r.x), float(r.y), float(r.z), float(r.w)],
                    "yaw_deg": round(yaw_deg(tf), 3),
                    "source": source,
                }
                path = _Path(out)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(payload, indent=2) + "\\n", encoding="utf-8")
                logger.info(f"wrote map-to-world transform: {path}")
            except Exception:
                logger.exception(f"failed writing PATROL_TRANSFORM_FILE={out!r}")
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/mapping/relocalization/module.py"
    )
    if not target.is_file():
        print(f"module not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if ATOMIC in text:
        print(f"already patched: {target}")
        return 0
    torn = '''                path = _Path(out)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(payload, indent=2) + "\\n", encoding="utf-8")
                logger.info(f"wrote map-to-world transform: {path}")
'''
    whole = '''                path = _Path(out)
                path.parent.mkdir(parents=True, exist_ok=True)
                # DIMOS_TRANSFORM_FILE_ATOMIC_V1 — readers must not see a half-written pose.
                tmp = path.with_suffix(path.suffix + ".tmp")
                tmp.write_text(json.dumps(payload, indent=2) + "\\n", encoding="utf-8")
                os.replace(tmp, path)
                logger.info(f"wrote map-to-world transform: {path}")
'''
    if MARKER in text and torn in text:
        target.write_text(text.replace(torn, whole, 1), encoding="utf-8")
        print(f"patched {target} -> {ATOMIC}")
        return 0
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if OLD not in text:
        print("expected submit() block not found", file=sys.stderr)
        return 1
    backup = target.with_suffix(target.suffix + ".pre_transform_file")
    if not backup.exists():
        backup.write_text(text, encoding="utf-8")
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
