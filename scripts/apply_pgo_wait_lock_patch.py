#!/usr/bin/env python3
"""Hold global_map until lidar localization has accepted a pose.

PGO otherwise attaches the prior map on its own match and publishes it in that
frame. The walls then show up rotated away from the cloud the lidar lock is using.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_PGO_WAIT_LOCK_V1"

OLD_START = """            self._reloc_thread = threading.Thread(
                target=self._relocalize_loop, name="pgo-relocalize", daemon=True
            )
            self._reloc_thread.start()
            logger.info(f"PGO premap loaded: map_file={cfg.map_file!r} keyframes={len(premap)}")
"""

NEW_START = """            if __import__("os").environ.get("DIMOS_PGO_WAIT_FOR_LIDAR", "0") == "1":
                # DIMOS_PGO_WAIT_LOCK_V1 — lidar lock places the premap.
                logger.info(
                    f"PGO premap staged: map_file={cfg.map_file!r} keyframes={len(premap)}; "
                    "global_map waits for lidar localization"
                )
            else:
                self._reloc_thread = threading.Thread(
                    target=self._relocalize_loop, name="pgo-relocalize", daemon=True
                )
                self._reloc_thread.start()
                logger.info(f"PGO premap loaded: map_file={cfg.map_file!r} keyframes={len(premap)}")
"""

OLD_LOOP = """        attached = self._map.pgo.attached
        while not self._fixes.empty():
            fix, fitness, current = self._fixes.get()
            self._map.pgo.relocalize(fix, fitness, current)
"""

NEW_LOOP = """        attached = self._map.pgo.attached
        # DIMOS_PGO_WAIT_LOCK_V1 — do not attach or publish from PGO's own match.
        wait_lock = __import__("os").environ.get("DIMOS_PGO_WAIT_FOR_LIDAR", "0") == "1"
        if wait_lock:
            while not self._fixes.empty():
                self._fixes.get()
            lock = self._lidar_lock_transform()
            if lock is not None and self._matcher is not None and not self._map.pgo.attached:
                self._map.pgo.relocalize(lock, 1.0, None)
        else:
            while not self._fixes.empty():
                fix, fitness, current = self._fixes.get()
                self._map.pgo.relocalize(fix, fitness, current)
"""

OLD_EMIT = """        emit = (
            rebuilt
            or self._map.pgo.attached != attached
            or self._frames % self.config.emit_every == 0
        )
        global_map = self._map.global_map() if emit else None
"""

NEW_EMIT = """        emit = (
            rebuilt
            or self._map.pgo.attached != attached
            or self._frames % self.config.emit_every == 0
        )
        if wait_lock and not self._global_map_ready():
            emit = False
        global_map = self._map.global_map() if emit else None
"""

HELPER = '''
    def _lidar_lock_transform(self):
        # DIMOS_PGO_WAIT_LOCK_V1 — the pose lidar relocalization already accepted.
        import json
        import os

        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        path = os.environ.get("PATROL_TRANSFORM_FILE", "").strip()
        if not path:
            return None
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return None
        cached = getattr(self, "_lock_tf", None)
        if getattr(self, "_lock_mtime", None) == mtime and cached is not None:
            return cached.now()
        try:
            payload = json.loads(open(path, encoding="utf-8").read())
            t = payload["translation"]
            r = payload["rotation_xyzw"]
            tf = Transform(
                translation=Vector3(float(t[0]), float(t[1]), float(t[2])),
                rotation=Quaternion(float(r[0]), float(r[1]), float(r[2]), float(r[3])),
                frame_id=str(payload.get("frame_id", "world")),
                child_frame_id=str(payload.get("child_frame_id", "map")),
            )
        except Exception:
            logger.exception("PGO global_map: could not read the lidar lock")
            return None
        self._lock_mtime = mtime
        self._lock_tf = tf
        return tf.now()

    def _global_map_ready(self) -> bool:
        # DIMOS_PGO_WAIT_LOCK_V1
        if self._lidar_lock_transform() is None:
            if not getattr(self, "_hold_logged", False):
                self._hold_logged = True
                logger.info("PGO global_map: held until lidar localization succeeds")
            return False
        if self._matcher is not None and self._map is not None and not self._map.pgo.attached:
            return False
        if not getattr(self, "_release_logged", False):
            self._release_logged = True
            logger.info("PGO global_map: lidar lock accepted, publishing")
        return True

'''


LOOP_OLD = '''    def _search_premap(self) -> bool:
        """Close a loop from the newest keyframe into the nearest premap keyframe. True if one landed."""
        if not self._attached or self._premap is None:
            return False
'''
LOOP_NEW = '''    def _search_premap(self) -> bool:
        """Close a loop from the newest keyframe into the nearest premap keyframe. True if one landed."""
        # DIMOS_PGO_NO_PREMAP_LOOP_V1 — lidar owns the map pose. These closures pull the walls off it.
        if __import__("os").environ.get("DIMOS_PGO_WAIT_FOR_LIDAR", "0") == "1":
            return False
        if not self._attached or self._premap is None:
            return False
'''

FRAME_OLD = """        if global_map is not None:
            self.global_map.publish(global_map)
"""
FRAME_NEW = """        if global_map is not None:
            if wait_lock:
                global_map = self._global_map_on_lidar_pose(global_map)
            self.global_map.publish(global_map)
"""

FRAME_HELPER = '''
    def _global_map_on_lidar_pose(self, cloud):
        # DIMOS_PGO_LIDAR_FRAME_V1 — draw the prior with the same world←map as loaded_map.
        lock = self._lidar_lock_transform()
        if lock is None or self._map is None or not self._map.pgo.attached or len(cloud) == 0:
            # A missing lock used to publish the other frame, which left a second map on screen.
            return None
        from dimos.msgs.geometry_msgs.Transform import Transform

        into_map = Transform.from_matrix(
            self._map.pgo.graph_from_world(),
            frame_id="map",
            child_frame_id="world",
        )
        return cloud.transform(into_map).transform(lock)

'''


def _upgrade(target: Path, text: str) -> int:
    if "DIMOS_PGO_DRAW_ONCE_V1" in text:
        moved = """            # DIMOS_PGO_DRAW_ONCE_V1 — do not move the loaded map. The lock places it once.
"""
        restored_move = """            lock = self._lidar_lock_transform()
            if lock is not None and self._matcher is not None and not self._map.pgo.attached:
                self._map.pgo.relocalize(lock, 1.0, None)
"""
        once_draw = """    def _global_map_on_lidar_pose(self, cloud):
        # DIMOS_PGO_DRAW_ONCE_V1 — cloud is in the loaded map frame. Apply world←map once.
        lock = self._lidar_lock_transform()
        if lock is None or len(cloud) == 0:
            return None
        return cloud.transform(lock)
"""
        restored_draw = """    def _global_map_on_lidar_pose(self, cloud):
        # DIMOS_PGO_LIDAR_FRAME_V1 — draw the prior with the same world←map as loaded_map.
        lock = self._lidar_lock_transform()
        if lock is None or self._map is None or not self._map.pgo.attached or len(cloud) == 0:
            # DIMOS_PGO_LOCK_KEEP_V1 — never publish the map in the other frame.
            return None
        from dimos.msgs.geometry_msgs.Transform import Transform

        into_map = Transform.from_matrix(
            self._map.pgo.graph_from_world(),
            frame_id="map",
            child_frame_id="world",
        )
        return cloud.transform(into_map).transform(lock)
"""
        ready = """            return False
        if not getattr(self, "_release_logged", False):
"""
        ready_back = """            return False
        if self._matcher is not None and self._map is not None and not self._map.pgo.attached:
            return False
        if not getattr(self, "_release_logged", False):
"""
        if moved not in text or once_draw not in text or ready not in text:
            print("could not restore the previous map draw", file=sys.stderr)
            return 1
        text = (
            text.replace(moved, restored_move, 1)
            .replace(once_draw, restored_draw, 1)
            .replace(ready, ready_back, 1)
        )
        target.write_text(text, encoding="utf-8")
        print(f"restored {target} -> previous lidar map draw")
        return 0
    changed = False
    pgo = target.parent / "pgo.py"
    pgo_text = pgo.read_text(encoding="utf-8") if pgo.is_file() else ""
    if "DIMOS_PGO_NO_PREMAP_LOOP_V1" not in pgo_text:
        if LOOP_OLD not in pgo_text:
            print("could not find _search_premap", file=sys.stderr)
            return 1
        pgo.write_text(pgo_text.replace(LOOP_OLD, LOOP_NEW, 1), encoding="utf-8")
        changed = True
    if "DIMOS_PGO_LIDAR_FRAME_V1" not in text:
        if FRAME_OLD not in text:
            print("could not find global_map.publish", file=sys.stderr)
            return 1
        text = text.replace(FRAME_OLD, FRAME_NEW, 1)
        text = text.replace(
            "    def _process(self, cloud: PointCloud2) -> None:\n",
            FRAME_HELPER + "    def _process(self, cloud: PointCloud2) -> None:\n",
            1,
        )
        target.write_text(text, encoding="utf-8")
        changed = True
    stale = '''        except Exception:
            logger.exception("PGO global_map: could not read the lidar lock")
            return None
'''
    kept = '''        except Exception:
            logger.exception("PGO global_map: could not read the lidar lock")
            # DIMOS_PGO_LOCK_KEEP_V1 — a half-written file must not drop the last good pose.
            return cached.now() if cached is not None else None
'''
    raw = '''        if lock is None or self._map is None or not self._map.pgo.attached or len(cloud) == 0:
            return cloud
'''
    held = '''        if lock is None or self._map is None or not self._map.pgo.attached or len(cloud) == 0:
            # DIMOS_PGO_LOCK_KEEP_V1 — never publish the map in the other frame.
            return None
'''
    publish_old = '''            if wait_lock:
                global_map = self._global_map_on_lidar_pose(global_map)
            self.global_map.publish(global_map)
'''
    publish_new = '''            if wait_lock:
                global_map = self._global_map_on_lidar_pose(global_map)
            if global_map is not None:
                self.global_map.publish(global_map)
'''
    publish_gated = '''            if wait_lock:
                global_map = self._global_map_on_lidar_pose(global_map)
            if global_map is not None and self._global_map_due():
                self.global_map.publish(global_map)
'''
    due_method = '''
    def _global_map_due(self) -> bool:
        # DIMOS_GLOBAL_MAP_REFRESH_V1 — redraw the prior when the lock moves onto the scan.
        import math

        lock = self._lidar_lock_transform()
        sent = int(getattr(self, "_global_map_sent", 0))
        prev = getattr(self, "_global_map_lock", None)
        xy = None
        if lock is not None:
            try:
                translation = lock.translation
                xy = (float(translation[0]), float(translation[1]))
            except Exception:
                xy = None
        if sent < 2:
            self._global_map_sent = sent + 1
            self._global_map_lock = xy
            return True
        if xy is None or prev is None:
            return False
        if math.hypot(xy[0] - prev[0], xy[1] - prev[1]) < 0.25:
            return False
        self._global_map_lock = xy
        self._global_map_sent = sent + 1
        logger.info(
            "PGO global_map: lock moved %.2fm, publishing"
            % math.hypot(xy[0] - prev[0], xy[1] - prev[1])
        )
        return True

'''
    publish_open = '''            if wait_lock:
                global_map = self._global_map_on_lidar_pose(global_map)
            if global_map is not None:
                self.global_map.publish(global_map)
'''
    refresh_v1 = '''    def _global_map_due(self) -> bool:
        # DIMOS_GLOBAL_MAP_REFRESH_V1 — redraw the prior when the lock moves onto the scan.
'''
    refresh_v2 = '''    def _global_map_due(self) -> bool:
        # DIMOS_GLOBAL_MAP_REFRESH_V2 — keep a slow copy coming so the costmap stays available.
        # Rerun still stops after a handful of snapshots.
        import time

        now = time.monotonic()
        last = float(getattr(self, "_global_map_at", 0.0))
        if now - last < 1.0:
            return False
        self._global_map_at = now
        return True

    def _global_map_due_v1_unused(self) -> bool:
        # DIMOS_GLOBAL_MAP_REFRESH_V1 — redraw the prior when the lock moves onto the scan.
'''
    if "DIMOS_GLOBAL_MAP_REFRESH_V1" in text and "DIMOS_GLOBAL_MAP_REFRESH_V2" not in text:
        if refresh_v1 not in text:
            print("costmap refresh block not found", file=sys.stderr)
            return 1
        text = text.replace(refresh_v1, refresh_v2, 1)
        target.write_text(text, encoding="utf-8")
        print(f"patched {target} -> DIMOS_GLOBAL_MAP_REFRESH_V2")
        return 0
    if "DIMOS_GLOBAL_MAP_REFRESH_V1" not in text and publish_open in text:
        if "    def _process(self, cloud: PointCloud2) -> None:\n" not in text:
            print("could not find _process for map refresh", file=sys.stderr)
            return 1
        text = text.replace(publish_open, publish_gated, 1)
        text = text.replace(
            "    def _process(self, cloud: PointCloud2) -> None:\n",
            due_method + "    def _process(self, cloud: PointCloud2) -> None:\n",
            1,
        )
        target.write_text(text, encoding="utf-8")
        print(f"patched {target} -> DIMOS_GLOBAL_MAP_REFRESH_V1")
        return 0
    if "DIMOS_PGO_LOCK_KEEP_V1" not in text:
        if stale not in text or raw not in text or publish_old not in text:
            print("could not upgrade the lidar-lock publish", file=sys.stderr)
            return 1
        text = text.replace(stale, kept, 1).replace(raw, held, 1).replace(publish_old, publish_new, 1)
        target.write_text(text, encoding="utf-8")
        changed = True
    print(f"patched {target} -> lidar frame" if changed else f"already patched: {target}")
    return 0


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
        return _upgrade(target, text)
    if OLD_START not in text or OLD_LOOP not in text or OLD_EMIT not in text:
        print("could not find PGO publish sites", file=sys.stderr)
        return 1
    if "    def _process(self, cloud: PointCloud2) -> None:\n" not in text:
        print("could not find _process", file=sys.stderr)
        return 1
    text = text.replace(OLD_START, NEW_START, 1)
    text = text.replace(OLD_LOOP, NEW_LOOP, 1)
    text = text.replace(OLD_EMIT, NEW_EMIT, 1)
    text = text.replace(
        "    def _process(self, cloud: PointCloud2) -> None:\n",
        HELPER + "    def _process(self, cloud: PointCloud2) -> None:\n",
        1,
    )
    target.write_text(text, encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
