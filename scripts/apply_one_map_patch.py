#!/usr/bin/env python3
"""One map in the viewer, and lidar matching in the odom world.

The prior cloud and the costmap were each moved by a different copy of the
pose, and every update stayed on the timeline, so the room drew on top of
itself and the gap grew. Both are now placed by the same world←map and logged
once. A body-frame scan was also being treated as a world scan, so the pose
followed the dog instead of staying on the map.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RERUN_ONE_MAP_V1"


def _site(argv: list[str]) -> Path:
    if argv:
        return Path(argv[0])
    return (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )


def _replace(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text(encoding="utf-8")
    if MARKER in text and label != "lidar-call":
        return
    if old not in text:
        raise SystemExit(f"{path}: missing {label}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def main() -> int:
    site = _site(sys.argv[1:])
    reloc = site / "dimos/mapping/relocalization/module.py"
    lidar = site / "dimos/mapping/relocalization/lidar/module.py"
    bridge = site / "dimos/visualization/rerun/bridge.py"
    for path in (reloc, lidar, bridge):
        if not path.is_file():
            raise SystemExit(f"missing {path}")

    reloc_text = reloc.read_text(encoding="utf-8")
    if "def _publish_loaded_map(" not in reloc_text:
        reloc_text = reloc_text.replace(
            "lambda _: self.loaded_map.publish(premap)",
            "lambda _: self._publish_loaded_map(premap)",
        )
        helper = '''
    def _publish_loaded_map(self, premap):
        # DIMOS_RERUN_ONE_MAP_V2 — locked loaded map, world view, one publish.
        if getattr(self, "_loaded_map_drawn", False):
            return
        tf = getattr(self, "_settle_last", None)
        if tf is None:
            return
        placed = premap.transform(tf.now() if hasattr(tf, "now") else tf)
        placed.frame_id = self.config.world_frame
        self._loaded_map_drawn = True
        self.loaded_map.publish(placed)
        logger.info("relocalize: loaded map drawn once in the world view")

'''
        needle = "    def _load_premap(self, map_file: str) -> None:\n"
        if needle not in reloc_text:
            raise SystemExit(f"{reloc}: no _load_premap")
        reloc_text = reloc_text.replace(needle, helper + needle, 1)
        reloc.write_text(reloc_text, encoding="utf-8")

    lidar_text = lidar.read_text(encoding="utf-8")
    if "DIMOS_RELOC_WORLD_CLOUD_V1" not in lidar_text:
        cloud_helper = '''
    def _cloud_in_world(self, msg):
        # DIMOS_RELOC_WORLD_CLOUD_V1 — a scan centered on the dog is not in the odom world.
        import math

        import numpy as np
        from dimos.msgs.geometry_msgs.Transform import Transform

        odom = getattr(self, "_latest_odom", None)
        if odom is None:
            return msg
        pts = msg.points_f32()
        if len(pts) < 100:
            return msg
        sample = pts[:: max(1, len(pts) // 4000)]
        ox, oy = float(odom.x), float(odom.y)
        near_origin = float(np.mean(np.hypot(sample[:, 0], sample[:, 1]) < 0.8))
        near_robot = float(np.mean(np.hypot(sample[:, 0] - ox, sample[:, 1] - oy) < 0.8))
        centroid = sample.mean(axis=0)
        c_origin = float(np.hypot(centroid[0], centroid[1]))
        c_robot = float(np.hypot(centroid[0] - ox, centroid[1] - oy))
        # DIMOS_RELOC_WORLD_CLOUD_V2 — a room cloud near the map origin is not a body scan.
        if c_origin > 0.7 or c_robot < 1.2 or near_robot + 0.05 >= near_origin:
            return msg
        placed = msg.transform(
            Transform.from_matrix(
                self._pose_matrix(ox, oy, float(odom.yaw)),
                frame_id="world",
                child_frame_id="base_link",
            )
        )
        if not getattr(self, "_body_cloud_logged", False):
            self._body_cloud_logged = True
            logger.info(
                "relocalize: lidar is centered on the robot, matching it in the odom world "
                f"near_origin={near_origin:.2f} near_robot={near_robot:.2f} "
                f"centroid_origin={c_origin:.2f} centroid_robot={c_robot:.2f}"
            )
        return placed

'''
        marker = "    def _relocalize(self, msg: PointCloud2) -> None:\n"
        if marker not in lidar_text:
            raise SystemExit(f"{lidar}: no _relocalize")
        lidar_text = lidar_text.replace(marker, cloud_helper + marker, 1)
        lidar_text = lidar_text.replace(
            "        assert self._relocalizer is not None\n",
            "        assert self._relocalizer is not None\n"
            "        msg = self._cloud_in_world(msg)  # DIMOS_RELOC_WORLD_CLOUD_V1\n",
            1,
        )
        old_submit = '''        self.submit(chosen, "lidar")
        if getattr(self, "_settle_anchor", None) is None:
            self._settle_anchor = chosen
        self._settle_last = chosen
'''
        new_submit = '''        if getattr(self, "_settle_anchor", None) is None:
            self._settle_anchor = chosen
        self._settle_last = chosen
        self.submit(chosen, "lidar")
'''
        if old_submit not in lidar_text:
            raise SystemExit(f"{lidar}: submit order not found")
        lidar_text = lidar_text.replace(old_submit, new_submit, 1)
        lidar.write_text(lidar_text, encoding="utf-8")

    lidar_now = lidar.read_text(encoding="utf-8")
    old_gate = '''        near_origin = float(np.mean(np.hypot(sample[:, 0], sample[:, 1]) < 0.8))
        near_robot = float(np.mean(np.hypot(sample[:, 0] - ox, sample[:, 1] - oy) < 0.8))
        if near_robot + 0.05 >= near_origin:
            return msg
'''
    new_gate = '''        near_origin = float(np.mean(np.hypot(sample[:, 0], sample[:, 1]) < 0.8))
        near_robot = float(np.mean(np.hypot(sample[:, 0] - ox, sample[:, 1] - oy) < 0.8))
        centroid = sample.mean(axis=0)
        c_origin = float(np.hypot(centroid[0], centroid[1]))
        c_robot = float(np.hypot(centroid[0] - ox, centroid[1] - oy))
        # DIMOS_RELOC_WORLD_CLOUD_V2 — a room cloud near the map origin is not a body scan.
        if c_origin > 0.7 or c_robot < 1.2 or near_robot + 0.05 >= near_origin:
            return msg
'''
    if "DIMOS_RELOC_WORLD_CLOUD_V2" not in lidar_now and old_gate in lidar_now:
        lidar.write_text(lidar_now.replace(old_gate, new_gate, 1), encoding="utf-8")
        print(f"upgraded body-cloud check: {lidar}")

    bridge_text = bridge.read_text(encoding="utf-8")
    if MARKER not in bridge_text:
        old_log = '''            # DIMOS_RERUN_MAP_TF_V2 — a static cloud ignores later yaw, so this one stays on the timeline.
            rr.log(entity_path, cast("Archetype", rerun_data))
'''
        new_log = '''            # DIMOS_RERUN_ONE_MAP_V1 — redraw these in place. Keeping every pose stacked the room.
            one_map = entity_path in {
                "world/loaded_map",
                "world/global_map",
                "world/global_costmap",
            }
            rr.log(entity_path, cast("Archetype", rerun_data), static=one_map)
'''
        if old_log not in bridge_text:
            raise SystemExit(f"{bridge}: log site not found")
        bridge_text = bridge_text.replace(old_log, new_log, 1)
        old_parent = '''            if not isinstance(rerun_data, rr.Transform3D):
                frame_id = getattr(msg, "frame_id", None)
'''
        new_parent = '''            if one_map:
                frame_id = None
            elif not isinstance(rerun_data, rr.Transform3D):
                frame_id = getattr(msg, "frame_id", None)
'''
        if old_parent not in bridge_text:
            raise SystemExit(f"{bridge}: parent site not found")
        bridge_text = bridge_text.replace(old_parent, new_parent, 1)
        # The following `if frame_id` must still exist; the old `if not isinstance` line is gone.
        bridge.write_text(bridge_text, encoding="utf-8")

    print(f"patched one map under {site}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
