#!/usr/bin/env python3
"""Patch DimOS lidar relocalization (V10).

DIMOS_INITIAL_POSE is where the dog is standing on the prior map, not the
world←map transform. The live cloud is already in the world frame, so the
map has to be placed with:

    world←map = odom * inverse(map pose of the dog)

Using the typed numbers directly draws the map about 2.8 m away from the dog
and seeds ICP outside its capture radius, so the clouds never meet.

V10 keeps the existing 1.8 m / 25° check, but applies it to the dog's recovered
position on the map. Thresholds are unchanged.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_SETTLE_PATCH_V10"
ODOM_MARKER = "DIMOS_RELOC_ODOM_COMPOSE_V1"

NEW_METHOD = '''
    def _pose_matrix(self, x: float, y: float, yaw: float):
        import math

        import numpy as np

        c, s = math.cos(yaw), math.sin(yaw)
        mat = np.eye(4)
        mat[0, 0], mat[0, 1], mat[1, 0], mat[1, 1] = c, -s, s, c
        mat[0, 3], mat[1, 3] = x, y
        return mat

    def _hint_map_pose(self):
        import math
        import os

        hint = os.environ.get("DIMOS_INITIAL_POSE", "").strip().replace("，", ",")
        if not hint:
            return None
        parts = [float(v) for v in hint.split(",")]
        if len(parts) < 3:
            return None
        return parts[0], parts[1], parts[2], math.degrees(parts[2])

    def _departure_world_map(self):
        """world←map that puts the dog on its typed map pose at the current odom."""
        import math

        import numpy as np

        hinted = self._hint_map_pose()
        odom = getattr(self, "_latest_odom", None)
        if hinted is None or odom is None:
            return None
        hx, hy, hyaw, _ = hinted
        world_base = self._pose_matrix(float(odom.x), float(odom.y), float(odom.yaw))
        map_base = self._pose_matrix(hx, hy, hyaw)
        world_map = world_base @ np.linalg.inv(map_base)
        yaw = math.atan2(float(world_map[1, 0]), float(world_map[0, 0]))
        return float(world_map[0, 3]), float(world_map[1, 3]), yaw

    def _robot_on_map(self, tf):
        """Where this world←map fix places the dog, in map meters and degrees."""
        import math

        import numpy as np

        odom = getattr(self, "_latest_odom", None)
        if odom is None:
            return None
        world_base = self._pose_matrix(float(odom.x), float(odom.y), float(odom.yaw))
        map_base = np.linalg.inv(tf.to_matrix()) @ world_base
        yaw = math.degrees(math.atan2(float(map_base[1, 0]), float(map_base[0, 0])))
        return float(map_base[0, 3]), float(map_base[1, 3]), yaw

    def _publish_departure_preview(self) -> None:
        import math
        import os

        import numpy as np
        from dimos.msgs.geometry_msgs.Transform import Transform

        if self._placed or getattr(self, "_settle_last", None) is not None:
            return
        seed = self._departure_world_map()
        hinted = self._hint_map_pose()
        if seed is None or hinted is None:
            return
        tx, ty, yaw = seed
        c, s = math.cos(yaw), math.sin(yaw)
        mat = np.eye(4)
        mat[0, 0], mat[0, 1], mat[1, 0], mat[1, 1] = c, -s, s, c
        mat[0, 3], mat[1, 3] = tx, ty
        provisional = Transform.from_matrix(
            mat, frame_id=self.config.world_frame, child_frame_id=self.config.map_frame
        )
        self.fixes.on_next(provisional)
        if not getattr(self, "_provisional_logged", False):
            self._provisional_logged = True
            hx, hy, _, hyaw_deg = hinted
            logger.info(
                f"relocalize provisional TF places the dog at map "
                f"x={hx:.2f} y={hy:.2f} yaw_deg={hyaw_deg:.1f} "
                f"(composed with odometry; ICP still running; not locked)"
            )

    def _on_odom(self, msg) -> None:
        import time

        self._latest_odom = msg
        now = time.monotonic()
        if now - getattr(self, "_provisional_at", 0.0) < 0.5:
            return
        self._provisional_at = now
        self._publish_departure_preview()

    def _relocalize(self, msg: PointCloud2) -> None:
        # DIMOS_RELOC_SETTLE_PATCH_V10
        assert self._relocalizer is not None
        import math
        import os

        from dimos.mapping.relocalization.module import fix_error, yaw_deg

        def _xy(tf_):
            try:
                return float(tf_.translation[0]), float(tf_.translation[1])
            except Exception:
                return float(tf_.translation.x), float(tf_.translation.y)

        tracking = getattr(self, "_settle_last", None) is not None
        if tracking:
            os.environ.pop("DIMOS_RELOC_WORLD_FROM_MAP", None)
        else:
            seed = self._departure_world_map()
            if seed is None:
                logger.info("relocalize: waiting for odometry before matching the departure pose")
                return
            os.environ["DIMOS_RELOC_WORLD_FROM_MAP"] = f"{seed[0]:.4f},{seed[1]:.4f},{seed[2]:.5f}"

        t0 = time.monotonic()
        try:
            tf = self._relocalizer.relocalize(
                msg.pointcloud, self.config.world_frame, self.config.map_frame
            )
        except Exception:
            logger.exception("relocalize() failed")
            return
        dt = time.monotonic() - t0
        if tf is None:
            logger.info(
                f"relocalize lidar: refused after {dt:.1f}s n_pts={len(msg)} "
                f"(below fitness_threshold={self.config.relocalize.fitness_threshold})"
            )
            return
        logger.info(f"relocalize lidar: time_cost={dt:.1f}s n_pts={len(msg)}")

        hinted = None
        if not tracking:
            try:
                hinted = self._hint_map_pose()
            except ValueError:
                hinted = None
        if hinted is not None:
            placed = self._robot_on_map(tf)
            if placed is None:
                logger.info("relocalize settle: no odometry; cannot check the departure pose")
                return
            px, py, pyaw = placed
            hx, hy, _, hyaw = hinted
            dy = (pyaw - hyaw + 180.0) % 360.0 - 180.0
            dist_hint = math.hypot(px - hx, py - hy)
            max_hint_yaw = float(os.environ.get("DIMOS_RELOC_HINT_YAW_DEG", "25"))
            max_hint_xy = float(os.environ.get("DIMOS_RELOC_HINT_XY_M", "1.8"))
            if abs(dy) > max_hint_yaw or dist_hint > max_hint_xy:
                logger.info(
                    f"relocalize settle: reject vs departure pose "
                    f"map=({px:.2f},{py:.2f},{pyaw:.1f}) "
                    f"hint=({hx:.2f},{hy:.2f},{hyaw:.1f}) "
                    f"dyaw={dy:.1f} xy_err={dist_hint:.2f}m "
                    f"(max {max_hint_xy}m / {max_hint_yaw}deg)"
                )
                return

        n_need = max(1, int(os.environ.get("DIMOS_RELOC_SETTLE_N", "3")))
        if tracking:
            n_need = max(1, int(os.environ.get("DIMOS_RELOC_TRACK_N", "1")))
        max_xy = float(os.environ.get("DIMOS_RELOC_SETTLE_M", "0.5"))
        max_yaw = float(os.environ.get("DIMOS_RELOC_SETTLE_YAW_DEG", "15"))
        buf = getattr(self, "_settle_buf", [])
        buf.append(tf)
        self._settle_buf = buf[-n_need:]
        if len(self._settle_buf) < n_need:
            logger.info(
                f"relocalize settle {len(self._settle_buf)}/{n_need} "
                f"yaw={yaw_deg(tf):.1f} t={tf.translation}"
            )
            return
        agreed = True
        for i, a in enumerate(self._settle_buf):
            for b in self._settle_buf[i + 1 :]:
                dyaw, dist = fix_error(a, b)
                if abs(dyaw) > max_yaw or dist > max_xy:
                    agreed = False
                    break
            if not agreed:
                break
        if not agreed:
            logger.info(
                f"relocalize settle disagreement; keep last "
                f"yaw={yaw_deg(tf):.1f} t={tf.translation}"
            )
            self._settle_buf = self._settle_buf[-1:]
            return

        def score(t):
            if hinted is None:
                return 0.0
            placed = self._robot_on_map(t)
            if placed is None:
                return 0.0
            px, py, pyaw = placed
            hx, hy, _, hyaw = hinted
            dy = (pyaw - hyaw + 180.0) % 360.0 - 180.0
            return math.hypot(px - hx, py - hy) + 0.05 * abs(dy)

        chosen = min(self._settle_buf, key=score) if hinted is not None else tf
        last = getattr(self, "_settle_last", None)
        if last is not None:
            dyaw, dist = fix_error(chosen, last)
            max_step_xy = float(os.environ.get("DIMOS_RELOC_TRACK_STEP_M", "0.6"))
            max_step_yaw = float(os.environ.get("DIMOS_RELOC_TRACK_STEP_YAW_DEG", "20"))
            if abs(dyaw) > max_step_yaw or dist > max_step_xy:
                logger.info(
                    f"relocalize track: reject jump yaw={yaw_deg(chosen):.1f} "
                    f"dyaw={dyaw:.1f} dist={dist:.2f}m "
                    f"(max {max_step_xy}m / {max_step_yaw}deg)"
                )
                self._settle_buf = []
                return
            logger.info(
                f"relocalize track update yaw={yaw_deg(chosen):.1f} "
                f"dyaw={dyaw:.1f} dist={dist:.2f}m t={chosen.translation}"
            )
        else:
            placed = self._robot_on_map(chosen)
            where = ""
            if placed is not None:
                where = f" dog_on_map=({placed[0]:.2f},{placed[1]:.2f},{placed[2]:.1f})"
            logger.info(
                f"relocalize settle {n_need}/{n_need} agreed "
                f"yaw={yaw_deg(chosen):.1f} t={chosen.translation}{where}"
            )
        self.submit(chosen, "lidar")
        if getattr(self, "_settle_anchor", None) is None:
            self._settle_anchor = chosen
        self._settle_last = chosen
        sx, sy = _xy(chosen)
        os.environ["DIMOS_RELOC_SEED_POSE"] = (
            f"{sx:.4f},{sy:.4f},{math.radians(yaw_deg(chosen)):.5f}"
        )
        os.environ.pop("DIMOS_RELOC_WORLD_FROM_MAP", None)
        self._settle_buf = []
'''


def _ensure_start_rpc(text: str) -> str:
    """The overridden start() must stay an @rpc or DimOS pickles the whole module."""
    old = '''    odom: In[PoseStamped]

    def start(self) -> None:
'''
    new = '''    odom: In[PoseStamped]

    @rpc
    def start(self) -> None:
'''
    if old in text and "@rpc\n    def start(self) -> None:" not in text.split("class LidarWindowRelocalization", 1)[-1]:
        return text.replace(old, new, 1)
    return text


def main() -> int:
    if len(sys.argv) > 1:
        target = Path(sys.argv[1])
    else:
        target = (
            Path(__file__).resolve().parents[1]
            / "simulation_legacy/.venv/lib/python3.12/site-packages"
            / "dimos/mapping/relocalization/lidar/module.py"
        )
    if not target.is_file():
        print(f"module not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        preview = '''        self.fixes.on_next(provisional)
        if not getattr(self, "_provisional_logged", False):
'''
        preview_fast = '''        self.fixes.on_next(provisional)
        # DIMOS_PREVIEW_MAP_V1 — global map draws on the typed pose while ICP runs.
        if not getattr(self, "_preview_map_written", False):
            self._preview_map_written = True
            try:
                import json
                from pathlib import Path as _Path

                _out = os.environ.get("PATROL_TRANSFORM_FILE", "").strip()
                if _out:
                    _t = provisional.translation
                    _r = provisional.rotation
                    _payload = {
                        "frame_id": str(provisional.frame_id),
                        "child_frame_id": str(provisional.child_frame_id),
                        "translation": [float(_t[0]), float(_t[1]), float(_t[2])],
                        "rotation_xyzw": [float(_r.x), float(_r.y), float(_r.z), float(_r.w)],
                        "yaw_deg": round(math.degrees(yaw), 3),
                        "source": "preview",
                    }
                    _path = _Path(_out)
                    _path.parent.mkdir(parents=True, exist_ok=True)
                    _tmp = _path.with_suffix(_path.suffix + ".tmp")
                    _tmp.write_text(json.dumps(_payload) + "\\n", encoding="utf-8")
                    os.replace(_tmp, _path)
            except Exception:
                logger.exception("relocalize preview map write failed")
        if not getattr(self, "_provisional_logged", False):
'''
        if "DIMOS_PREVIEW_MAP_V1" not in text and preview in text:
            text = text.replace(preview, preview_fast, 1)
        text = _ensure_start_rpc(text)
        text = text.replace(
            'hint = os.environ.get("DIMOS_INITIAL_POSE", "").strip()\n        if not hint:',
            'hint = os.environ.get("DIMOS_INITIAL_POSE", "").strip().replace("，", ",")\n        if not hint:',
            1,
        )
        target.write_text(text, encoding="utf-8")
        print(f"already patched: {target}")
        return 0
    start = text.find("    def _relocalize(self, msg: PointCloud2) -> None:")
    if start < 0:
        print("could not find _relocalize", file=sys.stderr)
        return 1
    end = text.find("\nclass LidarWindowRelocalization", start)
    if end < 0:
        print("could not find end of _relocalize", file=sys.stderr)
        return 1
    patched = text[:start] + NEW_METHOD.strip("\n") + "\n\n\n" + text[end:]
    if "PoseStamped" not in patched:
        needle = "from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2\n"
        insert = needle + "from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped\n"
        if needle not in patched:
            print("could not add PoseStamped import", file=sys.stderr)
            return 1
        patched = patched.replace(needle, insert, 1)
    old_class = '''class LidarWindowRelocalization(CloudRelocalization):
    """Matches the last `max_frames` scans merged."""

    lidar: In[PointCloud2]

    def clouds(self) -> Observable[PointCloud2]:
'''
    new_class = '''class LidarWindowRelocalization(CloudRelocalization):
    """Matches the last `max_frames` scans merged."""

    lidar: In[PointCloud2]
    # DIMOS_RELOC_ODOM_COMPOSE_V1 — current pose of the dog in the world frame.
    odom: In[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self._latest_odom = None
        self._provisional_at = 0.0
        self.register_disposable(self.odom.observable().subscribe(self._on_odom))

    def clouds(self) -> Observable[PointCloud2]:
'''
    if ODOM_MARKER not in patched:
        if old_class not in patched:
            print("could not find LidarWindowRelocalization header", file=sys.stderr)
            return 1
        patched = patched.replace(old_class, new_class, 1)
    backup = target.with_suffix(target.suffix + ".pre_settle")
    if not backup.exists():
        backup.write_text(text, encoding="utf-8")
    target.write_text(patched, encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
