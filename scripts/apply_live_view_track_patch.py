#!/usr/bin/env python3
"""Lighter live Rerun view, a green odometry trail, and wall-grid tracking.

The live scan was 12k points and the raw camera frame streamed at 1 Hz over the
hotspot, so the viewer lagged seconds behind the dog. Tracking only ran a narrow
ICP, so odometry drift turned the live cloud off the global map while walking.
"""
from __future__ import annotations

import sys
from pathlib import Path

SITE = (
    Path(sys.argv[1])
    if len(sys.argv) > 1
    else Path(__file__).resolve().parents[1] / "simulation_legacy/.venv/lib/python3.12/site-packages"
)


def edit(path: Path, marker: str, pairs: list[tuple[str, str]]) -> None:
    text = path.read_text(encoding="utf-8")
    if marker in text:
        print(f"already patched: {path.name} {marker}")
        return
    for old, new in pairs:
        if text.count(old) != 1:
            raise SystemExit(f"{path.name}: anchor found {text.count(old)} times: {old[:70]!r}")
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} -> {marker}")


def main() -> int:
    pc2 = SITE / "dimos/msgs/sensor_msgs/PointCloud2.py"
    edit(
        pc2,
        "DIMOS_RERUN_LIVE_POINTS_V1",
        [
            (
                '        cap = int(kwargs.pop("max_points", 12000) or 0)\n',
                "        # DIMOS_RERUN_LIVE_POINTS_V1 — live scans are sampled for the viewer link.\n"
                '        cap = int(kwargs.pop("max_points", int(__import__("os").environ.get("DIMOS_RERUN_LIVE_POINTS", "4000"))) or 0)\n',
            )
        ],
    )

    bridge = SITE / "dimos/visualization/rerun/bridge.py"
    edit(
        bridge,
        "DIMOS_RERUN_TRAIL_V1",
        [
            (
                '            if os.environ.get("DIMOS_SHOW_LOADED_MAP", "0").lower() in ("1", "true", "yes"):\n'
                '                keep.add("world/loaded_map")\n'
                "            if entity_path not in keep:\n",
                '            if os.environ.get("DIMOS_SHOW_LOADED_MAP", "0").lower() in ("1", "true", "yes"):\n'
                '                keep.add("world/loaded_map")\n'
                "            # A raw camera frame is megabytes; it starved the scan and pose.\n"
                '            if os.environ.get("DIMOS_RERUN_CAMERA", "0").lower() not in ("1", "true", "yes"):\n'
                '                keep.discard("world/color_image")\n'
                '                keep.discard("world/camera_info")\n'
                "            if entity_path not in keep:\n",
            ),
            (
                '        entity_path: str = self._get_entity_path(topic)\n'
                "        if self._tf_tree is not None and isinstance(msg, TFMessage):\n",
                '        entity_path: str = self._get_entity_path(topic)\n'
                "        # DIMOS_RERUN_TRAIL_V1 — where the dog actually walked, as a thin green line.\n"
                '        if entity_path == "world/odom":\n'
                "            try:\n"
                '                _p = (float(msg.x), float(msg.y), float(getattr(msg, "z", 0.0)) + 0.05)\n'
                '                _trail = getattr(self, "_trail", None)\n'
                "                if _trail is None:\n"
                "                    _trail = self._trail = []\n"
                '                    rr.log("world/trail", rr.Transform3D(parent_frame="tf#/world"), static=True)\n'
                "                _last = _trail[-1] if _trail else None\n"
                "                if _last is None or (_p[0] - _last[0]) ** 2 + (_p[1] - _last[1]) ** 2 > 0.0004:\n"
                "                    _trail.append(_p)\n"
                "                    if len(_trail) > 6000:\n"
                "                        del _trail[:1000]\n"
                "                    if len(_trail) >= 2:\n"
                "                        rr.log(\n"
                '                            "world/trail",\n'
                "                            rr.LineStrips3D([_trail], colors=[[0, 255, 0]], radii=0.01),\n"
                "                            static=True,\n"
                "                        )\n"
                "            except Exception:\n"
                '                logger.debug("trail draw failed", exc_info=True)\n'
                "        if self._tf_tree is not None and isinstance(msg, TFMessage):\n",
            ),
        ],
    )

    lidar_module = SITE / "dimos/mapping/relocalization/lidar/module.py"
    edit(
        lidar_module,
        "DIMOS_RELOC_TRACK_ODOM_V1",
        [
            (
                "        if tracking:\n"
                '            os.environ.pop("DIMOS_RELOC_WORLD_FROM_MAP", None)\n',
                "        if tracking:\n"
                '            os.environ.pop("DIMOS_RELOC_WORLD_FROM_MAP", None)\n'
                "            # DIMOS_RELOC_TRACK_ODOM_V1 — the tracker re-scores walls around this pose.\n"
                '            _odom_now = getattr(self, "_latest_odom", None)\n'
                "            if _odom_now is not None:\n"
                '                os.environ["DIMOS_RELOC_ODOM"] = (\n'
                '                    f"{float(_odom_now.x):.4f},{float(_odom_now.y):.4f},{float(_odom_now.yaw):.5f}"\n'
                "                )\n",
            )
        ],
    )

    edit(
        lidar_module,
        "DIMOS_RELOC_NUDGE_V1",
        [
            (
                "    def _on_odom(self, msg) -> None:\n"
                "        import math\n"
                "        import time\n"
                "\n"
                "        self._latest_odom = msg\n",
                "    def _check_nudge(self) -> None:\n"
                "        # DIMOS_RELOC_NUDGE_V1 — operator steps the locked dog pose on the map.\n"
                "        import json\n"
                "        import math\n"
                "        import os\n"
                "        import time\n"
                "\n"
                "        import numpy as np\n"
                "        from dimos.msgs.geometry_msgs.Transform import Transform\n"
                "\n"
                "        now = time.monotonic()\n"
                '        if now - getattr(self, "_nudge_at", 0.0) < 0.2:\n'
                "            return\n"
                "        self._nudge_at = now\n"
                '        path = os.environ.get("PATROL_NUDGE_FILE", "").strip()\n'
                "        if not path:\n"
                "            return\n"
                "        try:\n"
                "            data = json.loads(open(path, encoding='utf-8').read())\n"
                '            seq = int(data.get("seq", 0))\n'
                "        except (OSError, ValueError, TypeError, AttributeError):\n"
                "            seq = 0\n"
                "            data = {}\n"
                '        if not hasattr(self, "_nudge_seq"):\n'
                "            self._nudge_seq = seq\n"
                "            return\n"
                "        if seq <= self._nudge_seq:\n"
                "            return\n"
                "        self._nudge_seq = seq\n"
                '        lock = getattr(self, "_settle_last", None)\n'
                '        odom = getattr(self, "_latest_odom", None)\n'
                "        if lock is None or odom is None:\n"
                '            logger.info("relocalize nudge ignored: no locked pose yet")\n'
                "            return\n"
                "        wm = lock.to_matrix()\n"
                "        wb = self._pose_matrix(float(odom.x), float(odom.y), float(odom.yaw))\n"
                "        mb = np.linalg.inv(wm) @ wb\n"
                "        x = float(mb[0, 3]) + float(data.get('dx', 0.0))\n"
                "        y = float(mb[1, 3]) + float(data.get('dy', 0.0))\n"
                "        yaw = math.atan2(float(mb[1, 0]), float(mb[0, 0])) + math.radians(float(data.get('dyaw_deg', 0.0)))\n"
                "        wm_new = wb @ np.linalg.inv(self._pose_matrix(x, y, yaw))\n"
                "        tf = Transform.from_matrix(\n"
                "            wm_new, frame_id=self.config.world_frame, child_frame_id=self.config.map_frame\n"
                "        )\n"
                "        self._settle_last = tf\n"
                "        self._settle_anchor = tf\n"
                "        self._settle_buf = []\n"
                "        self.submit(tf, \"nudge\")\n"
                "        wyaw = math.atan2(float(wm_new[1, 0]), float(wm_new[0, 0]))\n"
                '        os.environ["DIMOS_RELOC_SEED_POSE"] = (\n'
                '            f"{float(wm_new[0, 3]):.4f},{float(wm_new[1, 3]):.4f},{wyaw:.5f}"\n'
                "        )\n"
                "        logger.info(\n"
                '            f"relocalize nudge {seq}: dog_on_map=({x:.2f},{y:.2f},{math.degrees(yaw):.1f})"\n'
                "        )\n"
                "\n"
                "    def _on_odom(self, msg) -> None:\n"
                "        import math\n"
                "        import time\n"
                "\n"
                "        self._latest_odom = msg\n"
                "        try:\n"
                "            self._check_nudge()\n"
                "        except Exception:\n"
                '            logger.exception("relocalize nudge failed")\n',
            )
        ],
    )

    edit(
        lidar_module,
        "DIMOS_RELOC_DOG_POSE_V1",
        [
            (
                "        self._nudge_at = now\n",
                "        self._nudge_at = now\n"
                "        # DIMOS_RELOC_DOG_POSE_V1 — live dog pose on the loaded map for the console.\n"
                '        pose_path = os.environ.get("PATROL_DOG_POSE_FILE", "").strip()\n'
                '        lock = getattr(self, "_settle_last", None)\n'
                '        odom = getattr(self, "_latest_odom", None)\n'
                "        if pose_path and lock is not None and odom is not None:\n"
                "            mb = np.linalg.inv(lock.to_matrix()) @ self._pose_matrix(\n"
                "                float(odom.x), float(odom.y), float(odom.yaw)\n"
                "            )\n"
                "            pose = {\n"
                '                "x": round(float(mb[0, 3]), 3),\n'
                '                "y": round(float(mb[1, 3]), 3),\n'
                '                "yaw": round(math.atan2(float(mb[1, 0]), float(mb[0, 0])), 4),\n'
                '                "nudge_seq": getattr(self, "_nudge_seq", 0),\n'
                '                "t": time.time(),\n'
                "            }\n"
                "            try:\n"
                '                tmp = pose_path + ".tmp"\n'
                "                with open(tmp, 'w', encoding='utf-8') as f:\n"
                "                    json.dump(pose, f)\n"
                "                os.replace(tmp, pose_path)\n"
                "            except OSError:\n"
                "                pass\n",
            )
        ],
    )

    edit(
        lidar_module,
        "DIMOS_RELOC_TRUST_HINT_V1",
        [
            (
                '            os.environ["DIMOS_RELOC_WORLD_FROM_MAP"] = f"{seed[0]:.4f},{seed[1]:.4f},{seed[2]:.5f}"\n',
                '            os.environ["DIMOS_RELOC_WORLD_FROM_MAP"] = f"{seed[0]:.4f},{seed[1]:.4f},{seed[2]:.5f}"\n'
                "            # DIMOS_RELOC_TRUST_HINT_V1 — operator placed the dog; lock on the typed pose.\n"
                '            if os.environ.get("DIMOS_RELOC_TRUST_HINT", "0") == "1":\n'
                "                from dimos.msgs.geometry_msgs.Transform import Transform\n"
                "\n"
                "                wm = self._pose_matrix(float(seed[0]), float(seed[1]), float(seed[2]))\n"
                "                chosen = Transform.from_matrix(\n"
                "                    wm, frame_id=self.config.world_frame, child_frame_id=self.config.map_frame\n"
                "                )\n"
                "                self._settle_anchor = chosen\n"
                "                self._settle_last = chosen\n"
                '                self.submit(chosen, "hint")\n'
                '                os.environ["DIMOS_RELOC_SEED_POSE"] = (\n'
                '                    f"{float(seed[0]):.4f},{float(seed[1]):.4f},{float(seed[2]):.5f}"\n'
                "                )\n"
                '                logger.info(f"relocalize locked on the typed pose: dog_on_map={self._robot_on_map(chosen)}")\n'
                "                return\n",
            )
        ],
    )

    edit(
        bridge,
        "DIMOS_RERUN_TRAIL_V2",
        [
            (
                "                _last = _trail[-1] if _trail else None\n"
                "                if _last is None or (_p[0] - _last[0]) ** 2 + (_p[1] - _last[1]) ** 2 > 0.0004:\n"
                "                    _trail.append(_p)\n"
                "                    if len(_trail) > 6000:\n"
                "                        del _trail[:1000]\n"
                "                    if len(_trail) >= 2:\n"
                "                        rr.log(\n"
                '                            "world/trail",\n'
                "                            rr.LineStrips3D([_trail], colors=[[0, 255, 0]], radii=0.01),\n"
                "                            static=True,\n"
                "                        )\n",
                "                # DIMOS_RERUN_TRAIL_V2 — resending the whole trail on every 2 cm\n"
                "                # step saturated the link. Finished 200-point chunks go out once;\n"
                "                # the open chunk is resent at most once a second.\n"
                "                import time as _time\n"
                "\n"
                "                _last = _trail[-1] if _trail else None\n"
                "                if _last is None or (_p[0] - _last[0]) ** 2 + (_p[1] - _last[1]) ** 2 > 0.0025:\n"
                "                    _trail.append(_p)\n"
                "                    _now = _time.monotonic()\n"
                '                    _chunk = getattr(self, "_trail_chunk", 0)\n'
                "                    _strip = rr.LineStrips3D([_trail], colors=[[0, 255, 0]], radii=0.01)\n"
                "                    if len(_trail) >= 200:\n"
                '                        rr.log(f"world/trail/{_chunk}", _strip, static=True)\n'
                "                        if _chunk >= 60:\n"
                '                            rr.log(f"world/trail/{_chunk - 60}", rr.Clear(recursive=False), static=True)\n'
                "                        self._trail_chunk = _chunk + 1\n"
                "                        _trail[:] = [_trail[-1]]\n"
                "                        self._trail_sent = _now\n"
                '                    elif len(_trail) >= 2 and _now - getattr(self, "_trail_sent", 0.0) >= 1.0:\n'
                '                        rr.log(f"world/trail/{_chunk}", _strip, static=True)\n'
                "                        self._trail_sent = _now\n",
            )
        ],
    )

    reloc = SITE / "dimos/mapping/relocalization/lidar/relocalize.py"
    edit(
        reloc,
        "DIMOS_RELOC_TRACK_GRID_V1",
        [
            (
                "                    _local = []\n"
                "                    for _dyaw in (0.0,):  # DIMOS_RELOC_TRACK_YAW_V1\n",
                "                    # DIMOS_RELOC_TRACK_GRID_V1 — re-score walls around the tracked\n"
                "                    # pose. Narrow ICP alone let odometry drift turn the scan off the map.\n"
                '                    _wg = getattr(self, "_wall_grid", None)\n'
                '                    _od = os.environ.get("DIMOS_RELOC_ODOM", "").strip()\n'
                "                    if _wg is not None and _od:\n"
                "                        _o = [float(v) for v in _od.split(',')]\n"
                "                        _lo, _gw, _gt, _mc = _wg\n"
                "                        _res = 0.05\n"
                "                        _pts = np.asarray(local_map.voxel_down_sample(0.05).points)\n"
                "                        _xy = np.zeros((0, 2))\n"
                "                        if len(_pts):\n"
                "                            _z = _pts[:, 2]\n"
                "                            _lowz = _z[_z <= np.median(_z)]\n"
                "                            _h, _e = np.histogram(_lowz, bins=np.arange(_lowz.min(), _lowz.max() + 0.05, 0.05))\n"
                "                            _fl = float(_e[int(np.argmax(_h))] + 0.025) if len(_h) else float(_z.min())\n"
                "                            _xy = _pts[(_z > _fl + 0.25) & (_z < _fl + 1.8), :2]\n"
                "                        if len(_xy) > 2500:\n"
                "                            _xy = _xy[np.random.default_rng(2).choice(len(_xy), 2500, replace=False)]\n"
                "                        if len(_xy) >= 800:\n"
                "\n"
                "                            def _pose(x, y, yaw):\n"
                "                                _c, _s = math.cos(yaw), math.sin(yaw)\n"
                "                                _m = np.eye(4)\n"
                "                                _m[0, 0], _m[0, 1], _m[1, 0], _m[1, 1] = _c, -_s, _s, _c\n"
                "                                _m[0, 3], _m[1, 3] = x, y\n"
                "                                return _m\n"
                "\n"
                "                            _wm = _pose(_hx, _hy, _hyaw)\n"
                "                            _wb = _pose(_o[0], _o[1], _o[2])\n"
                "                            _mb = np.linalg.inv(_wm) @ _wb\n"
                "                            _bx, _by = float(_mb[0, 3]), float(_mb[1, 3])\n"
                "                            _byaw = math.atan2(float(_mb[1, 0]), float(_mb[0, 0]))\n"
                "                            _bw = np.linalg.inv(_wb)\n"
                "                            _pb = _xy @ _bw[:2, :2].T + _bw[:2, 3]\n"
                "\n"
                "                            def _hits(pm):\n"
                "                                _q = ((pm - _lo) / _res).astype(int)\n"
                "                                _ok = (\n"
                "                                    (_q[:, 0] >= 0) & (_q[:, 1] >= 0)\n"
                "                                    & (_q[:, 0] < _gt.shape[0]) & (_q[:, 1] < _gt.shape[1])\n"
                "                                )\n"
                "                                _v = np.zeros(len(_q), dtype=bool)\n"
                "                                _v[_ok] = _gt[_q[_ok, 0], _q[_ok, 1]]\n"
                "                                return float(_v.mean())\n"
                "\n"
                "                            _best = None\n"
                "                            _base = None\n"
                "                            for _dd in range(-4, 5):\n"
                "                                _yaw = _byaw + math.radians(_dd)\n"
                "                                _c, _s = math.cos(_yaw), math.sin(_yaw)\n"
                "                                _rot = _pb @ np.array([[_c, -_s], [_s, _c]]).T\n"
                "                                for _ix in range(-4, 5):\n"
                "                                    for _iy in range(-4, 5):\n"
                "                                        _x = _bx + _ix * 0.05\n"
                "                                        _y = _by + _iy * 0.05\n"
                "                                        _f = _hits(_rot + (_x, _y))\n"
                "                                        if _dd == 0 and _ix == 0 and _iy == 0:\n"
                "                                            _base = _f\n"
                "                                        if _best is None or _f > _best[0]:\n"
                "                                            _best = (_f, _x, _y, _yaw)\n"
                "                            if _best[0] > _base + 0.02:\n"
                "                                _T = _pose(_best[1], _best[2], _best[3]) @ _bw\n"
                "                            else:\n"
                "                                _T = np.linalg.inv(_wm)\n"
                "                            _T = np.ascontiguousarray(_T)\n"
                "                            _r = reg.evaluate_registration(\n"
                "                                source.fine, target.fine, cfg.voxel_fine * cfg.icp_dist_factor * 2, _T\n"
                "                            )\n"
                "                            _r.transformation = _T\n"
                "                            return _r\n"
                "                    _local = []\n"
                "                    for _dyaw in (0.0,):  # DIMOS_RELOC_TRACK_YAW_V1\n",
            )
        ],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
