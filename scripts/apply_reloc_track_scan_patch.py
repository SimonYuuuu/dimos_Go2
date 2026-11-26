#!/usr/bin/env python3
"""Track with one nearby scan and a fine wall ICP; trust a good match.

Offline replay of the last patrol showed the live cloud is already an odometry
voxel map. Merging seven of those frames and ICP at 0.30 m voxel left walls
ambiguous (map wall thickness ≈ 0.25 m). A single nearby wall band with 0.10 m
point-to-plane ICP kept 10 cm inliers around 0.84 while the published pose
fell to 0.76 and the room rotated away.

When that match scores ≥ 0.80, apply a larger share of the yaw/xy error. The
previous slow-correct rails left odometry free to walk the map off.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_TRACK_SCAN_V1"

WINDOW_OLD = '''def window(
    scans: Observable[PointCloud2], cfg: RelocalizeConfig, interval: float
) -> Observable[PointCloud2]:
    """The last ``max_frames`` scans merged, at most one merge every ``interval`` s."""
    held: deque[PointCloud2] = deque(maxlen=cfg.max_frames)
    return scans.pipe(
        ops.do_action(held.append),
        ops.throttle_first(interval),
        ops.filter(lambda _: len(held) >= cfg.min_frames),
        ops.map(lambda _: reduce(operator.add, held)),
    )
'''

WINDOW_NEW = '''def window(
    scans: Observable[PointCloud2], cfg: RelocalizeConfig, interval: float
) -> Observable[PointCloud2]:
    """The last ``max_frames`` scans merged, at most one merge every ``interval`` s."""
    # DIMOS_RELOC_TRACK_SCAN_V1 — after lock, match one nearby frame, not seven.
    held: deque[PointCloud2] = deque(maxlen=max(cfg.max_frames, 1))

    def _merge(_: object) -> PointCloud2:
        import os

        n = cfg.max_frames
        if os.environ.get("DIMOS_RELOC_SEED_POSE", "").strip():
            n = max(1, int(os.environ.get("DIMOS_RELOC_TRACK_FRAMES", "1")))
        use = list(held)[-n:]
        return reduce(operator.add, use)

    return scans.pipe(
        ops.do_action(held.append),
        ops.throttle_first(interval),
        ops.filter(lambda _: len(held) >= min(cfg.min_frames, 1)),
        ops.map(_merge),
    )
'''

CROP_OLD = '''        msg = self._cloud_in_world(msg)  # DIMOS_RELOC_WORLD_CLOUD_V1
        import math
        import os
'''

CROP_NEW = '''        msg = self._cloud_in_world(msg)  # DIMOS_RELOC_WORLD_CLOUD_V1
        import math
        import os

        # DIMOS_RELOC_TRACK_SCAN_V1 — wall band near the dog for tracking ICP.
        if getattr(self, "_settle_last", None) is not None:
            try:
                import numpy as np
                from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as _PC2

                _odom = getattr(self, "_latest_odom", None)
                if _odom is not None:
                    _pts = np.asarray(msg.points_f32(), dtype=np.float32)
                    if _pts.ndim == 2 and _pts.shape[0] >= 200 and _pts.shape[1] >= 3:
                        _r = float(os.environ.get("DIMOS_RELOC_TRACK_RADIUS_M", "8"))
                        _near = np.hypot(
                            _pts[:, 0] - float(_odom.x), _pts[:, 1] - float(_odom.y)
                        )
                        _q = _pts[_near < _r]
                        if len(_q) >= 200:
                            _g = float(np.percentile(_q[:, 2], 10))
                            _band = _q[(_q[:, 2] > _g + 0.25) & (_q[:, 2] <= _g + 1.2)]
                            if len(_band) >= 200:
                                msg = _PC2.from_numpy(
                                    _band[:, :3],
                                    frame_id=getattr(msg, "frame_id", "world"),
                                )
            except Exception:
                pass
'''

ALIGN_OLD = '''        cfg = self.config
        reg = o3d.pipelines.registration
        source = self._prepare(local_map)
        target = self._target
        dist = cfg.voxel_coarse * cfg.coarse_dist_factor
'''

ALIGN_NEW = '''        cfg = self.config
        reg = o3d.pipelines.registration
        # DIMOS_RELOC_TRACK_SCAN_V1 — fine wall ICP against a matching prior.
        import os as _os_track

        _seed_early = _os_track.environ.get("DIMOS_RELOC_SEED_POSE", "").strip()
        _vf_save = _idf_save = None
        if _seed_early:
            _vf_save, _idf_save = cfg.voxel_fine, cfg.icp_dist_factor
            cfg.voxel_fine = float(_os_track.environ.get("DIMOS_RELOC_TRACK_VOXEL", "0.10"))
            cfg.icp_dist_factor = float(_os_track.environ.get("DIMOS_RELOC_TRACK_ICP_DIST", "1.2"))
            if getattr(self, "_track_target_voxel", None) != cfg.voxel_fine:
                self._track_target = self._prepare(self.map)
                self._track_target_voxel = cfg.voxel_fine
        source = self._prepare(local_map)
        target = getattr(self, "_track_target", None) if _seed_early else self._target
        if target is None:
            target = self._target
        dist = cfg.voxel_coarse * cfg.coarse_dist_factor
'''

SEED_YAW_OLD = '''                    for _dyaw in (0.0,):  # DIMOS_RELOC_TRACK_YAW_V1
'''

SEED_YAW_NEW = '''                    for _dyaw in (0.0, math.radians(3), math.radians(-3)):  # DIMOS_RELOC_TRACK_YAW_V1
'''

SEED_RETURN_OLD = '''                    if _local:
                        return max(_local, key=lambda r: r.fitness)
            except Exception:
                pass
'''

SEED_RETURN_NEW = '''                    if _local:
                        _best = max(_local, key=lambda r: r.fitness)
                        if _vf_save is not None:
                            cfg.voxel_fine, cfg.icp_dist_factor = _vf_save, _idf_save
                        return _best
                    if _vf_save is not None:
                        cfg.voxel_fine, cfg.icp_dist_factor = _vf_save, _idf_save
            except Exception:
                if _vf_save is not None:
                    try:
                        cfg.voxel_fine, cfg.icp_dist_factor = _vf_save, _idf_save
                    except Exception:
                        pass
'''

FUSE_OLD = '''                    # DIMOS_RELOC_SLOW_CORRECT_V1
                    _ey = lock_yaw if recover else err_yaw
                    _ex = lock_xy if recover else err_xy
                    _sign = 1 if _ey > 2.5 else (-1 if _ey < -2.5 else 0)
                    _agree = list(getattr(self, "_slow_agree", []))
                    _agree.append(_sign)
                    self._slow_agree = _agree[-4:]
                    _same = (
                        len(self._slow_agree) == 4
                        and len(set(self._slow_agree)) == 1
                        and self._slow_agree[0] != 0
                    )
                    if not _same and _ex < 0.20:
                        step_yaw = 0.0
                        scale = 0.0
                    else:
                        _cap = 0.4 if recover and abs(_ey) > 6.0 else 0.15
                        step_yaw = max(-_cap, min(_cap, 0.2 * _ey))
                        _move = min(0.04 if recover else 0.02, _ex)
                        scale = _move / _ex if _ex > 1e-4 else 0.0
                    base_x, base_y, base_yaw = exp_x, exp_y, exp_yaw
'''

FUSE_NEW = '''                    # DIMOS_RELOC_TRACK_SCAN_V1 — trust a sharp wall match; otherwise stay slow.
                    _ey = lock_yaw if recover else err_yaw
                    _ex = lock_xy if recover else err_xy
                    _fit = 0.0
                    _rel = getattr(self, "_relocalizer", None)
                    if _rel is not None and getattr(_rel, "_last_fitness", None) is not None:
                        _fit = float(_rel._last_fitness)
                    _sign = 1 if _ey > 2.0 else (-1 if _ey < -2.0 else 0)
                    _agree = list(getattr(self, "_slow_agree", []))
                    _agree.append(_sign)
                    self._slow_agree = _agree[-3:]
                    _same = (
                        len(self._slow_agree) >= 3
                        and len(set(self._slow_agree)) == 1
                        and self._slow_agree[0] != 0
                    )
                    _sharp = _fit >= float(os.environ.get("DIMOS_RELOC_TRACK_FIT_TRUST", "0.80"))
                    if _sharp and (abs(_ey) > 1.0 or _ex > 0.08):
                        _cap = 1.5 if recover else 1.0
                        step_yaw = max(-_cap, min(_cap, 0.45 * _ey))
                        _move = min(0.12 if recover else 0.08, 0.5 * _ex)
                        scale = _move / _ex if _ex > 1e-4 else 0.0
                    elif not _same and _ex < 0.20:
                        step_yaw = 0.0
                        scale = 0.0
                    else:
                        _cap = 0.4 if recover and abs(_ey) > 6.0 else 0.25
                        step_yaw = max(-_cap, min(_cap, 0.25 * _ey))
                        _move = min(0.05 if recover else 0.03, _ex)
                        scale = _move / _ex if _ex > 1e-4 else 0.0
                    base_x, base_y, base_yaw = exp_x, exp_y, exp_yaw
'''


def edit(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{path.name}: anchor found {count} times: {old[:70]!r}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def main() -> int:
    site = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    mod = site / "dimos/mapping/relocalization/lidar/module.py"
    rel = site / "dimos/mapping/relocalization/lidar/relocalize.py"
    if MARKER in mod.read_text(encoding="utf-8") and MARKER in rel.read_text(encoding="utf-8"):
        # An earlier version wrote truncated names (icp_dist_f); every relocalize() raised.
        text = rel.read_text(encoding="utf-8")
        fixed = re.sub(r"\bicp_dist_f\b", "icp_dist_factor", text)
        fixed = re.sub(r"\bcoarse_dist_f\b", "coarse_dist_factor", fixed)
        if fixed != text:
            rel.write_text(fixed, encoding="utf-8")
            print(f"repaired truncated attribute names in {rel.name}")
            return 0
        print(f"already patched: {MARKER}")
        return 0
    edit(mod, WINDOW_OLD, WINDOW_NEW)
    edit(mod, CROP_OLD, CROP_NEW)
    edit(mod, FUSE_OLD, FUSE_NEW)
    edit(rel, ALIGN_OLD, ALIGN_NEW)
    edit(rel, SEED_YAW_OLD, SEED_YAW_NEW)
    edit(rel, SEED_RETURN_OLD, SEED_RETURN_NEW)
    print(f"patched module.py + relocalize.py -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
