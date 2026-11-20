#!/usr/bin/env python3
"""Pause on a live lidar obstacle that sits on the path ahead.

The costmap only knows the prior map. This watches the current scan, stops
the gait while points between the floor and 1 m above it block the next
1.2 m of the path, and continues the same goal once those points leave.
It does not cancel the goal or request a replan.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_NAV_LIVE_CLOUD_V1"
MARKER_V2 = "DIMOS_NAV_LIVE_CLOUD_V2"

OLD_SEND = """            if cmd_vel is not None:
                self.cmd_vel.on_next(cmd_vel)
"""

NEW_SEND = """            if cmd_vel is not None:
                # DIMOS_NAV_LIVE_CLOUD_V2 — brake only while walking, past the body.
                if state == "path_following" and self._live_path_blocked():
                    cmd_vel = Twist()
                self.cmd_vel.on_next(cmd_vel)
"""

OLD_METHOD = """    def _yaw_only_ignored(self, odom_yaw: float, commanding: bool) -> bool:
"""

NEW_METHOD = '''    def note_lidar(self, msg: object) -> None:
        """DIMOS_NAV_LIVE_CLOUD_V1 — keep the newest scan for the path pause."""
        try:
            pts = msg.points_f32()  # type: ignore[attr-defined]
        except Exception:
            return
        if pts is None or len(pts) < 30:
            return
        with self._lock:
            self._live_pts = pts
            self._live_recv = time.monotonic()
            if not getattr(self, "_live_seen", False):
                self._live_seen = True
                logger.info("Live cloud pause armed (%d points).", len(pts))

    def _live_path_blocked(self) -> bool:
        """True when the scan shows a body-height obstacle on the path ahead."""
        try:
            blocked = self._live_corridor_blocked()
        except Exception:
            if not getattr(self, "_live_err", False):
                self._live_err = True
                logger.exception("Live cloud pause failed; map planner stays in charge.")
            return False
        now = time.monotonic()
        if blocked:
            if getattr(self, "_live_block_since", None) is None:
                self._live_block_since = now
            braking = now - self._live_block_since >= 0.25
        else:
            self._live_block_since = None
            braking = False
        was = getattr(self, "_live_holding", False)
        if braking and not was:
            logger.info(
                "Point cloud blocks the path ahead (%.2fm, %d pts). Pausing until it clears.",
                float(getattr(self, "_live_hit_m", -1.0)),
                int(getattr(self, "_live_hit_n", 0)),
            )
        elif was and not braking:
            logger.info("Path ahead is clear. Continuing the task.")
        self._live_holding = braking
        return braking

    def _live_corridor_blocked(self) -> bool:
        with self._lock:
            pts = getattr(self, "_live_pts", None)
            recv = float(getattr(self, "_live_recv", 0.0))
            odom = self._current_odom
            path = self._path
            idx = int(getattr(self, "_pose_index", 0))
        if pts is None or odom is None or path is None or not path.poses:
            return False
        if time.monotonic() - recv > 0.6:
            return False
        rx = float(odom.position.x)
        ry = float(odom.position.y)
        yaw = float(odom.orientation.euler.z)
        world = _scan_in_world(np.asarray(pts, dtype=np.float32), rx, ry, yaw)
        near = np.hypot(world[:, 0] - rx, world[:, 1] - ry)
        nearby = world[near < 1.6]
        near_n = near[near < 1.6]
        if len(nearby) < 30:
            return False
        # Floor returns sit under the robot, so they stay in the ground estimate.
        ground = float(np.percentile(nearby[:, 2], 15))
        # Ignore the body and the ground immediately in front of the feet.
        band = nearby[
            (near_n > 0.50)
            & (nearby[:, 2] > ground + 0.25)
            & (nearby[:, 2] <= ground + 1.0)
        ]
        self._live_hit_n = 0
        self._live_hit_m = -1.0
        if len(band) < 15:
            return False
        poly = _path_ahead(path, idx, rx, ry, 1.2)
        if poly is None:
            return False
        count, nearest = _polyline_stats(band[:, :2], poly, radius=0.25, min_ahead=0.55)
        self._live_hit_n = count
        self._live_hit_m = nearest
        return count >= 15

    def _yaw_only_ignored(self, odom_yaw: float, commanding: bool) -> bool:
'''

HELPERS = '''
def _scan_in_world(pts: np.ndarray, rx: float, ry: float, yaw: float) -> np.ndarray:
    """DIMOS_NAV_LIVE_CLOUD_V1 — body scans are rotated into the odom frame."""
    if len(pts) == 0:
        return pts
    step = max(1, len(pts) // 400)
    sample = pts[::step]
    near_origin = float(np.mean(np.hypot(sample[:, 0], sample[:, 1]) < 1.0))
    near_robot = float(np.mean(np.hypot(sample[:, 0] - rx, sample[:, 1] - ry) < 1.0))
    if near_robot + 0.05 >= near_origin:
        return pts
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    out = np.empty_like(pts)
    out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + rx
    out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + ry
    out[:, 2] = pts[:, 2]
    return out


def _path_ahead(path: Path, idx: int, rx: float, ry: float, length: float) -> np.ndarray | None:
    poses = path.poses
    if not poses:
        return None
    idx = max(0, min(int(idx), len(poses) - 1))
    out = [np.array([rx, ry], dtype=np.float64)]
    walked = 0.0
    for pose in poses[idx:]:
        point = np.array([float(pose.position.x), float(pose.position.y)], dtype=np.float64)
        step = float(np.hypot(*(point - out[-1])))
        if step < 1e-3:
            continue
        if walked + step >= length:
            out.append(out[-1] + (point - out[-1]) * ((length - walked) / step))
            break
        out.append(point)
        walked += step
    if len(out) < 2:
        return None
    return np.stack(out)


def _polyline_hits(
    xy: np.ndarray,
    poly: np.ndarray,
    *,
    radius: float,
    min_ahead: float,
    min_count: int,
) -> bool:
    hit = np.zeros(len(xy), dtype=bool)
    origin = poly[0]
    for start, end in zip(poly[:-1], poly[1:], strict=False):
        edge = end - start
        length2 = float(edge[0] * edge[0] + edge[1] * edge[1])
        if length2 < 1e-8:
            continue
        rel = xy - start
        t = np.clip((rel[:, 0] * edge[0] + rel[:, 1] * edge[1]) / length2, 0.0, 1.0)
        proj = start + t[:, None] * edge
        dist = np.hypot(xy[:, 0] - proj[:, 0], xy[:, 1] - proj[:, 1])
        ahead = np.hypot(proj[:, 0] - origin[0], proj[:, 1] - origin[1])
        hit |= (dist < radius) & (ahead > min_ahead)
        if int(hit.sum()) >= min_count:
            return True
    return False


def _polyline_stats(
    xy: np.ndarray,
    poly: np.ndarray,
    *,
    radius: float,
    min_ahead: float,
) -> tuple[int, float]:
    """DIMOS_NAV_LIVE_CLOUD_V2 — count path hits and the nearest one ahead."""
    hit = np.zeros(len(xy), dtype=bool)
    nearest = 99.0
    origin = poly[0]
    for start, end in zip(poly[:-1], poly[1:], strict=False):
        edge = end - start
        length2 = float(edge[0] * edge[0] + edge[1] * edge[1])
        if length2 < 1e-8:
            continue
        rel = xy - start
        t = np.clip((rel[:, 0] * edge[0] + rel[:, 1] * edge[1]) / length2, 0.0, 1.0)
        proj = start + t[:, None] * edge
        dist = np.hypot(xy[:, 0] - proj[:, 0], xy[:, 1] - proj[:, 1])
        ahead = np.hypot(proj[:, 0] - origin[0], proj[:, 1] - origin[1])
        close = (dist < radius) & (ahead > min_ahead)
        hit |= close
        if bool(np.any(close)):
            nearest = min(nearest, float(np.min(ahead[close])))
    if nearest > 90.0:
        nearest = -1.0
    return int(hit.sum()), nearest


'''

OLD_PORT = """    odom: In[PoseStamped]  # TODO: Use TF.
"""

NEW_PORT = """    odom: In[PoseStamped]  # TODO: Use TF.
    lidar: In[PointCloud2]  # DIMOS_NAV_LIVE_CLOUD_V1
"""

OLD_SUB = """        self.register_disposable(Disposable(self.odom.subscribe(self._planner.handle_odom)))
"""

NEW_SUB = """        self.register_disposable(Disposable(self.odom.subscribe(self._planner.handle_odom)))
        # DIMOS_NAV_LIVE_CLOUD_V1
        if self.lidar.transport is not None:
            self.register_disposable(
                Disposable(self.lidar.subscribe(self._planner._local_planner.note_lidar))
            )
"""

OLD_IMPORT = """from dimos.msgs.nav_msgs.Path import Path
"""

NEW_IMPORT = """from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
"""


def _nav_root() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/replanning_a_star"
    )


def _patch(path: Path, pairs: list[tuple[str, str]]) -> None:
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path}")
        return
    for old, _new in pairs:
        if old not in text:
            raise SystemExit(f"anchor missing in {path}: {old[:80]!r}")
    for old, new in pairs:
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path}")


V1_SEND = """            if cmd_vel is not None:
                # DIMOS_NAV_LIVE_CLOUD_V1 — brake on the live scan, keep the goal.
                if self._live_path_blocked():
                    cmd_vel = Twist()
                self.cmd_vel.on_next(cmd_vel)
"""

V1_HOLD = """        was = getattr(self, "_live_holding", False)
        if blocked and not was:
            logger.info("Point cloud blocks the path ahead. Pausing until it clears.")
        elif was and not blocked:
            logger.info("Path ahead is clear. Continuing the task.")
        self._live_holding = blocked
        return blocked
"""

V2_HOLD = """        now = time.monotonic()
        if blocked:
            if getattr(self, "_live_block_since", None) is None:
                self._live_block_since = now
            braking = now - self._live_block_since >= 0.25
        else:
            self._live_block_since = None
            braking = False
        was = getattr(self, "_live_holding", False)
        if braking and not was:
            logger.info(
                "Point cloud blocks the path ahead (%.2fm, %d pts). Pausing until it clears.",
                float(getattr(self, "_live_hit_m", -1.0)),
                int(getattr(self, "_live_hit_n", 0)),
            )
        elif was and not braking:
            logger.info("Path ahead is clear. Continuing the task.")
        self._live_holding = braking
        return braking
"""

V1_BAND = """        band = nearby[
            (near_n > 0.28)
            & (nearby[:, 2] > ground + 0.18)
            & (nearby[:, 2] <= ground + 1.0)
        ]
        if len(band) < 6:
            return False
        poly = _path_ahead(path, idx, rx, ry, 1.2)
        if poly is None:
            return False
        return _polyline_hits(band[:, :2], poly, radius=0.30, min_ahead=0.20, min_count=6)
"""

V2_BAND = """        # DIMOS_NAV_LIVE_CLOUD_V2 — ignore the body and the ground at the feet.
        band = nearby[
            (near_n > 0.50)
            & (nearby[:, 2] > ground + 0.25)
            & (nearby[:, 2] <= ground + 1.0)
        ]
        self._live_hit_n = 0
        self._live_hit_m = -1.0
        if len(band) < 15:
            return False
        poly = _path_ahead(path, idx, rx, ry, 1.2)
        if poly is None:
            return False
        count, nearest = _polyline_stats(band[:, :2], poly, radius=0.25, min_ahead=0.55)
        self._live_hit_n = count
        self._live_hit_m = nearest
        return count >= 15
"""

OLD_STUCK = """                if state in ("initial_rotation", "final_rotation"):
                    last_stuck_check = time.perf_counter()
                    continue
"""

NEW_STUCK = """                if state in ("initial_rotation", "final_rotation") or getattr(
                    self._local_planner, "_live_holding", False
                ):  # DIMOS_NAV_LIVE_CLOUD_V2
                    last_stuck_check = time.perf_counter()
                    continue
"""


def _upgrade_v2(local: Path, planner: Path) -> None:
    text = local.read_text(encoding="utf-8")
    if MARKER_V2 not in text:
        for old in (V1_SEND, V1_HOLD, V1_BAND):
            if old not in text:
                raise SystemExit(f"v2 anchor missing in {local}: {old[:60]!r}")
        text = text.replace(V1_SEND, NEW_SEND, 1).replace(V1_HOLD, V2_HOLD, 1).replace(V1_BAND, V2_BAND, 1)
        if "def _polyline_stats" not in text:
            stats = HELPERS.split("def _polyline_stats", 1)[1]
            text = text.rstrip() + "\n\ndef _polyline_stats" + stats
        local.write_text(text, encoding="utf-8")
        print(f"upgraded {local}")
    else:
        print(f"already v2: {local}")

    planned = planner.read_text(encoding="utf-8")
    if "DIMOS_NAV_LIVE_CLOUD_V2" in planned:
        print(f"already v2: {planner}")
        return
    if OLD_STUCK not in planned:
        raise SystemExit(f"stuck anchor missing in {planner}")
    planner.write_text(planned.replace(OLD_STUCK, NEW_STUCK, 1), encoding="utf-8")
    print(f"upgraded {planner}")


def main() -> int:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else _nav_root()
    local = root / "local_planner.py"
    module = root / "module.py"
    planner = root / "global_planner.py"
    if not local.is_file() or not module.is_file() or not planner.is_file():
        print(f"not found: {root}", file=sys.stderr)
        return 1
    _patch(local, [(OLD_SEND, NEW_SEND), (OLD_METHOD, NEW_METHOD)])
    local_text = local.read_text(encoding="utf-8")
    if "def _scan_in_world" not in local_text:
        local.write_text(local_text.rstrip() + "\n" + HELPERS, encoding="utf-8")
    _patch(
        module,
        [(OLD_IMPORT, NEW_IMPORT), (OLD_PORT, NEW_PORT), (OLD_SUB, NEW_SUB)],
    )
    _upgrade_v2(local, planner)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
