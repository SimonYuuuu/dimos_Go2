#!/usr/bin/env python3
"""Adaptive wall tracking on a likelihood field of the loaded map.

Installed into site-packages as dimos/mapping/relocalization/lidar/field_track.py by
apply_reloc_field_track_patch.py. After the first lock this replaces the ICP tracker.

The live cloud is already in the odometry world, so world<-map is found by sliding
the cloud's wall band over the map's wall band. The search window and the share of
each measured correction that is applied both follow the pose uncertainty, which
grows with the distance walked and the angle turned and shrinks on a sharp match.
Faster walking therefore gets a wider window and a stronger correction by itself.

Run as a script to replay recorded frames:
    reloc_field_track.py <frames dir> <map.pc2.lcm> [--kick-at S] [--drift-deg-s R]
"""

from __future__ import annotations

import math
import os
import time

import numpy as np

RES = 0.05  # field cell, meters
BAND_LO, BAND_HI = 0.25, 1.2  # wall band above the ground, meters
SCORE_MIN = 0.45  # below this the walls are not recognised; odometry carries the pose
EARN_FAR = 0.01  # score a move beyond 0.10 m / 1.5 deg must gain over the current pose
EARN_WIDEN = 0.02  # score gain at the window edge that widens the next search
STEP_XY, STEP_YAW = 0.15, 3.0  # most the published pose moves in one update (m, deg)
DEAD_XY, DEAD_YAW = 0.01, 0.15  # smaller corrections are not published
SIG_XY_MIN, SIG_XY_MAX = 0.03, 0.35
SIG_YAW_MIN, SIG_YAW_MAX = 0.4, 5.0
DRIFT_XY_PER_M = 0.03  # odometry position drift, meters per meter walked
DRIFT_YAW_PER_M = 0.5  # odometry heading drift, degrees per meter walked
DRIFT_YAW_PER_DEG = 0.02  # odometry heading drift, degrees per degree turned


def _rot(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def wall_band(xyz: np.ndarray) -> np.ndarray:
    """XY of the points between shin and chest height, one per field cell."""
    if len(xyz) < 50:
        return np.empty((0, 2))
    ground = float(np.percentile(xyz[:, 2], 10))
    band = xyz[(xyz[:, 2] > ground + BAND_LO) & (xyz[:, 2] <= ground + BAND_HI)][:, :2]
    if not len(band):
        return np.empty((0, 2))
    cells = np.unique(np.floor(band / RES).astype(np.int64), axis=0)
    return (cells + 0.5) * RES


class Field:
    """exp(-d^2 / 2 sigma^2) of the distance to the nearest map wall cell."""

    def __init__(self, map_xyz: np.ndarray) -> None:
        from scipy import ndimage

        ground = float(np.percentile(map_xyz[:, 2], 10))
        keep = (map_xyz[:, 2] > ground + BAND_LO) & (map_xyz[:, 2] <= ground + BAND_HI)
        wall = np.asarray(map_xyz[keep][:, :2], dtype=np.float64)
        self.lo = wall.min(axis=0) - 2.0
        self.shape = tuple(int(v) for v in np.ceil((wall.max(axis=0) + 2.0 - self.lo) / RES))
        count = np.zeros(self.shape, dtype=np.int32)
        ij = np.floor((wall - self.lo) / RES).astype(np.int64)
        np.add.at(count, (ij[:, 0], ij[:, 1]), 1)
        dist = ndimage.distance_transform_edt(count < 2) * RES
        self.fine = self._field(dist, 0.07)
        self.coarse = self._field(dist, 0.20)

    @staticmethod
    def _field(dist: np.ndarray, sigma: float) -> np.ndarray:
        out = np.exp(-0.5 * (dist / sigma) ** 2).astype(np.float32)
        out[0, :] = out[-1, :] = out[:, 0] = out[:, -1] = 0.0  # points off the map score zero
        return out.ravel()

    def scores(
        self,
        flat: np.ndarray,
        pts: np.ndarray,
        about: np.ndarray,
        yaws: np.ndarray,
        center: np.ndarray,
        radius: int,
        step: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Mean field value for every (yaw, cell offset) around `center`.

        `pts` are in the map frame, rotated about `about`. Returns the score table
        and the offsets in cells, one row per table column.
        """
        height, width = self.shape
        span = np.arange(-radius, radius + 1, step)
        oi, oj = (g.ravel() for g in np.meshgrid(span, span, indexing="ij"))
        rel = pts - about
        table = np.empty((len(yaws), len(oi)), dtype=np.float32)
        for row, yaw in enumerate(yaws):
            moved = rel @ _rot(float(yaw)).T + about + center
            cell = np.floor((moved - self.lo) / RES).astype(np.int64)
            i = np.clip(cell[None, :, 0] + oi[:, None], 0, height - 1)
            j = np.clip(cell[None, :, 1] + oj[:, None], 0, width - 1)
            table[row] = flat[i * width + j].mean(axis=1)
        return table, np.stack([oi, oj], axis=1)


class Result:
    __slots__ = ("kind", "wm", "score", "seed", "dyaw", "dxy", "need_yaw", "need_xy", "win")

    def __init__(self, kind: str) -> None:
        self.kind = kind  # step | hold | lost | gate | thin
        self.wm = None
        self.score = self.seed = 0.0
        self.dyaw = self.dxy = self.need_yaw = self.need_xy = 0.0
        self.win = (0.0, 0.0)


class Tracker:
    def __init__(self, map_xyz: np.ndarray) -> None:
        self.field = Field(np.asarray(map_xyz, dtype=np.float32)[:, :3])
        self.cov_xy = np.eye(2) * 0.25**2  # first lock is a typed pose
        self.sig_yaw = 5.0
        self.odom = None
        self.travel = 0.0
        self.start_pose = None  # where the first lock put the dog on the map

    def trust(self, sig_xy: float = 0.05, sig_yaw: float = 1.0) -> None:
        """The operator moved the pose by hand: search close to it."""
        self.cov_xy = np.eye(2) * sig_xy**2
        self.sig_yaw = sig_yaw
        self.start_pose = None  # the departure gate is measured from the moved pose

    def _walked(self, odom: tuple[float, float, float]) -> None:
        if self.odom is not None:
            dist = math.hypot(odom[0] - self.odom[0], odom[1] - self.odom[1])
            turn = abs(math.degrees((odom[2] - self.odom[2] + math.pi) % (2 * math.pi) - math.pi))
            self.travel += dist
            lever = math.radians(self.sig_yaw) * dist  # a heading error swings the position
            self.cov_xy = self.cov_xy + np.eye(2) * ((DRIFT_XY_PER_M * dist) ** 2 + lever**2)
            self.sig_yaw = math.hypot(
                self.sig_yaw, DRIFT_YAW_PER_M * dist + DRIFT_YAW_PER_DEG * turn
            )
        self.odom = odom
        self._bound()

    def _bound(self) -> None:
        vals, vecs = np.linalg.eigh(self.cov_xy)
        vals = np.clip(vals, SIG_XY_MIN**2, SIG_XY_MAX**2)
        self.cov_xy = (vecs * vals) @ vecs.T
        self.sig_yaw = min(max(self.sig_yaw, SIG_YAW_MIN), SIG_YAW_MAX)

    def update(
        self,
        cloud_world: np.ndarray,
        wm: np.ndarray,
        odom: tuple[float, float, float],
        gate: tuple[float, float] | None = None,
    ) -> Result:
        """One measurement. `wm` is the published world<-map; the new one is in the result.

        `gate` (meters, degrees) bounds how far the dog may be moved from the first
        lock before it has walked half a meter.
        """
        self._walked(odom)
        near = np.hypot(cloud_world[:, 0] - odom[0], cloud_world[:, 1] - odom[1]) < 8.0
        band_world = wall_band(cloud_world[near])
        if len(band_world) < 150:
            return Result("thin")
        mw = np.linalg.inv(wm)
        band = band_world @ mw[:2, :2].T + mw[:2, 3]
        dog = (mw @ np.array([odom[0], odom[1], 0.0, 1.0]))[:2]
        dog_yaw = math.degrees(odom[2] + math.atan2(mw[1, 0], mw[0, 0]))
        if self.start_pose is None:
            self.start_pose = (float(dog[0]), float(dog[1]), dog_yaw)

        sig_xy = float(np.sqrt(np.linalg.eigvalsh(self.cov_xy)[-1]))
        win_xy = min(max(3.0 * sig_xy, 0.10), 1.0)
        win_yaw = min(max(3.0 * self.sig_yaw, 1.5), 15.0)
        field = self.field
        center = np.zeros(2)
        yaw0 = 0.0
        if win_xy > 0.16 or win_yaw > 2.0:
            few = band[:: max(1, len(band) // 500)]
            yaws = np.linspace(-win_yaw, win_yaw, 2 * max(2, math.ceil(win_yaw / 1.5)) + 1)
            radius = max(2, 2 * math.ceil(win_xy / (2 * RES)))
            table, offs = field.scores(field.coarse, few, dog, yaws, center, radius, 2)
            # Among near-equal peaks take the one nearest the current pose.
            far = (np.hypot(offs[:, 0], offs[:, 1]) / max(radius, 1)) ** 2
            turn = (np.abs(yaws) / max(win_yaw, 1e-6)) ** 2
            row, col = np.unravel_index(
                int(np.argmax(table - 0.03 * far[None, :] - 0.03 * turn[:, None])), table.shape
            )
            center = offs[col] * RES
            yaw0 = float(yaws[row])
        some = band[:: max(1, len(band) // 900)]
        # A one-sided view leaves the heading on a plateau several degrees wide. Looking
        # across the whole plateau puts the estimate at its middle and keeps the window open.
        half = min(max(win_yaw, 1.5), 6.0)
        yaws = yaw0 + np.arange(-half, half + 1e-6, 0.25 if half <= 1.5 else 0.5)
        table, offs = field.scores(field.fine, some, dog, yaws, center, 3, 1)
        row, col = np.unravel_index(int(np.argmax(table)), table.shape)
        best = float(table[row, col])
        seed = float(field.scores(field.fine, some, dog, np.zeros(1), np.zeros(2), 0, 1)[0][0, 0])

        out = Result("hold")
        out.score, out.seed, out.win = best, seed, (win_xy, win_yaw)
        if best < SCORE_MIN:
            # Not on the map's walls anywhere in the window: widen it and let odometry carry on.
            self.cov_xy = self.cov_xy * 2.0
            self.sig_yaw *= 1.5
            self._bound()
            out.kind = "lost"
            return out

        # Peak position: score-weighted mean of the cells close to the best score.
        weight = np.exp((table - best) / 0.02)
        weight /= weight.sum()
        w_yaw, w_off = weight.sum(axis=1), weight.sum(axis=0)
        need_yaw = float(w_yaw @ yaws)
        need_xy = center + (w_off @ offs) * RES
        dxy = offs * RES + center - need_xy
        noise_xy = (dxy * w_off[:, None]).T @ dxy + np.eye(2) * 0.02**2
        noise_yaw = float(w_yaw @ (yaws - need_yaw) ** 2) + 0.2**2
        blur = min(max((best - SCORE_MIN) / 0.30, 0.1), 1.0) ** 2  # a weak match counts less
        noise_xy, noise_yaw = noise_xy / blur, noise_yaw / blur
        out.need_yaw, out.need_xy = need_yaw, float(np.hypot(*need_xy))

        # A far move has to earn its score. Along a bare corridor, or where the room no
        # longer looks like the map, many poses score about the same and odometry is better.
        earned = best - seed
        if (out.need_xy > 0.10 or abs(need_yaw) > 1.5) and earned < EARN_FAR:
            return out

        gain_xy = self.cov_xy @ np.linalg.inv(self.cov_xy + noise_xy)
        gain_yaw = self.sig_yaw**2 / (self.sig_yaw**2 + noise_yaw)
        edge = (
            row in (0, len(yaws) - 1)
            or abs(int(offs[col, 0])) == 3
            or abs(int(offs[col, 1])) == 3
        )
        if edge and earned >= EARN_WIDEN:
            # A clearly better peak at the window edge: the pose is further off than assumed.
            self.cov_xy = self.cov_xy * 4.0
            self.sig_yaw *= 2.0
        else:
            self.cov_xy = (np.eye(2) - gain_xy) @ self.cov_xy
            self.cov_xy = 0.5 * (self.cov_xy + self.cov_xy.T)
            self.sig_yaw *= math.sqrt(1.0 - gain_yaw)
        self._bound()

        step_xy = gain_xy @ need_xy
        step_yaw = gain_yaw * need_yaw
        size = float(np.hypot(*step_xy))
        if size > STEP_XY:
            step_xy = step_xy * (STEP_XY / size)
        step_yaw = min(max(step_yaw, -STEP_YAW), STEP_YAW)
        out.dyaw, out.dxy = float(step_yaw), float(np.hypot(*step_xy))
        if out.dxy < DEAD_XY and abs(step_yaw) < DEAD_YAW:
            return out
        if gate is not None and self.travel < 0.5:
            sx, sy, syaw = self.start_pose
            moved = math.hypot(dog[0] + step_xy[0] - sx, dog[1] + step_xy[1] - sy)
            turned = abs((dog_yaw + step_yaw - syaw + 180.0) % 360.0 - 180.0)
            if moved > gate[0] or turned > gate[1]:
                out.kind = "gate"
                return out

        # Scan in map: q' = R (q - dog) + dog + step. world<-map takes the inverse.
        fix = np.eye(4)
        fix[:2, :2] = _rot(step_yaw)
        fix[:2, 3] = dog + step_xy - fix[:2, :2] @ dog
        out.wm = wm @ np.linalg.inv(fix)
        out.kind = "step"
        return out


def track_step(mod, msg, logger) -> None:
    """Called from CloudRelocalization._relocalize once a first lock exists."""
    from dimos.msgs.geometry_msgs.Transform import Transform

    odom_msg = getattr(mod, "_latest_odom", None)
    last = getattr(mod, "_settle_last", None)
    if odom_msg is None or last is None:
        return
    odom = (float(odom_msg.x), float(odom_msg.y), float(odom_msg.yaw))
    tracker = getattr(mod, "_field_tracker", None)
    if tracker is None:
        t0 = time.monotonic()
        tracker = Tracker(np.asarray(mod.premap.points_f32(), dtype=np.float32)[:, :3])
        mod._field_tracker = tracker
        logger.info(
            f"relocalize field: map wall field {tracker.field.shape[0]}x{tracker.field.shape[1]} "
            f"cells of {RES}m built in {time.monotonic() - t0:.2f}s"
        )
    elif last is not getattr(mod, "_field_last", None):
        tracker.trust()  # nudged by the operator
    gate = (
        float(os.environ.get("DIMOS_RELOC_HINT_XY_M", "0.70")),
        float(os.environ.get("DIMOS_RELOC_HINT_YAW_DEG", "20")),
    )
    t0 = time.monotonic()
    cloud = np.asarray(msg.points_f32(), dtype=np.float32)[:, :3]
    res = tracker.update(cloud, np.asarray(last.to_matrix(), dtype=np.float64), odom, gate)
    cost = time.monotonic() - t0
    note = (
        f"score={res.score:.2f} at_pose={res.seed:.2f} "
        f"need_dyaw={res.need_yaw:+.1f} need_dxy={res.need_xy:.2f}m "
        f"window={res.win[0]:.2f}m/{res.win[1]:.1f}deg cost={cost * 1000:.0f}ms"
    )
    if res.kind != "step":
        now = time.monotonic()
        quiet = res.kind == "hold" and getattr(mod, "_field_kind", "") == "hold"
        if not quiet or now - getattr(mod, "_field_log_at", 0.0) > 5.0:
            mod._field_log_at = now
            logger.info(f"relocalize field: {res.kind} {note}")
        mod._field_kind = res.kind
        mod._field_last = last
        return
    if getattr(mod, "_settle_last", None) is not last:
        return  # a nudge landed while matching; it wins
    tf = Transform.from_matrix(
        res.wm, frame_id=mod.config.world_frame, child_frame_id=mod.config.map_frame
    )
    mod._settle_last = tf
    mod._field_last = tf
    mod._field_kind = "step"
    if getattr(mod, "_settle_anchor", None) is None:
        mod._settle_anchor = tf
    logger.info(f"relocalize field: step dyaw={res.dyaw:+.2f} dxy={res.dxy:.3f}m {note}")
    mod.submit(tf, "lidar")
    yaw = math.atan2(float(res.wm[1, 0]), float(res.wm[0, 0]))
    os.environ["DIMOS_RELOC_SEED_POSE"] = (
        f"{float(res.wm[0, 3]):.4f},{float(res.wm[1, 3]):.4f},{yaw:.5f}"
    )


def _replay(argv: list[str]) -> int:
    import argparse
    import glob

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    ap = argparse.ArgumentParser(description="Replay recorded frames through the tracker.")
    ap.add_argument("frames")
    ap.add_argument("map")
    ap.add_argument("--kick-at", type=float, default=-1.0, help="second at which to knock the pose")
    ap.add_argument("--kick-deg", type=float, default=8.0)
    ap.add_argument("--kick-m", type=float, default=0.5)
    ap.add_argument("--drift-deg-s", type=float, default=0.0, help="extra odometry heading drift")
    ap.add_argument("--start-deg", type=float, default=0.0, help="error added to the first pose")
    ap.add_argument("--start-m", type=float, default=0.0)
    ap.add_argument("--show", type=float, nargs=2, help="print every frame between two seconds")
    args = ap.parse_args(argv)

    with open(args.map, "rb") as fh:
        premap = PointCloud2.lcm_decode(fh.read())
    tracker = Tracker(np.asarray(premap.points_f32(), dtype=np.float32)[:, :3])
    wm = None
    t_first = None
    kicked = False
    rows = []
    for path in sorted(glob.glob(os.path.join(args.frames, "frame_*.npz"))):
        frame = np.load(path)
        if not np.any(frame["wm"]):
            continue
        t = float(frame["t"][0])
        t_first = t if t_first is None else t_first
        pts = np.asarray(frame["pts"], dtype=np.float64)
        ox, oy, oyaw = (float(v) for v in frame["odom"])
        if args.drift_deg_s:
            # Turn the whole odometry world about its origin, as a drifting heading does.
            spin = _rot(args.drift_deg_s * (t - t_first))
            pts[:, :2] = pts[:, :2] @ spin.T
            ox, oy = spin @ np.array([ox, oy])
            oyaw += math.radians(args.drift_deg_s * (t - t_first))
        if wm is None:
            wm = np.array(frame["wm"], dtype=np.float64)
            if args.start_deg or args.start_m:
                bump = np.eye(4)
                bump[:2, :2] = _rot(args.start_deg)
                bump[0, 3] = args.start_m
                wm = wm @ bump
        if args.kick_at >= 0 and not kicked and t - t_first >= args.kick_at:
            kicked = True
            bump = np.eye(4)
            bump[:2, :2] = _rot(args.kick_deg)
            bump[0, 3] = args.kick_m
            wm = wm @ bump
        t0 = time.monotonic()
        res = tracker.update(pts, wm, (ox, oy, oyaw))
        cost = time.monotonic() - t0
        if res.wm is not None:
            wm = res.wm
        if args.show and args.show[0] <= t - t_first <= args.show[1]:
            print(
                f"{t - t_first:6.1f} {res.kind:5s} at_pose={res.seed:.2f} score={res.score:.2f} "
                f"need={res.need_yaw:+.2f}deg/{res.need_xy:.2f}m step={res.dyaw:+.2f}deg/{res.dxy:.3f}m "
                f"window={res.win[0]:.2f}m/{res.win[1]:.1f}deg travel={tracker.travel:.1f}m"
            )
        rows.append(
            (
                t - t_first,
                res.seed,
                res.score,
                math.degrees(math.atan2(wm[1, 0], wm[0, 0])),
                res.need_yaw,
                res.need_xy,
                cost,
                res.kind == "lost",
                res.win[0],
                res.win[1],
            )
        )
    table = np.array(rows)
    print("t(s)  at_pose med/min   wm_yaw   need_dyaw max  need_dxy max  lost  window max    ms max")
    for start in range(0, int(table[-1, 0]) + 1, 20):
        part = table[(table[:, 0] >= start) & (table[:, 0] < start + 20)]
        if not len(part):
            continue
        print(
            f"{start:4d}    {np.median(part[:, 1]):.2f} / {part[:, 1].min():.2f}   "
            f"{np.median(part[:, 3]):7.1f}     {np.abs(part[:, 4]).max():5.2f}        "
            f"{part[:, 5].max():.2f}       {int(part[:, 7].sum()):2d}   "
            f"{part[:, 8].max():.2f}m/{part[:, 9].max():4.1f}deg  {part[:, 6].max() * 1000:5.0f}"
        )
    print(
        f"frames={len(table)} at_pose median={np.median(table[:, 1]):.2f} "
        f"below 0.70: {int((table[:, 1] < 0.70).sum())} lost: {int(table[:, 7].sum())} "
        f"cost median={np.median(table[:, 6]) * 1000:.0f}ms"
    )
    return 0


if __name__ == "__main__":
    import sys

    raise SystemExit(_replay(sys.argv[1:]))
