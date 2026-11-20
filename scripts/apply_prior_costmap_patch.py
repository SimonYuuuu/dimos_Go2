#!/usr/bin/env python3
"""Keep the costmap on the prior map.

Live scans were merged into global_map. After the pose drifted, the
occupancy grid grew to cover both the prior room and the skewed cloud.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_PRIOR_COSTMAP_V1"

OLD = '''    def global_map(self) -> PointCloud2:
        """The map in the robot-anchored world frame."""
        pts = self._base.pts
        if self.pgo.n_keyframes:
            newest = self._placed(self.pgo.n_keyframes - 1)
            pts = np.concatenate([self._base.pts[self._base.outside(newest.cols)], newest.pts])
'''

NEW = '''    def global_map(self) -> PointCloud2:
        """The map in the robot-anchored world frame."""
        # DIMOS_PRIOR_COSTMAP_V1 — the costmap is the prior room, not this run's scans.
        if self._has_premap():
            n_premap = len(self._premap)
            owners = [
                (scan, i, pose)
                for scan, i, pose in zip(
                    self._premap.scans, range(n_premap), self.pgo.premap_poses(), strict=True
                )
                if len(scan)
            ]
            pts = self._render(owners).pts if owners else np.zeros((0, 3), np.float32)
        else:
            pts = self._base.pts
            if self.pgo.n_keyframes:
                newest = self._placed(self.pgo.n_keyframes - 1)
                pts = np.concatenate(
                    [self._base.pts[self._base.outside(newest.cols)], newest.pts]
                )
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/navigation/go2/loop_closure/pgo_map.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if OLD not in text:
        print("global_map body not found", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
