#!/usr/bin/env python3
"""Record lidar frames, odometry and the published world<-map during a patrol.

Offline tests compare matchers on the same data. Every Nth frame goes to
DIMOS_RELOC_RECORD_DIR as frame_<n>.npz with pts (world frame), odom
(x, y, yaw), wm (published 4x4 world<-map, or zeros) and t (unix time).
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_RECORD_V1"

OLD = """        self.register_disposable(self.odom.observable().subscribe(self._on_odom))

    def clouds(self) -> Observable[PointCloud2]:
        return window(
"""

NEW = """        self.register_disposable(self.odom.observable().subscribe(self._on_odom))
        # DIMOS_RELOC_RECORD_V1 — raw frames for offline matcher tests.
        import os

        if os.environ.get("DIMOS_RELOC_RECORD_DIR", "").strip():
            self.register_disposable(self.lidar.observable().subscribe(self._record_frame))

    def _record_frame(self, msg) -> None:
        import os
        import time

        import numpy as np

        self._rec_n = getattr(self, "_rec_n", 0) + 1
        every = max(1, int(os.environ.get("DIMOS_RELOC_RECORD_EVERY", "5")))
        if self._rec_n % every:
            return
        odom = getattr(self, "_latest_odom", None)
        if odom is None:
            return
        try:
            pts = np.asarray(msg.points_f32(), dtype=np.float32)[:, :3]
            last = getattr(self, "_settle_last", None)
            wm = np.asarray(last.to_matrix(), dtype=np.float64) if last is not None else np.zeros((4, 4))
            folder = os.environ["DIMOS_RELOC_RECORD_DIR"].strip()
            os.makedirs(folder, exist_ok=True)
            np.savez(
                os.path.join(folder, f"frame_{self._rec_n:06d}.npz"),
                pts=pts,
                odom=np.array([float(odom.x), float(odom.y), float(odom.yaw)]),
                wm=wm,
                t=np.array([time.time()]),
            )
        except Exception:
            if not getattr(self, "_rec_err", False):
                self._rec_err = True
                logger.exception("relocalize record failed")

    def clouds(self) -> Observable[PointCloud2]:
        return window(
"""


def main() -> int:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/mapping/relocalization/lidar/module.py"
    )
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if text.count(OLD) != 1:
        print(f"anchor found {text.count(OLD)} times", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
