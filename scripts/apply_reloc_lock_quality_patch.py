#!/usr/bin/env python3
"""Lock the stronger initial match, not the first cluster that repeats.

Two poses can both pass the hint window. The weaker one was repeating and
became the lock, so the live cloud sat off the map.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_LOCK_QUALITY_V1"

ATTEMPT_OLD = '''        result = self.align(local_map)
        logger.info(f"align: fitness={result.fitness:.3f} rmse={result.inlier_rmse:.3f}")
'''

ATTEMPT_NEW = '''        result = self.align(local_map)
        self._last_fitness = float(result.fitness)
        self._last_rmse = float(result.inlier_rmse)
        logger.info(f"align: fitness={result.fitness:.3f} rmse={result.inlier_rmse:.3f}")
'''

SETTLE_OLD = '''        n_need = max(1, int(os.environ.get("DIMOS_RELOC_SETTLE_N", "3")))
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
'''

SETTLE_NEW = '''        n_need = max(1, int(os.environ.get("DIMOS_RELOC_SETTLE_N", "3")))
        if tracking:
            n_need = max(1, int(os.environ.get("DIMOS_RELOC_TRACK_N", "1")))
        max_xy = float(os.environ.get("DIMOS_RELOC_SETTLE_M", "0.5"))
        max_yaw = float(os.environ.get("DIMOS_RELOC_SETTLE_YAW_DEG", "15"))
        fit_now = 0.0
        rel = getattr(self, "_relocalizer", None)
        if rel is not None and getattr(rel, "_last_fitness", None) is not None:
            fit_now = float(rel._last_fitness)

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

        if not tracking:
            # DIMOS_RELOC_LOCK_QUALITY_V1 — only the stronger cluster may lock.
            window = list(getattr(self, "_lock_window", []))
            window.append((tf, fit_now))
            self._lock_window = window[-8:]
            fits = sorted(item[1] for item in self._lock_window)
            best_fit = fits[-1]
            mid_fit = fits[len(fits) // 2]
            margin = max(0.02, 0.5 * (best_fit - mid_fit))
            strong = [item[0] for item in self._lock_window if item[1] >= best_fit - margin]
            logger.info(
                f"relocalize lock band {len(strong)}/{len(self._lock_window)} "
                f"fit={fit_now:.3f} best={best_fit:.3f} margin={margin:.3f} "
                f"yaw={yaw_deg(tf):.1f}"
            )
            if len(strong) < n_need:
                return
            agreed = True
            for i, a in enumerate(strong):
                for b in strong[i + 1 :]:
                    dyaw, dist = fix_error(a, b)
                    if abs(dyaw) > max_yaw or dist > max_xy:
                        agreed = False
                        break
                if not agreed:
                    break
            if not agreed:
                logger.info(
                    f"relocalize lock: strong matches disagree "
                    f"yaw={yaw_deg(tf):.1f} t={tf.translation}"
                )
                return
            chosen = min(strong, key=score) if hinted is not None else strong[-1]
        else:
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
            chosen = min(self._settle_buf, key=score) if hinted is not None else tf
'''


def main() -> int:
    root = Path(__file__).resolve().parents[1] / "simulation_legacy/.venv/lib/python3.12/site-packages/dimos/mapping/relocalization/lidar"
    module = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "module.py"
    relocalize = root / "relocalize.py"
    if not module.is_file() or not relocalize.is_file():
        print("reloc sources not found", file=sys.stderr)
        return 1
    module_text = module.read_text(encoding="utf-8")
    rel_text = relocalize.read_text(encoding="utf-8")
    if MARKER in module_text and "_last_fitness" in rel_text:
        print(f"already patched: {module}")
        return 0
    if "_last_fitness" not in rel_text:
        if ATTEMPT_OLD not in rel_text:
            print("align log not found", file=sys.stderr)
            return 1
        relocalize.write_text(rel_text.replace(ATTEMPT_OLD, ATTEMPT_NEW, 1), encoding="utf-8")
    if MARKER not in module_text:
        if SETTLE_OLD not in module_text:
            print("settle block not found", file=sys.stderr)
            return 1
        module.write_text(module_text.replace(SETTLE_OLD, SETTLE_NEW, 1), encoding="utf-8")
    print(f"patched {module}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
