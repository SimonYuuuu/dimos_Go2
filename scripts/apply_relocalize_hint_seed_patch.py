#!/usr/bin/env python3
"""Seed ICP refine with DIMOS_INITIAL_POSE (+ small yaw nudges).

Rectangular rooms often produce a high-fitness wrong yaw. Including the
operator hint (and nearby yaw hypotheses) as ICP starts biases toward the
correct basin.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_HINT_SEED_V5"
OLD_MARKERS = ("DIMOS_RELOC_HINT_SEED_V1",)

OLD_V1_TAIL = '''        # DIMOS_RELOC_HINT_SEED_V1 — also refine from operator initial pose.
        import math
        import os

        hint = os.environ.get("DIMOS_INITIAL_POSE", "").strip()
        if hint:
            try:
                parts = [float(x) for x in hint.split(",")]
                if len(parts) >= 3:
                    hx, hy, hyaw = parts[0], parts[1], parts[2]
                    # world->map hint; Open3D wants map<-world (= inverse).
                    c, s = math.cos(hyaw), math.sin(hyaw)
                    T_wm = np.eye(4)
                    T_wm[0, 0], T_wm[0, 1], T_wm[1, 0], T_wm[1, 1] = c, -s, s, c
                    T_wm[0, 3], T_wm[1, 3] = hx, hy
                    T_mw = np.linalg.inv(T_wm)
                    scored.append(refine(T_mw))
            except Exception:
                pass
        seed = os.environ.get("DIMOS_RELOC_SEED_POSE", "").strip()
        seed_scored = []
        if seed:
            try:
                parts = [float(x) for x in seed.split(",")]
                if len(parts) >= 3:
                    hx, hy, hyaw = parts[0], parts[1], parts[2]
                    for dyaw in (0.0, math.radians(5), math.radians(-5)):
                        yaw = hyaw + dyaw
                        c, s = math.cos(yaw), math.sin(yaw)
                        T_wm = np.eye(4)
                        T_wm[0, 0], T_wm[0, 1], T_wm[1, 0], T_wm[1, 1] = c, -s, s, c
                        T_wm[0, 3], T_wm[1, 3] = hx, hy
                        seed_scored.append(refine(np.linalg.inv(T_wm)))
            except Exception:
                pass
        # DIMOS_RELOC_HINT_SEED_V3 — once a lock exists, keep that basin.
        # A higher-fitness corridor flip must not replace a seed that already passes.
        if seed_scored:
            best_seed = max(seed_scored, key=lambda r: r.fitness)
            if best_seed.fitness >= cfg.fitness_threshold:
                return best_seed
        return max(scored + seed_scored, key=lambda r: r.fitness)
'''

NEW = '''        scored = [
            refine(np.asarray(hypothesis().transformation))
            for _ in range(max(cfg.ransac_restarts, 1))
        ]
        # DIMOS_RELOC_HINT_SEED_V2 — refine from hint and nearby yaw nudges.
        import math
        import os

        hint = os.environ.get("DIMOS_INITIAL_POSE", "").strip()
        if hint:
            try:
                parts = [float(x) for x in hint.split(",")]
                if len(parts) >= 3:
                    hx, hy, hyaw = parts[0], parts[1], parts[2]
                    nudges = [0.0, math.radians(10), math.radians(-10), math.radians(20), math.radians(-20)]
                    for d yaw in nudges:
                        yaw = hyaw + dyaw
                        c, s = math.cos(yaw), math.sin(yaw)
                        T_wm = np.eye(4)
                        T_wm[0, 0], T_wm[0, 1], T_wm[1, 0], T_wm[1, 1] = c, -s, s, c
                        T_wm[0, 3], T_wm[1, 3] = hx, hy
                        scored.append(refine(np.linalg.inv(T_wm)))
            except Exception:
                pass
        return max(scored, key=lambda r: r.fitness)
'''

# Fix typo in NEW - I accidentally wrote "d yaw"
NEW = NEW.replace("for d yaw in nudges:", "for dyaw in nudges:")


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/mapping/relocalization/lidar/relocalize.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if (
        "DIMOS_RELOC_MORNING_V5" in text
        or "DIMOS_RELOC_GRID_V1" in text
        or "DIMOS_RELOC_MATCH_BAND_V3" in text
        or "DIMOS_RELOC_MATCH_BAND_V2" in text
        or "DIMOS_RELOC_MATCH_BAND_V1" in text
    ):
        print(f"match band present, leave it: {target}")
        return 0
    if MARKER in text:
        old_nudge = "for _dyaw in (0.0, math.radians(5), math.radians(-5)):"
        new_nudge = "for _dyaw in (0.0,):  # DIMOS_RELOC_TRACK_YAW_V1"
        changed = False
        if old_nudge in text:
            text = text.replace(old_nudge, new_nudge, 1)
            changed = True
        slow = '''                        _local.append(refine(np.linalg.inv(_T)))
                    if _local:
                        _best = max(_local, key=lambda r: r.fitness)
                        if _best.fitness >= cfg.fitness_threshold:
                            return _best
'''
        fast = '''                        # DIMOS_RELOC_FAST_SEED_V1 — one narrow ICP, closest yaw first.
                        _hit = reg.registration_icp(
                            source.fine,
                            target.fine,
                            cfg.voxel_fine * cfg.icp_dist_factor,
                            np.linalg.inv(_T),
                            reg.TransformationEstimationPointToPlane(),
                            reg.ICPConvergenceCriteria(max_iteration=30),
                        )
                        if _hit.fitness >= cfg.fitness_threshold:
                            return _hit
                        _local.append(_hit)
                    if _local:
                        _best = max(_local, key=lambda r: r.fitness)
                        if _best.fitness >= cfg.fitness_threshold:
                            return _best
'''
        if slow in text and "DIMOS_RELOC_FAST_SEED_V1" not in text:
            text = text.replace(slow, fast, 1)
            changed = True
        if changed:
            target.write_text(text, encoding="utf-8")
            print(f"initial seed updated: {target}")
        else:
            print(f"already patched: {target}")
        return 0
    if "DIMOS_RELOC_HINT_SEED_V4" in text:
        old = '''        scored = [
            refine(np.asarray(hypothesis().transformation))
            for _ in range(max(cfg.ransac_restarts, 1))
        ]
        # DIMOS_RELOC_HINT_SEED_V4 — refine from hint and nearby yaw nudges.
'''
        new = '''        # DIMOS_RELOC_HINT_SEED_V5 — world←map of the departure pose.
        # The runtime composes the dog's map pose with odometry. A global
        # RANSAC hit must not replace that basin when it actually matches.
        _prior = os.environ.get("DIMOS_RELOC_WORLD_FROM_MAP", "").strip()
        if _prior:
            try:
                _parts = [float(x) for x in _prior.split(",")]
                if len(_parts) >= 3:
                    _hx, _hy, _hyaw = _parts[0], _parts[1], _parts[2]
                    _local = []
                    for _dyaw in (
                        0.0,
                        math.radians(10),
                        math.radians(-10),
                        math.radians(20),
                        math.radians(-20),
                    ):
                        _yaw = _hyaw + _dyaw
                        _c, _s = math.cos(_yaw), math.sin(_yaw)
                        _T = np.eye(4)
                        _T[0, 0], _T[0, 1], _T[1, 0], _T[1, 1] = _c, -_s, _s, _c
                        _T[0, 3], _T[1, 3] = _hx, _hy
                        _local.append(refine(np.linalg.inv(_T)))
                    if _local:
                        _best = max(_local, key=lambda r: r.fitness)
                        if _best.fitness >= cfg.fitness_threshold:
                            return _best
            except Exception:
                pass

        scored = [
            refine(np.asarray(hypothesis().transformation))
            for _ in range(max(cfg.ransac_restarts, 1))
        ]
'''
        if old not in text:
            print("could not find V4 hint-as-origin block", file=sys.stderr)
            return 1
        text = text.replace(old, new, 1)
        # Drop the copy that treated the map pose numbers as world←map.
        drop_start = text.find("        hint = os.environ.get(\"DIMOS_INITIAL_POSE\"")
        drop_end = text.find("        seed = os.environ.get(\"DIMOS_RELOC_SEED_POSE\"", drop_start)
        if drop_start < 0 or drop_end < 0:
            print("could not remove raw initial-pose seed", file=sys.stderr)
            return 1
        text = text[:drop_start] + text[drop_end:]
        target.write_text(text, encoding="utf-8")
        print(f"patched {target} -> {MARKER}")
        return 0
    if "DIMOS_RELOC_HINT_SEED_V3" in text or "DIMOS_RELOC_HINT_SEED_V2" in text:
        scored_at = text.find("        scored = [")
        if scored_at < 0:
            print("could not find scored block", file=sys.stderr)
            return 1
        if "DIMOS_RELOC_HINT_SEED_V4" not in text:
            fast = '''        # DIMOS_RELOC_HINT_SEED_V4 — after the first lock, local ICP only.
        # A global RANSAC here costs about a second and cannot track a turn.
        import math
        import os
        _seed = os.environ.get("DIMOS_RELOC_SEED_POSE", "").strip()
        if _seed:
            try:
                _parts = [float(x) for x in _seed.split(",")]
                if len(_parts) >= 3:
                    _hx, _hy, _hyaw = _parts[0], _parts[1], _parts[2]
                    _local = []
                    # DIMOS_RELOC_TRACK_YAW_V1 — one seed. Extra yaw guesses rotate a matched cloud.
                    for _dyaw in (0.0,):
                        _yaw = _hyaw + _dyaw
                        _c, _s = math.cos(_yaw), math.sin(_yaw)
                        _T = np.eye(4)
                        _T[0, 0], _T[0, 1], _T[1, 0], _T[1, 1] = _c, -_s, _s, _c
                        _T[0, 3], _T[1, 3] = _hx, _hy
                        _local.append(refine(np.linalg.inv(_T)))
                    if _local:
                        return max(_local, key=lambda r: r.fitness)
            except Exception:
                pass

'''
            text = text[:scored_at] + fast + text[scored_at:]
        text = text.replace("DIMOS_RELOC_HINT_SEED_V3", "DIMOS_RELOC_HINT_SEED_V4", 1)
        target.write_text(text, encoding="utf-8")
        print(f"patched {target} -> {MARKER}")
        return 0
    if "DIMOS_RELOC_HINT_SEED_V2" in text:
        start = text.find("        # DIMOS_RELOC_HINT_SEED_V2")
        end = text.find("        return max(scored, key=lambda r: r.fitness)", start)
        if start < 0 or end < 0:
            print("could not upgrade V2 hint seed", file=sys.stderr)
            return 1
        end = end + len("        return max(scored, key=lambda r: r.fitness)")
        # Keep the V2 hint nudges; append the seed-pose preference.
        v2 = text[start:end]
        addition = '''
        seed = os.environ.get("DIMOS_RELOC_SEED_POSE", "").strip()
        seed_scored = []
        if seed:
            try:
                parts = [float(x) for x in seed.split(",")]
                if len(parts) >= 3:
                    hx, hy, hyaw = parts[0], parts[1], parts[2]
                    for dyaw in (0.0, math.radians(5), math.radians(-5)):
                        yaw = hyaw + dyaw
                        c, s = math.cos(yaw), math.sin(yaw)
                        T_wm = np.eye(4)
                        T_wm[0, 0], T_wm[0, 1], T_wm[1, 0], T_wm[1, 1] = c, -s, s, c
                        T_wm[0, 3], T_wm[1, 3] = hx, hy
                        seed_scored.append(refine(np.linalg.inv(T_wm)))
            except Exception:
                pass
        # DIMOS_RELOC_HINT_SEED_V3 — once a lock exists, keep that basin.
        if seed_scored:
            best_seed = max(seed_scored, key=lambda r: r.fitness)
            if best_seed.fitness >= cfg.fitness_threshold:
                return best_seed
        return max(scored + seed_scored, key=lambda r: r.fitness)'''
        # Drop V2's original return; addition supplies it.
        v2_body = v2[: v2.rfind("        return max")]
        text = text[:start] + v2_body + addition + "\n" + text[end:]
        text = text.replace("DIMOS_RELOC_HINT_SEED_V2", "DIMOS_RELOC_HINT_SEED_V3", 1)
        target.write_text(text, encoding="utf-8")
        print(f"patched {target} -> {MARKER}")
        return 0
    backup = target.with_suffix(target.suffix + ".pre_hint_seed")
    if not backup.exists():
        backup.write_text(text, encoding="utf-8")
    # Prefer restoring clean then applying, if V1 present replace scored+seed block
    if "DIMOS_RELOC_HINT_SEED_V1" in text and OLD_V1_TAIL in text:
        # Replace from scored= through return max
        start = text.find("        scored = [")
        end = text.find("        return max(scored, key=lambda r: r.fitness)", start)
        if start < 0 or end < 0:
            print("could not locate scored block with V1", file=sys.stderr)
            return 1
        end = end + len("        return max(scored, key=lambda r: r.fitness)")
        text = text[:start] + NEW.strip("\n") + "\n" + text[end:]
    else:
        old = '''        scored = [
            refine(np.asarray(hypothesis().transformation))
            for _ in range(max(cfg.ransac_restarts, 1))
        ]
        return max(scored, key=lambda r: r.fitness)
'''
        if old not in text:
            # try restore backup
            if backup.exists():
                text = backup.read_text(encoding="utf-8")
            if old not in text:
                print("expected scored= block not found", file=sys.stderr)
                return 1
        text = text.replace(old, NEW.strip("\n") + "\n", 1)
    target.write_text(text, encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
