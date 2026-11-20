#!/usr/bin/env python3
"""Bound tracking ICP by how far it moves the dog, not the map origin.

Tracking restored the seed whenever the world<-map translation changed by more
than 8 cm. A yaw correction moves that translation by (distance from the world
origin) x angle, so far from the start every real correction was undone and the
lidar fix followed odometry drift. The bound now applies to the dog's pose on
the map: DIMOS_RELOC_TRACK_ICP_M (default 0.30 m) and
DIMOS_RELOC_TRACK_ICP_DEG (default 8 degrees).
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RELOC_TRACK_DOG_BOUND_V1"

OLD = (
    "                        _moved = np.linalg.inv(np.asarray(_hit.transformation))\n"
    "                        if math.hypot(float(_moved[0, 3]) - _hx, float(_moved[1, 3]) - _hy) > 0.08:\n"
    "                            _hit.transformation = np.linalg.inv(_T)\n"
)
NEW = (
    "                        _moved = np.linalg.inv(np.asarray(_hit.transformation))\n"
    f"                        # {MARKER} — judge the correction at the dog.\n"
    '                        _odom = os.environ.get("DIMOS_RELOC_ODOM", "").strip()\n'
    "                        _too_far = False\n"
    "                        if _odom:\n"
    '                            _ox, _oy, _oyaw = (float(v) for v in _odom.split(",")[:3])\n'
    "                            _wb = np.eye(4)\n"
    "                            _wb[0, 0], _wb[0, 1] = math.cos(_oyaw), -math.sin(_oyaw)\n"
    "                            _wb[1, 0], _wb[1, 1] = math.sin(_oyaw), math.cos(_oyaw)\n"
    "                            _wb[0, 3], _wb[1, 3] = _ox, _oy\n"
    "                            _mb_new = np.linalg.inv(_moved) @ _wb\n"
    "                            _mb_old = np.linalg.inv(_T) @ _wb\n"
    "                            _shift = math.hypot(\n"
    "                                float(_mb_new[0, 3] - _mb_old[0, 3]),\n"
    "                                float(_mb_new[1, 3] - _mb_old[1, 3]),\n"
    "                            )\n"
    "                            _turn = math.degrees(\n"
    "                                math.atan2(float(_mb_new[1, 0]), float(_mb_new[0, 0]))\n"
    "                                - math.atan2(float(_mb_old[1, 0]), float(_mb_old[0, 0]))\n"
    "                            )\n"
    "                            _turn = (_turn + 180.0) % 360.0 - 180.0\n"
    "                            _too_far = (\n"
    '                                _shift > float(os.environ.get("DIMOS_RELOC_TRACK_ICP_M", "0.30"))\n'
    '                                or abs(_turn) > float(os.environ.get("DIMOS_RELOC_TRACK_ICP_DEG", "8"))\n'
    "                            )\n"
    "                        else:\n"
    "                            _too_far = (\n"
    "                                math.hypot(float(_moved[0, 3]) - _hx, float(_moved[1, 3]) - _hy) > 0.08\n"
    "                            )\n"
    "                        if _too_far:\n"
    "                            _hit.transformation = np.linalg.inv(_T)\n"
)


def main() -> None:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    path = site / "dimos/mapping/relocalization/lidar/relocalize.py"
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path.name} {MARKER}")
        return
    if text.count(OLD) != 1:
        raise SystemExit(f"{path.name}: anchor found {text.count(OLD)} times")
    path.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {path.name} -> {MARKER}")


if __name__ == "__main__":
    main()
