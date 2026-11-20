#!/usr/bin/env python3
"""Give odometry a moving timestamp so the Rerun body follows the dog.

Unitree often repeats one stamp. Rerun then keeps the first base_link pose,
and world/robot_body stays where it was drawn.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_ODOM_STAMP_V1"
OLD = """    def _publish_tf(self, msg: PoseStamped) -> None:
        msg.frame_id = self.config.odom_frame_id
"""
NEW = """    def _publish_tf(self, msg: PoseStamped) -> None:
        # DIMOS_ODOM_STAMP_V1 — a frozen stamp leaves robot_body fixed in Rerun.
        now = time.time()
        last = getattr(self, "_odom_stamp", 0.0)
        if msg.ts <= last + 1e-3:
            msg.ts = now
        self._odom_stamp = float(msg.ts)
        msg.frame_id = self.config.odom_frame_id
"""


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/robot/unitree/go2/connection.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if "import time" not in text:
        print("connection.py does not import time", file=sys.stderr)
        return 1
    if OLD not in text:
        print("could not find _publish_tf", file=sys.stderr)
        return 1
    target.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
