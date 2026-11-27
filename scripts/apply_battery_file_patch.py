#!/usr/bin/env python3
"""Write Go2 battery SOC to PATROL_BATTERY_FILE whenever lowstate arrives."""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_BATTERY_FILE_V1"

OLD = '''    def _on_lowstate(self, msg: LowStateMsg) -> None:
        """Cache the latest low-level state push (battery, IMU, motors, etc.)."""
        self._latest_lowstate = msg
'''

NEW = '''    def _on_lowstate(self, msg: LowStateMsg) -> None:
        """Cache the latest low-level state push (battery, IMU, motors, etc.)."""
        self._latest_lowstate = msg
        # DIMOS_BATTERY_FILE_V1 — console and patrol read SOC from this file.
        try:
            import json
            import os
            import time

            soc = int(msg["data"]["bms_state"]["soc"])  # type: ignore[index]
            path = os.environ.get("PATROL_BATTERY_FILE", "").strip()
            if not path or not 0 <= soc <= 100:
                return
            now = time.time()
            if now - float(getattr(self, "_battery_file_at", 0.0)) < 2.0:
                return
            self._battery_file_at = now
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump({"soc": soc, "t": now}, handle)
            os.replace(tmp, path)
        except (KeyError, TypeError, ValueError, OSError):
            return
'''


def main() -> None:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages/dimos/robot/unitree/go2/connection.py"
    )
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path.name} {MARKER}")
        return
    if text.count(OLD) != 1:
        raise SystemExit(f"{path.name}: lowstate anchor found {text.count(OLD)} times")
    path.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    print(f"patched {path.name} -> {MARKER}")


if __name__ == "__main__":
    main()
