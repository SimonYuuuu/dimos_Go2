#!/usr/bin/env python3
"""Keep a slow Rerun viewer from stalling localization.

rr.log runs on a daemon thread. Live scans are dropped when that thread is
still inside a blocked gRPC send. Maps, costmaps, and tf are kept as the
latest sample and logged when the viewer catches up.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RERUN_DROP_V1"

HELPERS = '''
    def _viz_offer(self, msg: Any, topic: Any, entity_path: str) -> None:
        # DIMOS_RERUN_DROP_V1 — the viewer must not stall the robot.
        essential = entity_path in {
            "world/tf",
            "world/odom",
            "world/loaded_map",
            "world/global_map",
            "world/global_costmap",
            "world/navigation_costmap",
            "world/path",
            "world/goal_request",
        }
        if essential:
            with self._viz_lock:
                self._viz_priority[entity_path] = (msg, topic)
            self._viz_event.set()
            return
        try:
            self._viz_q.put_nowait((msg, topic))
        except queue.Full:
            try:
                self._viz_q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._viz_q.put_nowait((msg, topic))
            except queue.Full:
                self._viz_dropped += 1
        self._viz_event.set()

    def _viz_loop(self) -> None:
        while not getattr(self, "_viz_stop", False):
            self._viz_event.wait(0.5)
            self._viz_event.clear()
            batch: list[tuple[Any, Any]] = []
            with self._viz_lock:
                batch.extend(self._viz_priority.values())
                self._viz_priority.clear()
            while True:
                try:
                    batch.append(self._viz_q.get_nowait())
                except queue.Empty:
                    break
            for msg, topic in batch:
                try:
                    self._write_rerun(msg, topic)
                except Exception:
                    logger.warning("rerun log failed", exc_info=True)

'''

START = '''        pubsubs = _resolve_pubsubs(self.config)
'''

START_NEW = '''        # DIMOS_RERUN_DROP_V1 — log off the data thread. A full viewer drops frames.
        self._viz_stop = False
        self._viz_priority = {}
        self._viz_lock = threading.Lock()
        self._viz_event = threading.Event()
        self._viz_q = queue.Queue(maxsize=1)
        self._viz_dropped = 0
        self._viz_thread = threading.Thread(
            target=self._viz_loop, name="rerun-drop", daemon=True
        )
        self._viz_thread.start()

        pubsubs = _resolve_pubsubs(self.config)
'''

OFFER = '''        if self._tf_tree is not None and isinstance(msg, TFMessage):
'''

OFFER_NEW = '''        # DIMOS_RERUN_DROP_V1 — a blocked viewer must not stall localization.
        self._viz_offer(msg, topic, entity_path)
        return

    def _write_rerun(self, msg: Any, topic: Any) -> None:
        """Log one sample. Runs on the viewer thread, never on the data thread."""
        import rerun as rr

        entity_path: str = self._get_entity_path(topic)
        if self._tf_tree is not None and isinstance(msg, TFMessage):
'''

STOP = '''    def stop(self) -> None:
        self._override_cache.clear()
'''

STOP_NEW = '''    def stop(self) -> None:
        self._viz_stop = True
        event = getattr(self, "_viz_event", None)
        if event is not None:
            event.set()
        self._override_cache.clear()
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/visualization/rerun/bridge.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if "import queue\n" not in text:
        if "import os\n" not in text:
            print("import os missing", file=sys.stderr)
            return 1
        text = text.replace("import os\n", "import os\nimport queue\n", 1)
    if OFFER not in text or START not in text or STOP not in text:
        print("bridge anchors missing", file=sys.stderr)
        return 1
    text = text.replace(OFFER, OFFER_NEW, 1)
    text = text.replace(START, START_NEW, 1)
    text = text.replace(STOP, STOP_NEW, 1)
    # Helpers sit just before _on_message.
    needle = "    def _on_message(self, msg: Any, topic: Any) -> None:\n"
    if needle not in text:
        print("on_message missing", file=sys.stderr)
        return 1
    text = text.replace(needle, HELPERS + needle, 1)
    target.write_text(text, encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
