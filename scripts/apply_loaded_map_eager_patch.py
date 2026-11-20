#!/usr/bin/env python3
"""Show prior map in Rerun immediately (do not wait for an accepted ICP fix).

Upstream only publishes loaded_map via fix_stream after the first fix, and a
one-shot publish in start() is often dropped (Rerun not subscribed yet). Also
loaded_map lives in the map frame — without a TF it never appears in the world
view.

V3: timer-republish loaded_map. The world←map preview is published by the
lidar module once odometry arrives, composed from the dog's map pose.
Do not paste DIMOS_INITIAL_POSE in as the map origin.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_LOADED_MAP_EAGER_V4"
OLD_MARKERS = ("DIMOS_LOADED_MAP_EAGER_V1", "DIMOS_LOADED_MAP_EAGER_V3")

NEW_LOAD = '''    def _load_premap(self, map_file: str) -> None:
        # DIMOS_LOADED_MAP_EAGER_V4
        import reactivex as rx

        path = resolve_named_path(map_file, MAP_SUFFIX)
        premap = PointCloud2.lcm_decode(path.read_bytes())

        premap.frame_id = self.config.map_frame
        self.premap = premap

        # A few publishes cover a late Rerun subscribe. The cloud is logged static,
        # so repeating it every few seconds only fills the viewer and freezes the page.
        self.register_disposable(
            rx.merge(rx.timer(0.5), rx.timer(2.0), rx.timer(8.0)).subscribe(
                lambda _: self.loaded_map.publish(premap)
            )
        )
        # Keep upstream behavior: also republish on every accepted fix.
        self.register_disposable(
            fix_stream(self.fixes, self.config.republish_loaded_map).subscribe(
                lambda _: self.loaded_map.publish(premap)
            )
        )

        # DIMOS_LOADED_MAP_EAGER_V4 — no raw-hint TF. The lidar module publishes
        # world←map = odom * inverse(dog pose on the map) once odometry arrives.
        logger.info(
            "relocalize: prior map loaded; world←map waits for odometry "
            "so the dog's map pose is not used as the map origin"
        )
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/mapping/relocalization/module.py"
    )
    if not target.is_file():
        print(f"module not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    old_timer = """        self.register_disposable(
            rx.timer(0.5, 3.0).subscribe(lambda _: self.loaded_map.publish(premap))
        )"""
    new_timer = """        # DIMOS_LOADED_MAP_EAGER_V4 — a few times, then stop.
        self.register_disposable(
            rx.merge(rx.timer(0.5), rx.timer(2.0), rx.timer(8.0)).subscribe(
                lambda _: self.loaded_map.publish(premap)
            )
        )"""
    if "DIMOS_LOADED_MAP_EAGER_V3" in text and old_timer in text:
        text = text.replace(old_timer, new_timer, 1).replace(
            "DIMOS_LOADED_MAP_EAGER_V3", "DIMOS_LOADED_MAP_EAGER_V4"
        )
        target.write_text(text, encoding="utf-8")
        print(f"patched {target} -> {MARKER}")
        return 0
    start = text.find("    def _load_premap(self, map_file: str) -> None:")
    if start < 0:
        print("could not find _load_premap", file=sys.stderr)
        return 1
    # Next method at class indent
    end = text.find("\n    @property\n    def placed", start)
    if end < 0:
        end = text.find("\n    def keep_relocalizing", start)
    if end < 0:
        print("could not find end of _load_premap", file=sys.stderr)
        return 1
    backup = target.with_suffix(target.suffix + ".pre_eager_map")
    if not backup.exists():
        backup.write_text(text, encoding="utf-8")
    patched = text[:start] + NEW_LOAD.rstrip() + "\n\n" + text[end + 1 :]
    target.write_text(patched, encoding="utf-8")
    print(f"patched {target} -> {MARKER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
