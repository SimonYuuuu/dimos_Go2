#!/usr/bin/env python3
"""Keep Rerun on the dog, the live scan, the path, and the obstacle map.

The prior cloud, camera, and accumulated map were filling the phone link.
loaded_map is not sent unless DIMOS_SHOW_LOADED_MAP=1.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_RERUN_TRACK_V1"
MARKER_V2 = "DIMOS_RERUN_TRACK_V2"
MARKER_V3 = "DIMOS_RERUN_TRACK_V3"

FILTER = '''        entity_path: str = self._get_entity_path(topic)

        # DIMOS_RERUN_TRACK_V2 — pose, live scan, path, obstacle map, and a 1 Hz camera.
        if os.environ.get("DIMOS_RERUN_TRACK", "1").lower() not in ("0", "false", "no"):
            keep = {
                "world/tf",
                "world/odom",
                "world/lidar",
                "world/path",
                "world/navigation_costmap",
                "world/global_costmap",
                "world/goal_request",
                "world/color_image",
                "world/camera_info",
                "world/global_map",
            }
            if os.environ.get("DIMOS_SHOW_LOADED_MAP", "0").lower() in ("1", "true", "yes"):
                keep.add("world/loaded_map")
            if entity_path not in keep:
                return
            # DIMOS_RERUN_TRACK_V4 — a later lock correction is drawn; not an 8 Hz stream.
            if entity_path == "world/global_map":
                sent = getattr(self, "_global_map_once", 0)
                if sent >= 8:
                    return
                self._global_map_once = sent + 1
'''

OLD_BLUEPRINT = '''def _go2_rerun_blueprint() -> Blueprint:
    """Split layout: camera feed + 3D world view side by side."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.TimeSeriesView(
                    origin="plots",
                    contents=["plots/odom/**", "plots/cmd_vel/**"],
                    name="odom + cmd_vel",
                ),
                rrb.TimeSeriesView(origin="plots/telemetry", name="telemetry"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                    "world/pgo": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )
'''

NEW_BLUEPRINT = '''def _go2_rerun_blueprint() -> Blueprint:
    """3D tracking view. Camera, plots, and the prior map stay out of the link."""
    import os

    import rerun as rr
    import rerun.blueprint as rrb

    # DIMOS_RERUN_TRACK_V2
    if os.environ.get("DIMOS_RERUN_TRACK", "1").lower() not in ("0", "false", "no"):
        return rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial2DView(origin="world/color_image", name="相机"),
                rrb.Spatial3DView(
                    origin="world",
                    name="巡检",
                    background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                    line_grid=rrb.LineGrid3D(
                        plane=rr.components.Plane3D.XY.with_distance(0.5),
                    ),
                    overrides={
                        "world/loaded_map": rrb.EntityBehavior(visible=False),
                        "world/pgo": rrb.EntityBehavior(visible=False),
                        "world/pgo_keyframes": rrb.EntityBehavior(visible=False),
                        "world/pgo_loops": rrb.EntityBehavior(visible=False),
                    },
                ),
                column_shares=[1, 2],
            ),
            rrb.TimePanel(state="collapsed"),
            rrb.SelectionPanel(state="collapsed"),
        )

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.TimeSeriesView(
                    origin="plots",
                    contents=["plots/odom/**", "plots/cmd_vel/**"],
                    name="odom + cmd_vel",
                ),
                rrb.TimeSeriesView(origin="plots/telemetry", name="telemetry"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                    "world/pgo": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )
'''

OLD_HZ = '''    "max_hz": {
        "world/global_map": 0,  # publishes at ~7.8 Hz
        "world/color_image": 0,  # publishes at ~14 Hz
        "world/global_costmap": 0,  # publishes at ~7.6 Hz
        "world/lidar": 1,  # publishes at ~7.7 Hz; hidden by default in the blueprint
    },
'''

NEW_HZ = '''    "max_hz": {
        # DIMOS_RERUN_TRACK_V2 — every lidar sweep. Camera is one frame a second.
        "world/lidar": 10,
        "world/global_costmap": 4,
        "world/navigation_costmap": 4,
        "world/path": 5,
        "world/global_map": 0.2,
        "world/color_image": 1,
    },
'''


def _ensure_path_draw(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    if "_draw_nav_path" in text:
        return
    fn = '''def _draw_nav_path(path: Any) -> Any:
    """DIMOS_RERUN_PATH_V1 — thick line so the route stays visible over the scan."""
    return path.to_rerun(color=(255, 196, 0), z_offset=0.45, radii=0.08)


'''
    needle = "rerun_config: dict[str, Any] = {\n"
    old = '"world/camera_info": _convert_camera_info,\n'
    new = old + '        "world/path": _draw_nav_path,\n'
    if needle not in text or old not in text:
        raise SystemExit(f"{path}: cannot add path draw")
    path.write_text(text.replace(needle, fn + needle, 1).replace(old, new, 1), encoding="utf-8")
    print(f"path draw added: {path}")


def _site(argv: list[str]) -> Path:
    if argv:
        return Path(argv[0])
    return (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )


KEEP_V2 = '''                "world/camera_info",
            }
            if os.environ.get("DIMOS_SHOW_LOADED_MAP", "0").lower() in ("1", "true", "yes"):
                keep.add("world/loaded_map")
            if entity_path not in keep:
                return
'''

KEEP_V3 = '''                "world/camera_info",
                "world/global_map",
            }
            if os.environ.get("DIMOS_SHOW_LOADED_MAP", "0").lower() in ("1", "true", "yes"):
                keep.add("world/loaded_map")
            if entity_path not in keep:
                return
            # DIMOS_RERUN_TRACK_V4 — a later lock correction is drawn; not an 8 Hz stream.
            if entity_path == "world/global_map":
                sent = getattr(self, "_global_map_once", 0)
                if sent >= 8:
                    return
                self._global_map_once = sent + 1
'''

HIDE_GLOBAL = '''                        "world/loaded_map": rrb.EntityBehavior(visible=False),
                        "world/global_map": rrb.EntityBehavior(visible=False),
'''

SHOW_GLOBAL = '''                        "world/loaded_map": rrb.EntityBehavior(visible=True),
'''


def _upgrade_v3(bridge: Path, basic: Path) -> None:
    bridge_text = bridge.read_text(encoding="utf-8")
    old_cap = '''            # DIMOS_RERUN_TRACK_V3 — two static snapshots, then the 8 Hz stream stops.
            if entity_path == "world/global_map":
                sent = getattr(self, "_global_map_once", 0)
                if sent >= 2:
                    return
                self._global_map_once = sent + 1
'''
    new_cap = '''            # DIMOS_RERUN_TRACK_V4 — a later lock correction is drawn; not an 8 Hz stream.
            if entity_path == "world/global_map":
                sent = getattr(self, "_global_map_once", 0)
                if sent >= 8:
                    return
                self._global_map_once = sent + 1
'''
    if old_cap in bridge_text:
        bridge.write_text(bridge_text.replace(old_cap, new_cap, 1), encoding="utf-8")
        print(f"patched {bridge} -> DIMOS_RERUN_TRACK_V4")
        bridge_text = new_cap
    if MARKER_V3 not in bridge_text and "DIMOS_RERUN_TRACK_V4" not in bridge.read_text(encoding="utf-8"):
        if KEEP_V2 not in bridge_text:
            raise SystemExit(f"{bridge}: track filter has no v2 keep list")
        bridge.write_text(bridge_text.replace(KEEP_V2, KEEP_V3, 1), encoding="utf-8")
        print(f"upgraded {bridge}")
    else:
        print(f"already v3: {bridge}")

    basic_text = basic.read_text(encoding="utf-8")
    if HIDE_GLOBAL in basic_text:
        basic.write_text(basic_text.replace(HIDE_GLOBAL, SHOW_GLOBAL, 1), encoding="utf-8")
        print(f"upgraded {basic}")
    else:
        print(f"already showing global map: {basic}")


def main() -> int:
    site = _site(sys.argv[1:])
    bridge = site / "dimos/visualization/rerun/bridge.py"
    basic = site / "dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py"
    for path in (bridge, basic):
        if not path.is_file():
            print(f"not found: {path}", file=sys.stderr)
            return 1

    bridge_text = bridge.read_text(encoding="utf-8")
    if MARKER_V3 in bridge_text:
        print(f"already patched: {bridge}")
    elif MARKER_V2 in bridge_text:
        print(f"already patched: {bridge}")
    elif MARKER in bridge_text:
        old_keep = '''                "world/goal_request",
            }'''
        new_keep = '''                "world/goal_request",
                "world/color_image",
                "world/camera_info",
            }'''
        if old_keep not in bridge_text:
            print("camera keep-list not found", file=sys.stderr)
            return 1
        bridge_text = bridge_text.replace(old_keep, new_keep, 1).replace(MARKER, MARKER_V2, 1)
        bridge.write_text(bridge_text, encoding="utf-8")
        print(f"patched {bridge}")
    else:
        needle = "        entity_path: str = self._get_entity_path(topic)\n"
        if needle not in bridge_text:
            print("bridge entity path not found", file=sys.stderr)
            return 1
        bridge.write_text(bridge_text.replace(needle, FILTER, 1), encoding="utf-8")
        print(f"patched {bridge}")

    basic_text = basic.read_text(encoding="utf-8")
    if MARKER_V2 in basic_text and '"world/color_image": 1' in basic_text:
        _ensure_path_draw(basic)
        _upgrade_v3(bridge, basic)
        print(f"already patched: {basic}")
        return 0
    if MARKER in basic_text and '"world/lidar": 10' in basic_text:
        old_view = '''        return rrb.Blueprint(
            rrb.Spatial3DView(
                origin="world",
                name="巡检",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/loaded_map": rrb.EntityBehavior(visible=False),
                    "world/global_map": rrb.EntityBehavior(visible=False),
                    "world/color_image": rrb.EntityBehavior(visible=False),
                    "world/pgo": rrb.EntityBehavior(visible=False),
                    "world/pgo_keyframes": rrb.EntityBehavior(visible=False),
                    "world/pgo_loops": rrb.EntityBehavior(visible=False),
                },
            ),
            rrb.TimePanel(state="collapsed"),
            rrb.SelectionPanel(state="collapsed"),
        )'''
        new_view = '''        return rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial2DView(origin="world/color_image", name="相机"),
                rrb.Spatial3DView(
                    origin="world",
                    name="巡检",
                    background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                    line_grid=rrb.LineGrid3D(
                        plane=rr.components.Plane3D.XY.with_distance(0.5),
                    ),
                    overrides={
                        "world/loaded_map": rrb.EntityBehavior(visible=False),
                        "world/pgo": rrb.EntityBehavior(visible=False),
                        "world/pgo_keyframes": rrb.EntityBehavior(visible=False),
                        "world/pgo_loops": rrb.EntityBehavior(visible=False),
                    },
                ),
                column_shares=[1, 2],
            ),
            rrb.TimePanel(state="collapsed"),
            rrb.SelectionPanel(state="collapsed"),
        )'''
        if old_view not in basic_text:
            print("track view not found", file=sys.stderr)
            return 1
        basic_text = basic_text.replace(old_view, new_view, 1)
        basic_text = basic_text.replace('"world/color_image": 0.2', '"world/color_image": 1', 1)
        basic_text = basic_text.replace(MARKER, MARKER_V2)
        basic.write_text(basic_text, encoding="utf-8")
        _ensure_path_draw(basic)
        print(f"patched {basic}")
        return 0
    if OLD_BLUEPRINT not in basic_text:
        print("blueprint block not found", file=sys.stderr)
        return 1
    if OLD_HZ not in basic_text:
        print("max_hz block not found", file=sys.stderr)
        return 1
    basic.write_text(
        basic_text.replace(OLD_BLUEPRINT, NEW_BLUEPRINT, 1).replace(OLD_HZ, NEW_HZ, 1),
        encoding="utf-8",
    )
    _ensure_path_draw(basic)
    print(f"patched {basic}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
