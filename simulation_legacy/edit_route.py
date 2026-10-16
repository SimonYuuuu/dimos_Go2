#!/usr/bin/env python3
"""Create a patrol route by clicking on a saved point-cloud map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pointcloud_preview import read_xy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=Path, required=True, help="Saved .pc2.lcm map")
    parser.add_argument("--out", type=Path, default=Path("patrol_route.json"))
    parser.add_argument("--dwell", type=float, default=10.0, help="Default dwell time at each point")
    parser.add_argument("--start-x", type=float, default=-1.0)
    parser.add_argument("--start-y", type=float, default=1.0)
    args = parser.parse_args()

    points = read_xy(args.map)
    if len(points) == 0:
        raise ValueError(f"Map contains no points: {args.map}")
    route: list[tuple[float, float]] = []

    figure, axis = plt.subplots(figsize=(10, 8))
    axis.scatter(points[:, 0], points[:, 1], s=0.2, c="#6b8977", alpha=0.7)
    axis.set_aspect("equal", adjustable="box")
    axis.set_title("Left click: add waypoint | Right click: undo | Enter: save route")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    line, = axis.plot([], [], "r-o", linewidth=1.5, markersize=4)

    def redraw() -> None:
        if route:
            xy = np.asarray(route)
            line.set_data(xy[:, 0], xy[:, 1])
        else:
            line.set_data([], [])
        figure.canvas.draw_idle()

    def on_click(event) -> None:  # type: ignore[no-untyped-def]
        if event.inaxes != axis or event.xdata is None or event.ydata is None:
            return
        if event.button == 1:
            route.append((round(float(event.xdata), 3), round(float(event.ydata), 3)))
            redraw()
        elif event.button == 3 and route:
            route.pop()
            redraw()

    def on_key(event) -> None:  # type: ignore[no-untyped-def]
        if event.key not in ("enter", "return"):
            return
        if not route:
            print("Add at least one waypoint before saving.")
            return
        payload = {
            "start_at": None,
            "return_to_start": {"x": args.start_x, "y": args.start_y},
            "waypoints": [{"x": x, "y": y, "dwell_s": args.dwell} for x, y in route],
        }
        args.out.write_text(json.dumps(payload, indent=2) + "\n")
        print(f"Saved {len(route)} waypoints to {args.out}")
        plt.close(figure)

    figure.canvas.mpl_connect("button_press_event", on_click)
    figure.canvas.mpl_connect("key_press_event", on_key)
    plt.show()


if __name__ == "__main__":
    main()