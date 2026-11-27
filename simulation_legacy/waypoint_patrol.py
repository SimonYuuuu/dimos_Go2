#!/usr/bin/env python3
"""Drive the Go2 through a scheduled fixed route.

Publishes each waypoint to /goal_request (consumed by ReplanningAStarPlanner)
and blocks until /goal_reached fires before sending the next one. Run this
against an already-running `dimos ... run unitree-go2-relocalization` (or any
blueprint that includes ReplanningAStarPlanner) — it does not start DimOS
itself.

Usage:
    .venv/bin/python waypoint_patrol.py --route patrol_route.json
    .venv/bin/python waypoint_patrol.py --route patrol_route.json --auto
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos_lcm.std_msgs import Bool

GOAL_TIMEOUT_S = 60.0
GOAL_RESEND_S = 3.0
GOAL_FREEZE_M = 1.5


def dog_on_map() -> tuple[float, float] | None:
    """The dog's live map position written by the relocalization module."""
    path = os.environ.get("PATROL_DOG_POSE_FILE", "").strip()
    if not path:
        return None
    try:
        pose = json.loads(Path(path).read_text(encoding="utf-8"))
        if time.time() - float(pose.get("t", 0.0)) > 3.0:
            return None
        return float(pose["x"]), float(pose["y"])
    except (OSError, ValueError, KeyError, TypeError):
        return None
COSTMAP_TIMEOUT_S = 120.0


def load_route(
    path: Path,
) -> tuple[
    list[tuple[float, float, float]],
    tuple[float, float] | None,
    str | None,
    float | None,
    float | None,
]:
    data = json.loads(path.read_text())
    waypoints = [
        (float(point["x"]), float(point["y"]), float(point.get("dwell_s", 0)))
        for point in data["waypoints"]
    ]
    if not waypoints:
        raise ValueError("route must contain at least one waypoint")
    start = data.get("return_to_start")
    return_to_start = (float(start["x"]), float(start["y"])) if start else None
    max_duration_s = data.get("max_duration_s")
    repeat_interval_s = data.get("repeat_interval_s")
    return (
        waypoints,
        return_to_start,
        data.get("start_at"),
        float(max_duration_s) if max_duration_s else None,
        float(repeat_interval_s) if repeat_interval_s else None,
    )


def write_status(path: Path | None, **fields: object) -> None:
    if path is None:
        return
    state = {"updated_at": datetime.now(timezone.utc).isoformat(), **fields}
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(state, indent=2) + "\n")
    temporary.replace(path)


def transform_signature(transform_file: Path | None) -> tuple[float, float, float] | None:
    """Coarse world<-map signature. Changes when a tracking fix should move the goal."""
    if transform_file is None:
        return None
    try:
        transform = json.loads(transform_file.read_text())
        tx, ty, _tz = transform["translation"]
        yaw = float(transform.get("yaw_deg", 0.0))
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    # 0.4 m / 5 deg. Finer updates cancel the live goal and zero the gait.
    return (round(float(tx) * 2.5) / 2.5, round(float(ty) * 2.5) / 2.5, round(yaw / 5.0) * 5.0)


def map_point_to_world(
    x: float, y: float, transform_file: Path | None
) -> tuple[float, float]:
    if transform_file is None:
        return x, y
    try:
        transform = json.loads(transform_file.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"No valid map-to-world transform at {transform_file}; refusing to move"
        ) from error
    if transform.get("frame_id") != "world" or transform.get("child_frame_id") != "map":
        raise RuntimeError("Localization transform must be world <- map")
    tx, ty, _tz = transform["translation"]
    qx, qy, qz, qw = transform["rotation_xyzw"]
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return (
        tx + math.cos(yaw) * x - math.sin(yaw) * y,
        ty + math.sin(yaw) * x + math.cos(yaw) * y,
    )


_task_seq = 0


def battery_soc() -> int | None:
    path = os.environ.get("PATROL_BATTERY_FILE", "").strip()
    if not path:
        return None
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        soc = int(data["soc"])
        age = time.time() - float(data.get("t", 0.0))
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if age > 30 or not 0 <= soc <= 100:
        return None
    return soc


def battery_low() -> bool:
    soc = battery_soc()
    return soc is not None and soc < 5


def fire_state(control_file: Path | None) -> str:
    return str(read_control(control_file).get("fire_state", "NaN"))


def log_task(event: str, **fields: object) -> None:
    """One JSON line per patrol event, in the task file named by PATROL_TASK_LOG."""
    global _task_seq
    raw = os.environ.get("PATROL_TASK_LOG", "").strip()
    if not raw:
        return
    _task_seq += 1
    record = {
        "seq": _task_seq,
        "t": datetime.now(timezone.utc).isoformat(),
        "event": event,
        **fields,
    }
    path = Path(raw)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
    except OSError:
        return


def read_control(control_file: Path | None) -> dict[str, object]:
    if control_file is None:
        return {}
    try:
        data = json.loads(control_file.read_text())
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def wait_until(
    start_at: str | None,
    auto: bool,
    status_file: Path | None,
    control_file: Path | None,
) -> bool:
    if not auto:
        input("Localization confirmed. Press Enter to start the patrol... ")
    if not start_at:
        return not bool(read_control(control_file).get("stop_requested"))
    scheduled = datetime.strptime(start_at, "%H:%M").time()
    now = datetime.now()
    target = datetime.combine(now.date(), scheduled)
    if target <= now:
        target += timedelta(days=1)
    while True:
        control = read_control(control_file)
        if control.get("stop_requested"):
            write_status(status_file, phase="stopped", message="Scheduled patrol cancelled before start")
            return False
        now = datetime.now()
        if control.get("paused"):
            write_status(status_file, phase="paused_fire", message="Scheduled patrol paused for fire alarm; waiting for Safe confirmation")
            time.sleep(0.1)
            continue
        if now >= target:
            return True
        remaining = int((target - now).total_seconds())
        write_status(status_file, phase="scheduled", message=f"Scheduled start in {remaining}s")
        print(f"Scheduled patrol begins in {remaining}s", end="\r", flush=True)
        time.sleep(min(max(remaining, 1), 0.25))


def run_patrol(
    waypoints: list[tuple[float, float, float]],
    return_to_start: tuple[float, float] | None,
    max_duration_s: float | None,
    repeat_interval_s: float | None,
    status_file: Path | None,
    transform_file: Path | None,
    control_file: Path | None,
    repeat_enabled: bool = False,
) -> None:
    goal_reached = threading.Event()
    goal_failed = threading.Event()

    def _on_goal_reached(msg: Bool) -> None:
        if msg.data:
            goal_reached.set()
        else:
            goal_failed.set()

    # Use DimOS factory so Zenoh keys are `dimos/<name>/...` (leading `/` is illegal).
    goal_request = make_transport("goal_request", PoseStamped)
    reached = make_transport("goal_reached", Bool)
    stop_movement = make_transport("stop_movement", Bool)
    costmap_ready = threading.Event()

    def _on_global_costmap(msg: OccupancyGrid) -> None:
        grid = getattr(msg, "grid", None)
        if msg.width > 0 and msg.height > 0 and getattr(grid, "size", 0) > 0:
            costmap_ready.set()

    global_costmap = make_transport("global_costmap", OccupancyGrid)
    reached.subscribe(_on_goal_reached)
    global_costmap.subscribe(_on_global_costmap)

    # Let the subscription attach before the first publish.
    time.sleep(0.5)

    deadline: float | None = None
    cycle = 0

    def stop_requested() -> bool:
        return bool(read_control(control_file).get("stop_requested"))

    def duration_expired() -> bool:
        return deadline is not None and time.monotonic() >= deadline

    def publish_goal(x: float, y: float) -> None:
        world_x, world_y = map_point_to_world(x, y, transform_file)
        goal_request.broadcast(
            None,
            PoseStamped(
                position=(world_x, world_y, 0),
                orientation=(0, 0, 0, 1),
                frame_id="world",
            ),
        )

    def navigate(
        x: float,
        y: float,
        phase: str,
        cycle_number: int,
        index: int,
        total: int,
        allow_pause: bool = True,
    ) -> str:
        world_x, world_y = map_point_to_world(x, y, transform_file)
        while allow_pause and read_control(control_file).get("paused"):
            write_status(
                status_file,
                phase="paused_fire",
                cycle=cycle_number,
                index=index,
                total=total,
                x=x,
                y=y,
                message="Fire alarm active; waiting for Safe confirmation before navigation",
            )
            if battery_low():
                return "battery"
            if stop_requested():
                return "stop"
            time.sleep(0.1)
        goal_reached.clear()
        goal_failed.clear()
        write_status(
            status_file,
            phase=phase,
            cycle=cycle_number,
            index=index,
            total=total,
            x=x,
            y=y,
            world_x=world_x,
            world_y=world_y,
            message="Navigating to home point" if phase == "returning" else "Navigating to route point",
        )
        goal_request.broadcast(
            None,
            PoseStamped(
                position=(world_x, world_y, 0),
                orientation=(0, 0, 0, 1),
                frame_id="world",
            ),
        )
        goal_deadline = time.monotonic() + GOAL_TIMEOUT_S
        last_send = time.monotonic()
        paused = False
        last_tf = transform_signature(transform_file)
        last_pub = (world_x, world_y)
        while True:
            if allow_pause and read_control(control_file).get("paused"):
                if not paused:
                    stop_movement.broadcast(None, Bool(data=True))
                    paused = True
                    write_status(
                        status_file,
                        phase="paused_fire",
                        cycle=cycle_number,
                        index=index,
                        total=total,
                        x=x,
                        y=y,
                        message="Patrol paused for fire alarm; waiting for operator confirmation",
                    )
                if battery_low():
                    return "battery"
                if stop_requested():
                    return "stop"
                time.sleep(0.1)
                continue
            if paused:
                if battery_low():
                    return "battery"
                if stop_requested():
                    return "stop"
                paused = False
                goal_reached.clear()
                publish_goal(x, y)
                goal_deadline = time.monotonic() + GOAL_TIMEOUT_S
                write_status(
                    status_file,
                    phase=phase,
                    cycle=cycle_number,
                    index=index,
                    total=total,
                    x=x,
                    y=y,
                    message="Operator confirmed alarm cleared; resuming previous goal",
                )
            if goal_reached.is_set():
                return "reached"
            fresh_tf = transform_signature(transform_file)
            if fresh_tf is not None and fresh_tf != last_tf:
                last_tf = fresh_tf
                nx, ny = map_point_to_world(x, y, transform_file)
                # A few centimeters of tracking noise was cancelling the stop and
                # starting the same path again. Republish only after a real shift,
                # and not on the final approach, where a new goal bends the path.
                dog = dog_on_map()
                approaching = dog is not None and math.hypot(dog[0] - x, dog[1] - y) < GOAL_FREEZE_M
                if not approaching and math.hypot(nx - last_pub[0], ny - last_pub[1]) >= 0.4:
                    last_pub = (nx, ny)
                    goal_reached.clear()
                    publish_goal(x, y)
                    goal_deadline = time.monotonic() + GOAL_TIMEOUT_S
                    write_status(
                        status_file,
                        phase=phase,
                        cycle=cycle_number,
                        index=index,
                        total=total,
                        x=x,
                        y=y,
                        message="Localization updated; republishing the current goal",
                    )
            if read_control(control_file).get("halt"):
                stop_movement.broadcast(None, Bool(data=True))
                return "stop"
            if allow_pause and battery_low():
                stop_movement.broadcast(None, Bool(data=True))
                return "battery"
            if allow_pause and stop_requested():
                stop_movement.broadcast(None, Bool(data=True))
                return "stop"
            if not paused and time.monotonic() >= goal_deadline:
                return "timeout"
            # The planner drops a goal for good after "No path found"; the
            # costmap keeps changing, so ask again.
            if goal_failed.is_set() and time.monotonic() - last_send >= GOAL_RESEND_S:
                goal_failed.clear()
                last_send = time.monotonic()
                print(f"  planner gave up on ({x}, {y}); resending goal", flush=True)
                publish_goal(x, y)
            goal_reached.wait(timeout=0.1)

    def wait_dwell(seconds: float, cycle_number: int, index: int) -> bool:
        remaining = seconds
        paused = False
        while remaining > 0:
            if battery_low() or stop_requested():
                return False
            if read_control(control_file).get("paused"):
                if not paused:
                    stop_movement.broadcast(None, Bool(data=True))
                    paused = True
                    write_status(
                        status_file,
                        phase="paused_fire",
                        cycle=cycle_number,
                        index=index,
                        total=len(waypoints),
                        message="Patrol paused for fire alarm; waiting for operator confirmation",
                    )
                time.sleep(0.1)
                continue
            paused = False
            interval = min(0.1, remaining)
            time.sleep(interval)
            remaining -= interval
        return True

    stop_reason = "completed"
    failure: str | None = None
    home_reached = False
    navigation_ready = False

    def finish_patrol(reason: str, error: str | None = None) -> None:
        nonlocal stop_reason, failure, home_reached
        stop_reason = reason
        failure = error
        if read_control(control_file).get("halt"):
            stop_movement.broadcast(None, Bool(data=True))
            write_status(
                status_file,
                phase="stopped",
                cycle=cycle,
                index=len(waypoints),
                total=len(waypoints),
                message="Halted in place; not returning home",
            )
            print("Halted in place", flush=True)
            return
        if return_to_start is not None and navigation_ready:
            while read_control(control_file).get("paused"):
                write_status(
                    status_file,
                    phase="paused_fire",
                    cycle=cycle,
                    index=len(waypoints),
                    total=len(waypoints),
                    message="Patrol has ended; fire alarm is active, waiting for Safe before returning home",
                )
                time.sleep(0.1)
            home_x, home_y = return_to_start
            log_task(
                "return_home",
                reason=stop_reason,
                x=home_x,
                y=home_y,
                fire=fire_state(control_file),
                soc=battery_soc(),
            )
            write_status(
                status_file,
                phase="returning",
                cycle=cycle,
                index=len(waypoints),
                total=len(waypoints),
                message="Patrol ending; navigating from current position to the home point",
            )
            result = navigate(
                home_x,
                home_y,
                "returning",
                cycle,
                len(waypoints),
                len(waypoints),
                allow_pause=False,
            )
            home_reached = result == "reached"
            if not home_reached:
                failure = failure or "Could not reach the home point"
                stop_reason = "error"

        final_message = (
            "电量低于 5%，已回到出发返回点"
            if stop_reason == "battery" and home_reached
            else "电量低于 5%，返航未完成"
            if stop_reason == "battery"
            else "Patrol stopped and robot returned home"
            if stop_reason == "stopped" and home_reached
            else "任务时长已到，本圈走完并已回到出发返回点"
            if stop_reason == "duration" and home_reached
            else "Patrol completed and robot returned home"
            if stop_reason == "completed" and home_reached
            else failure or ("Patrol completed at the final waypoint" if stop_reason == "completed" else "Patrol ended")
        )
        phase = (
            "error"
            if stop_reason == "error"
            else "completed"
            if stop_reason in ("completed", "duration", "battery")
            else "stopped"
        )
        log_task(
            "finished",
            reason=stop_reason,
            phase=phase,
            home_reached=home_reached,
            fire=fire_state(control_file),
            soc=battery_soc(),
            message=final_message,
        )
        write_status(status_file, phase=phase, cycle=cycle, index=len(waypoints), total=len(waypoints), message=final_message)
        print(final_message, flush=True)

    try:
        costmap_deadline = time.monotonic() + COSTMAP_TIMEOUT_S
        while not costmap_ready.wait(timeout=0.1):
            if stop_requested():
                stop_reason = "stopped"
                failure = "Patrol stopped before a navigation costmap was available"
                break
            if read_control(control_file).get("paused"):
                write_status(
                    status_file,
                    phase="paused_fire",
                    cycle=0,
                    index=0,
                    total=len(waypoints),
                    message="Fire alarm active; waiting for Safe before navigation setup",
                )
                if time.monotonic() >= costmap_deadline:
                    stop_reason = "error"
                    failure = "Fire alarm remained active while waiting for /global_costmap; no navigation goal was sent"
                    break
                continue
            if time.monotonic() >= costmap_deadline:
                stop_reason = "error"
                failure = "No valid /global_costmap received within 60s; no navigation goal was sent"
                break
            write_status(
                status_file,
                phase="waiting_for_costmap",
                cycle=0,
                index=0,
                total=len(waypoints),
                message="Localization is ready; waiting for the navigation costmap before dispatching goals",
            )
        navigation_ready = costmap_ready.is_set()
        if navigation_ready:
            deadline = time.monotonic() + max_duration_s if max_duration_s else None

        if stop_reason == "completed" and return_to_start is not None:
            result = navigate(*return_to_start, "initializing", 0, 0, len(waypoints))
            if result == "reached":
                home_reached = True
                write_status(
                    status_file,
                    phase="running",
                    cycle=0,
                    index=0,
                    total=len(waypoints),
                    message="At route departure point; beginning patrol",
                )
            elif result == "battery":
                stop_reason = "battery"
            elif result == "timeout":
                failure = "Could not reach route departure point"
                stop_reason = "error"
            else:
                stop_reason = "stopped"

        while True:
            if stop_reason != "completed":
                break
            if stop_requested():
                stop_reason = "stopped"
                break
            if battery_low():
                stop_reason = "battery"
                log_task("battery_return", fire=fire_state(control_file), soc=battery_soc(), message="电量低于 5%，立即返回出发返回点")
                break
            if duration_expired():
                stop_reason = "duration"
                break
            cycle += 1
            write_status(
                status_file,
                phase="running",
                cycle=cycle,
                index=0,
                total=len(waypoints),
                message="Starting patrol cycle",
            )
            finishing_round = False
            for i, (x, y, dwell_s) in enumerate(waypoints, start=1):
                if stop_requested():
                    stop_reason = "stopped"
                    break
                if battery_low():
                    stop_reason = "battery"
                    log_task(
                        "battery_return",
                        cycle=cycle,
                        waypoint_id=i,
                        x=x,
                        y=y,
                        fire=fire_state(control_file),
                        soc=battery_soc(),
                        message="电量低于 5%，立即返回出发返回点",
                    )
                    break
                if duration_expired() and not finishing_round:
                    finishing_round = True
                    log_task(
                        "duration_finish_round",
                        cycle=cycle,
                        waypoint_id=i,
                        x=x,
                        y=y,
                        fire=fire_state(control_file),
                        soc=battery_soc(),
                        message="任务时长已到，先走完本圈剩余巡检点，再回出发返回点",
                    )
                    write_status(
                        status_file,
                        phase="navigating",
                        cycle=cycle,
                        index=i,
                        total=len(waypoints),
                        x=x,
                        y=y,
                        message="任务时长已到，先走完本圈剩余巡检点，再回出发返回点",
                    )
                log_task(
                    "navigate",
                    cycle=cycle,
                    waypoint_id=i,
                    x=x,
                    y=y,
                    dwell_s=dwell_s,
                    fire=fire_state(control_file),
                    soc=battery_soc(),
                )
                result = navigate(x, y, "navigating", cycle, i, len(waypoints))
                if result == "battery":
                    stop_reason = "battery"
                    break
                if result == "stop":
                    stop_reason = "stopped"
                    break
                if result == "timeout":
                    print(f"[cycle {cycle}] [{i}/{len(waypoints)}] skipped map=({x}, {y}): no path within {GOAL_TIMEOUT_S:g}s", flush=True)
                    write_status(
                        status_file,
                        phase="running",
                        cycle=cycle,
                        index=i,
                        total=len(waypoints),
                        x=x,
                        y=y,
                        message=f"Point {i} unreachable for {GOAL_TIMEOUT_S:g}s; skipping to the next point",
                    )
                    continue
                print(
                    f"[cycle {cycle}] [{i}/{len(waypoints)}] reached map=({x}, {y})",
                    flush=True,
                )
                write_status(
                    status_file,
                    phase="dwell" if dwell_s > 0 else "running",
                    cycle=cycle,
                    index=i,
                    total=len(waypoints),
                    x=x,
                    y=y,
                    dwell_s=dwell_s,
                    message=f"Reached point {i}; inspection dwell",
                )
                print(f"  reached ({x}, {y}); dwell {dwell_s:g}s", flush=True)
                log_task(
                    "reached",
                    cycle=cycle,
                    waypoint_id=i,
                    x=x,
                    y=y,
                    dwell_s=dwell_s,
                    fire=fire_state(control_file),
                    soc=battery_soc(),
                )
                if dwell_s > 0 and not wait_dwell(dwell_s, cycle, i):
                    stop_reason = "battery" if battery_low() else "stopped"
                    break

            if stop_reason != "completed":
                break
            if duration_expired() or not repeat_enabled:
                if duration_expired():
                    stop_reason = "duration"
                break
            next_cycle = min(repeat_interval_s or 0, max(0, deadline - time.monotonic())) if deadline else (repeat_interval_s or 0)
            if next_cycle <= 0:
                continue
            write_status(
                status_file,
                phase="waiting",
                cycle=cycle,
                index=len(waypoints),
                total=len(waypoints),
                message=f"Waiting {next_cycle:g}s for next patrol cycle",
            )
            wait_remaining = next_cycle
            while wait_remaining > 0:
                if stop_requested() or duration_expired():
                    if duration_expired():
                        stop_reason = "duration"
                    else:
                        stop_reason = "stopped"
                    break
                if read_control(control_file).get("paused"):
                    write_status(
                        status_file,
                        phase="paused_fire",
                        cycle=cycle,
                        index=len(waypoints),
                        total=len(waypoints),
                        message="Patrol paused between cycles for fire alarm; waiting for Safe confirmation",
                    )
                    time.sleep(0.1)
                    continue
                interval = min(0.1, wait_remaining)
                time.sleep(interval)
                wait_remaining -= interval
            if stop_reason != "completed":
                break

        finish_patrol(stop_reason, failure)
    except KeyboardInterrupt:
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            finish_patrol("stopped", "Patrol stopped by operator")
        finally:
            signal.signal(signal.SIGINT, previous_handler)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", type=Path, default=Path("patrol_route.json"))
    parser.add_argument("--auto", action="store_true", help="Start at route start_at instead of waiting for Enter")
    parser.add_argument("--immediate", action="store_true", help="Ignore any legacy route start_at and start after localization")
    parser.add_argument("--start-at", type=str, default=None, help="Override route schedule with a daily HH:MM start")
    parser.add_argument("--max-duration-s", type=float, default=None, help="Override route maximum duration for this run")
    parser.add_argument("--repeat-interval-s", type=float, default=None, help="Override route repeat interval for this run")
    parser.add_argument("--repeat-enabled", action="store_true", help="Repeat route cycles until the task duration expires")
    parser.add_argument("--single-run", action="store_true", help="Run exactly one route cycle")
    parser.add_argument("--control-file", type=Path, default=None)
    parser.add_argument("--status-file", type=Path, default=None)
    parser.add_argument("--transform-file", type=Path, default=None)
    args = parser.parse_args()
    waypoints, return_to_start, start_at, max_duration_s, repeat_interval_s = load_route(args.route)
    if args.immediate:
        start_at = None
    elif args.start_at is not None:
        start_at = args.start_at
    if args.max_duration_s is not None:
        max_duration_s = args.max_duration_s
    if args.repeat_interval_s is not None:
        repeat_interval_s = args.repeat_interval_s or None
    repeat_enabled = args.repeat_enabled or (bool(repeat_interval_s) and not args.single_run)
    if args.single_run:
        repeat_enabled = False
    if repeat_enabled and not max_duration_s:
        parser.error("continuous repeat requires a finite max_duration_s")
    write_status(args.status_file, phase="waiting_for_start", message="Localization accepted; applying route schedule")
    wait_until(start_at, args.auto, args.status_file, args.control_file)
    run_patrol(
        waypoints,
        return_to_start,
        max_duration_s,
        repeat_interval_s,
        args.status_file,
        args.transform_file,
        args.control_file,
        repeat_enabled,
    )
