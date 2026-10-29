#!/usr/bin/env python3
"""Local scheduling and monitoring UI for Go2 mapping and patrol operations."""

from __future__ import annotations

import json
import os
import re
import signal
import socket
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from flask import Flask, jsonify, request, send_file
from pointcloud_preview import read_xy

ROOT = Path(__file__).resolve().parent
MAPS = ROOT / "maps"
ROUTES = ROOT / "routes"
STATUS_FILE = MAPS / "patrol-status.json"
CONTROL_FILE = MAPS / "patrol-control.json"
FIRE_LOG_FILE = MAPS / "fire-alarm-events.jsonl"
PAGE_FILE = ROOT / "patrol_console.html"
app = Flask(__name__)
lock = threading.Lock()
mapping_proc: subprocess.Popen[bytes] | None = None
patrol_proc: subprocess.Popen[bytes] | None = None
mapping_log_path: Path | None = None
patrol_log_path: Path | None = None


def _safe_name(value: str) -> str:
    value = Path(value).name
    if not re.fullmatch(r"[A-Za-z0-9_-]+(?:\.json)?", value):
        raise ValueError("Use letters, digits, underscores and hyphens for file names.")
    return value


def _route_path(name: str) -> Path:
    name = _safe_name(name)
    if not name.endswith(".json"):
        name += ".json"
    candidate = ROUTES / name
    if candidate.exists():
        return candidate
    if name == "patrol_route.json" and (ROOT / name).exists():
        return ROOT / name
    return candidate


def _available_map_names() -> set[str]:
    return {
        path.name
        for path in MAPS.glob("*.pc2.lcm")
        if not path.name.startswith("acceptance-")
    }


def _available_route_names() -> set[str]:
    names = {
        path.name for path in ROUTES.glob("*.json") if not path.name.startswith("acceptance-")
    }
    if (ROOT / "patrol_route.json").exists():
        names.add("patrol_route.json")
    return names


def _tail(path: Path | None, limit: int = 25) -> list[str]:
    if path is None or not path.exists():
        return []
    return path.read_text(errors="replace").splitlines()[-limit:]


def _process_running(proc: subprocess.Popen[bytes] | None) -> bool:
    return proc is not None and proc.poll() is None


def _update_patrol_control(**updates: object) -> None:
    try:
        state = json.loads(CONTROL_FILE.read_text())
        if not isinstance(state, dict):
            state = {}
    except (OSError, json.JSONDecodeError):
        state = {}
    state.update(updates)
    state["updated_at"] = time.time()
    temporary = CONTROL_FILE.with_suffix(CONTROL_FILE.suffix + ".tmp")
    temporary.write_text(json.dumps(state) + "\n")
    temporary.replace(CONTROL_FILE)


def _record_fire_state(state_name: str, source: str) -> dict[str, object]:
    event: dict[str, object] = {
        "state": state_name,
        "source": source,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    try:
        control = json.loads(CONTROL_FILE.read_text())
        if not isinstance(control, dict):
            control = {}
    except (OSError, json.JSONDecodeError):
        control = {}
    events = control.get("fire_events", [])
    if not isinstance(events, list):
        events = []
    events.append(event)
    _update_patrol_control(fire_state=state_name, fire_source=source, fire_events=events[-100:])
    MAPS.mkdir(exist_ok=True)
    with FIRE_LOG_FILE.open("a", encoding="utf-8") as log:
        log.write(json.dumps(event, ensure_ascii=False) + "\n")
    return event


def _occupied_service_ports() -> list[int]:
    occupied: list[int] = []
    for port in (7779, 9877, 9878, 3030):
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.15):
                occupied.append(port)
        except OSError:
            pass
    return occupied


def _lcm_network_error() -> str | None:
    try:
        link = subprocess.run(
            ["ip", "-o", "link", "show", "dev", "lo"],
            capture_output=True,
            text=True,
            check=False,
        )
        route = subprocess.run(
            ["ip", "route", "show", "224.0.0.0/4", "dev", "lo"],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError as error:
        return f"Cannot inspect LCM loopback networking: {error}"
    missing: list[str] = []
    if link.returncode != 0 or "MULTICAST" not in link.stdout:
        missing.append("enable loopback multicast")
    if route.returncode != 0 or not route.stdout.strip():
        missing.append("add the loopback multicast route")
    if not missing:
        return None
    return (
        "LCM loopback networking is not configured ("
        + " and ".join(missing)
        + "). Configure it once on the Linux host with administrator privileges; "
        "this VS Code/container environment may block sudo: "
        "sudo ip link set lo multicast on && "
        "sudo ip route replace 224.0.0.0/4 dev lo && "
        "sudo sysctl -w net.core.rmem_max=67108864 && "
        "sudo sysctl -w net.core.rmem_default=67108864"
    )


@app.get("/")
def index() -> Any:
    return send_file(PAGE_FILE)


@app.get("/api/maps")
def list_maps() -> Any:
    MAPS.mkdir(exist_ok=True)
    return jsonify(sorted(_available_map_names()))


@app.get("/api/routes")
def list_routes() -> Any:
    ROUTES.mkdir(exist_ok=True)
    return jsonify(sorted(_available_route_names()))


@app.get("/api/map-points")
def map_points() -> Any:
    name = Path(request.args.get("map", "")).name
    if not name.endswith(".pc2.lcm"):
        return jsonify(error="Select a .pc2.lcm map."), 400
    path = MAPS / name
    if not path.is_file():
        return jsonify(error="Selected map does not exist."), 404
    try:
        points = read_xy(path)
    except Exception as error:
        return jsonify(error=f"Could not decode map: {error}"), 422
    if not len(points):
        return jsonify(error="Selected map is empty."), 422
    return jsonify(points.round(3).tolist())


@app.get("/api/route")
def get_route() -> Any:
    try:
        path = _route_path(request.args.get("name", "patrol_route.json"))
        return jsonify(json.loads(path.read_text()))
    except (ValueError, OSError, json.JSONDecodeError) as error:
        return jsonify(error=str(error)), 400


@app.post("/api/route")
def save_route() -> Any:
    body = request.get_json(force=True)
    route = body.get("route", {})
    if not route.get("waypoints"):
        return jsonify(error="Add at least one inspection point."), 400
    try:
        name = _safe_name(body.get("name", "patrol_route"))
    except ValueError as error:
        return jsonify(error=str(error)), 400
    if not name.endswith(".json"):
        name += ".json"
    ROUTES.mkdir(exist_ok=True)
    path = ROUTES / name
    path.write_text(json.dumps(route, indent=2) + "\n")
    return jsonify(ok=True, name=name)


@app.post("/api/mapping/start")
def start_mapping() -> Any:
    global mapping_proc, mapping_log_path
    body = request.get_json(force=True)
    mode = body.get("mode", "simulation")
    name = body.get("name", "warehouse")
    if mode not in ("simulation", "real"):
        return jsonify(error="Mode must be simulation or real."), 400
    try:
        name = _safe_name(name)
    except ValueError as error:
        return jsonify(error=str(error)), 400
    if mode == "real" and not body.get("robot_ip"):
        return jsonify(error="Enter the robot IP for real-robot mapping."), 400
    lcm_error = _lcm_network_error()
    if lcm_error:
        return jsonify(error=lcm_error), 409
    if _occupied_service_ports():
        return jsonify(error=f"DimOS/Rerun ports already in use: {_occupied_service_ports()}. Stop the existing run first."), 409
    with lock:
        if _process_running(mapping_proc):
            return jsonify(error="Mapping is already running."), 409
        if _process_running(patrol_proc):
            return jsonify(error="Stop the patrol before starting mapping."), 409
        MAPS.mkdir(exist_ok=True)
        mapping_log_path = MAPS / f"{name}.mapping.log"
        log = mapping_log_path.open("ab")
        env = os.environ.copy()
        env["DIMOS_MODE"] = mode
        if body.get("robot_ip"):
            env["ROBOT_IP"] = body["robot_ip"]
        mapping_proc = subprocess.Popen(
            [str(ROOT / "start_mapping.sh"), name],
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        log.close()
    return jsonify(ok=True, pid=mapping_proc.pid, log=str(mapping_log_path))


@app.post("/api/mapping/stop")
def stop_mapping() -> Any:
    with lock:
        if not _process_running(mapping_proc):
            return jsonify(error="No mapping session is running."), 409
        os.killpg(mapping_proc.pid, signal.SIGINT)
    return jsonify(ok=True, message="Stopping recorder; map export will run automatically.")


@app.post("/api/patrol/start")
def start_patrol() -> Any:
    global patrol_proc, patrol_log_path
    body = request.get_json(force=True)
    mode = body.get("mode", "simulation")
    if mode not in ("simulation", "real"):
        return jsonify(error="Mode must be simulation or real."), 400
    if mode == "real" and not body.get("robot_ip"):
        return jsonify(error="Enter the robot IP for a real-robot patrol."), 400
    execution_mode = body.get("execution_mode", "immediate")
    if execution_mode not in ("immediate", "daily"):
        return jsonify(error="Execution mode must be immediate or daily."), 400
    start_at = body.get("start_at", "")
    if execution_mode == "daily" and not re.fullmatch(r"(?:[01]\d|2[0-3]):[0-5]\d", start_at):
        return jsonify(error="Choose a valid daily start time (HH:MM)."), 400
    try:
        max_duration_s = float(body.get("max_duration_s", 1800))
        repeat_interval_s = float(body.get("repeat_interval_s", 0))
    except (TypeError, ValueError):
        return jsonify(error="Duration and repeat interval must be numeric."), 400
    if max_duration_s <= 0 or repeat_interval_s < 0:
        return jsonify(error="Duration must be positive and repeat interval cannot be negative."), 400
    lcm_error = _lcm_network_error()
    if lcm_error:
        return jsonify(error=lcm_error), 409
    repeat_enabled = body.get("repeat_enabled", False)
    if not isinstance(repeat_enabled, bool):
        return jsonify(error="repeat_enabled must be true or false."), 400
    if _occupied_service_ports():
        return jsonify(error=f"DimOS/Rerun ports already in use: {_occupied_service_ports()}. Stop the existing run first."), 409
    try:
        map_name = Path(body.get("map", "")).name
        route_path = _route_path(body.get("route", "patrol_route.json"))
    except ValueError as error:
        return jsonify(error=str(error)), 400
    if map_name not in _available_map_names():
        return jsonify(error="Selected map is not an available production map."), 400
    if route_path.name not in _available_route_names():
        return jsonify(error="Selected route is not an available production route."), 400
    map_path = MAPS / map_name
    if not map_name.endswith(".pc2.lcm") or not map_path.is_file():
        return jsonify(error="Selected map file does not exist."), 400
    if not route_path.is_file():
        return jsonify(error="Selected route file does not exist."), 400
    with lock:
        if _process_running(patrol_proc):
            return jsonify(error="A patrol task is already running."), 409
        if _process_running(mapping_proc):
            return jsonify(error="Stop mapping and export its map before patrolling."), 409
        patrol_log_path = MAPS / "patrol.log"
        log = patrol_log_path.open("wb")
        env = os.environ.copy()
        env["DIMOS_MODE"] = mode
        env["PATROL_STATUS_FILE"] = str(STATUS_FILE)
        env["PATROL_CONTROL_FILE"] = str(CONTROL_FILE)
        env["PATROL_START_MODE"] = execution_mode
        env["PATROL_START_AT"] = start_at if execution_mode == "daily" else ""
        env["PATROL_MAX_DURATION_S"] = str(max_duration_s)
        env["PATROL_REPEAT_INTERVAL_S"] = str(repeat_interval_s)
        env["PATROL_REPEAT_ENABLED"] = "1" if repeat_enabled else "0"
        if body.get("robot_ip"):
            env["ROBOT_IP"] = body["robot_ip"]
        if body.get("initial_pose"):
            env["DIMOS_INITIAL_POSE"] = body["initial_pose"]
        try:
            prior_control = json.loads(CONTROL_FILE.read_text())
            if not isinstance(prior_control, dict):
                prior_control = {}
        except (OSError, json.JSONDecodeError):
            prior_control = {}
        fire_state = prior_control.get("fire_state", "NaN")
        if fire_state not in ("NaN", "Fire_alarm", "Safe"):
            fire_state = "NaN"
        fire_source = str(prior_control.get("fire_source", "no signal connected"))
        fire_events = prior_control.get("fire_events", [])
        _update_patrol_control(
            paused=fire_state == "Fire_alarm",
            stop_requested=False,
            fire_state=fire_state,
            fire_source=fire_source,
            fire_events=fire_events if isinstance(fire_events, list) else [],
        )
        if fire_state == "NaN":
            _record_fire_state("NaN", "no signal connected")
        patrol_proc = subprocess.Popen(
            [str(ROOT / "start_patrol.sh"), str(route_path), str(map_path)],
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        log.close()
    return jsonify(ok=True, pid=patrol_proc.pid, log=str(patrol_log_path))


@app.post("/api/patrol/stop")
def stop_patrol() -> Any:
    with lock:
        if not _process_running(patrol_proc):
            return jsonify(error="No patrol task is running."), 409
        _update_patrol_control(paused=False, stop_requested=True)
    return jsonify(ok=True, message="Stop requested; robot will return to the route home point before shutdown.")


@app.post("/api/patrol/pause")
@app.post("/api/patrol/fire-alarm")
def pause_patrol() -> Any:
    with lock:
        if not _process_running(patrol_proc):
            return jsonify(error="No patrol task is running."), 409
        _record_fire_state("Fire_alarm", "operator/API")
        _update_patrol_control(paused=True, stop_requested=False)
    return jsonify(ok=True, message="Patrol pause requested; navigation will stop and wait for operator confirmation.")


@app.post("/api/patrol/resume")
@app.post("/api/patrol/safe")
def resume_patrol() -> Any:
    with lock:
        if not _process_running(patrol_proc):
            return jsonify(error="No patrol task is running."), 409
        _record_fire_state("Safe", "operator confirmation/API")
        _update_patrol_control(paused=False)
    return jsonify(ok=True, message="Operator confirmed the alarm is cleared; patrol will resume.")


@app.post("/api/fire-signal")
def fire_signal() -> Any:
    body = request.get_json(force=True)
    state_name = body.get("state")
    if state_name not in ("Fire_alarm", "Safe"):
        return jsonify(error="state must be Fire_alarm or Safe."), 400
    source = str(body.get("source", "detector"))[:80]
    with lock:
        _record_fire_state(state_name, source)
        _update_patrol_control(paused=state_name == "Fire_alarm")
    return jsonify(ok=True, state=state_name)


@app.get("/api/status")
def status() -> Any:
    try:
        patrol = json.loads(STATUS_FILE.read_text())
    except (OSError, json.JSONDecodeError):
        patrol = {"phase": "idle", "message": "No patrol status yet."}
    try:
        localization = json.loads((MAPS / "map-to-world.json").read_text())
    except (OSError, json.JSONDecodeError):
        localization = None
    mapping_log = _tail(mapping_log_path)
    patrol_log = _tail(patrol_log_path, limit=80)
    patrol_running = _process_running(patrol_proc)
    patrol_phase = patrol.get("phase", "idle")
    active_phases = {
        "localizing",
        "initializing",
        "waiting_for_localization",
        "waiting_for_costmap",
        "waiting_for_start",
        "scheduled",
        "navigating",
        "dwell",
        "returning",
        "waiting",
        "paused_fire",
    }
    if not patrol_running and patrol_phase in active_phases:
        exit_code = patrol_proc.poll() if patrol_proc is not None else None
        diagnostic = next(
            (
                line.strip()
                for line in reversed(patrol_log)
                if "CalledProcessError:" in line or "Error:" in line or "No such file" in line
            ),
            "Patrol launcher exited before localization completed.",
        )
        patrol = {
            **patrol,
            "phase": "error",
            "message": f"Patrol process is no longer running (exit {exit_code}): {diagnostic}",
            "exit_code": exit_code,
        }
        patrol_phase = "error"
    try:
        control = json.loads(CONTROL_FILE.read_text())
        if not isinstance(control, dict):
            control = {}
    except (OSError, json.JSONDecodeError):
        control = {}
    mapping_stage = "recording" if _process_running(mapping_proc) else "idle"
    if _process_running(mapping_proc) and any("Building optimized map" in line for line in mapping_log):
        mapping_stage = "exporting"
    elif not _process_running(mapping_proc) and any("Map saved:" in line for line in mapping_log):
        mapping_stage = "map_ready"
    return jsonify(
        fire_alarm={
            "state": control.get("fire_state", "NaN"),
            "source": control.get("fire_source", "no signal connected"),
            "events": control.get("fire_events", []),
        },
        mapping={
            "running": _process_running(mapping_proc),
            "phase": mapping_stage,
            "log": mapping_log,
        },
        patrol={
            **patrol,
            "running": patrol_running,
            "log": patrol_log,
            "map_to_world": localization,
        },
    )


if __name__ == "__main__":
    MAPS.mkdir(exist_ok=True)
    ROUTES.mkdir(exist_ok=True)
    host = os.environ.get("PATROL_UI_HOST", "127.0.0.1")
    app.run(host=host, port=7781, debug=False)