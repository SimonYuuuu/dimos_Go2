#!/usr/bin/env python3
"""On-robot Go2 service: owns DimOS, mapping, navigation, alarms and local data."""
from __future__ import annotations

import hmac
import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from flask import Flask, jsonify, request, send_file

PROJECT = Path(__file__).resolve().parents[1]
SIM_ROOT = PROJECT / "simulation_legacy"
MAPS = SIM_ROOT / "maps"
ROUTES = PROJECT / "robot_data" / "routes"
DATA = PROJECT / "robot_data"
STATUS_FILE = DATA / "patrol-status.json"
CONTROL_FILE = DATA / "patrol-control.json"
EVENTS_FILE = DATA / "fire-alarm-events.jsonl"
MAPPING_LOG = DATA / "mapping.log"
PATROL_LOG = DATA / "patrol.log"
TOKEN = os.environ.get("ROBOT_AGENT_TOKEN", "")
ROBOT_IP = os.environ.get("ROBOT_IP", "")
AES_KEY = os.environ.get("UNITREE_AES_128_KEY", "")
DIMOS_MODE = os.environ.get("DIMOS_MODE", "real")
app = Flask(__name__)
lock = threading.RLock()
mapping_proc: subprocess.Popen[bytes] | None = None
patrol_proc: subprocess.Popen[bytes] | None = None

DATA.mkdir(parents=True, exist_ok=True)
ROUTES.mkdir(parents=True, exist_ok=True)
MAPS.mkdir(parents=True, exist_ok=True)


def _read_json(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temp.replace(path)


def _tail(path: Path, count: int = 100) -> list[str]:
    try:
        return path.read_text(encoding="utf-8", errors="replace").splitlines()[-count:]
    except OSError:
        return []


def _running(proc: subprocess.Popen[bytes] | None) -> bool:
    return proc is not None and proc.poll() is None


def _safe_name(value: object) -> str:
    name = Path(str(value or "")).name
    if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        raise ValueError("Name must use only letters, digits, underscores, and hyphens.")
    return name


def _control(**updates: object) -> dict[str, Any]:
    state = _read_json(CONTROL_FILE, {})
    if not isinstance(state, dict):
        state = {}
    state.update(updates)
    state["updated_at"] = datetime.now(timezone.utc).isoformat()
    _write_json(CONTROL_FILE, state)
    return state


def _event(state: str, source: str, detail: str = "") -> dict[str, str]:
    event = {
        "state": state,
        "source": source[:100],
        "detail": detail[:300],
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    with EVENTS_FILE.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, ensure_ascii=False) + "\n")
    control = _read_json(CONTROL_FILE, {})
    events = control.get("fire_events", []) if isinstance(control, dict) else []
    if not isinstance(events, list):
        events = []
    events.append(event)
    _control(fire_state=state, fire_source=source[:100], fire_events=events[-100:])
    return event


if not EVENTS_FILE.exists():
    _event("NaN", "no signal connected", "Detector has not reported a state")
if not CONTROL_FILE.exists():
    _write_json(CONTROL_FILE, {"paused": False, "stop_requested": False, "fire_state": "NaN"})


@app.before_request
def require_agent_token():
    if request.endpoint == "health":
        return None
    supplied = request.headers.get("Authorization", "")
    expected = f"Bearer {TOKEN}"
    if not TOKEN or not hmac.compare_digest(supplied, expected):
        return jsonify(error="Unauthorized robot-agent request."), 401
    return None


@app.get("/health")
def health():
    return jsonify(ok=True, service="dimos-go2-robot-agent", mode=DIMOS_MODE)


@app.get("/api/v1/status")
def status():
    control = _read_json(CONTROL_FILE, {})
    patrol = _read_json(STATUS_FILE, {"phase": "idle", "message": "No patrol status yet."})
    if not isinstance(control, dict):
        control = {}
    return jsonify(
        robot={"mode": DIMOS_MODE, "robot_ip": ROBOT_IP, "online": True},
        mapping={
            "running": _running(mapping_proc),
            "phase": "recording" if _running(mapping_proc) else "idle",
            "log": _tail(MAPPING_LOG, 60),
        },
        patrol={**patrol, "running": _running(patrol_proc), "log": _tail(PATROL_LOG, 80)},
        fire_alarm={
            "state": control.get("fire_state", "NaN"),
            "source": control.get("fire_source", "no signal connected"),
            "events": control.get("fire_events", []),
        },
    )


@app.get("/api/v1/events")
def events():
    try:
        after = max(0, int(request.args.get("after", "0")))
    except ValueError:
        return jsonify(error="after must be a non-negative integer."), 400
    try:
        lines = EVENTS_FILE.read_text(encoding="utf-8").splitlines()
    except OSError:
        lines = []
    items = []
    for index, line in enumerate(lines[after:], start=after):
        try:
            items.append({"cursor": index + 1, **json.loads(line)})
        except json.JSONDecodeError:
            continue
    return jsonify(cursor=len(lines), events=items)


@app.post("/api/v1/fire-signal")
def fire_signal():
    body = request.get_json(silent=True) or {}
    state = body.get("state")
    if state not in ("Fire_alarm", "Safe"):
        return jsonify(error="state must be Fire_alarm or Safe."), 400
    source = str(body.get("source", "detector"))[:100]
    detail = str(body.get("detail", ""))[:300]
    event = _event(state, source, detail)
    _control(paused=state == "Fire_alarm")
    if state == "Fire_alarm" and not _running(patrol_proc):
        event["detail"] = detail or "Alarm received while no patrol is active"
    return jsonify(ok=True, event=event)


@app.get("/api/v1/maps")
def maps():
    return jsonify(sorted(path.name for path in MAPS.glob("*.pc2.lcm")))


@app.get("/api/v1/maps/<name>/points")
def map_points(name: str):
    try:
        safe = Path(name).name
    except TypeError:
        return jsonify(error="Invalid map name."), 400
    path = MAPS / safe
    if not safe.endswith(".pc2.lcm") or not path.is_file():
        return jsonify(error="Map not found."), 404
    try:
        from pointcloud_preview import read_xy

        points = read_xy(path, max_points=30000)
    except Exception as error:  # Surface decoder errors to the operator for diagnosis.
        return jsonify(error=f"Could not decode map: {error}"), 422
    return jsonify(points.round(3).tolist())


@app.get("/api/v1/maps/<name>/artifact/<kind>")
def map_artifact(name: str, kind: str):
    safe = _safe_name(Path(name).stem)
    extensions = {"map": ".pc2.lcm", "recording": ".db", "rerun": ".rrd"}
    if kind not in extensions:
        return jsonify(error="kind must be map, recording, or rerun."), 400
    file_path = MAPS / f"{safe}{extensions[kind]}"
    if not file_path.is_file():
        return jsonify(error="Artifact not found."), 404
    return send_file(file_path, as_attachment=True, download_name=file_path.name)


@app.post("/api/v1/mapping/start")
def start_mapping():
    global mapping_proc
    body = request.get_json(silent=True) or {}
    try:
        name = _safe_name(body.get("name", "go2-map"))
    except ValueError as error:
        return jsonify(error=str(error)), 400
    if DIMOS_MODE != "real":
        return jsonify(error="Robot agent only accepts real-robot tasks; simulation stays in simulation_legacy."), 409
    if not ROBOT_IP:
        return jsonify(error="Configure ROBOT_IP on the robot agent host."), 503
    if not ROBOT_IP:
        return jsonify(error="Configure ROBOT_IP on the robot agent host."), 503
    with lock:
        if _running(mapping_proc) or _running(patrol_proc):
            return jsonify(error="Another robot task is already active."), 409
        env = os.environ.copy()
        env.pop("ROBOT_AGENT_TOKEN", None)
        env.update(DIMOS_MODE="real", ROBOT_IP=ROBOT_IP)
        if AES_KEY:
            env["UNITREE_AES_128_KEY"] = AES_KEY
        with MAPPING_LOG.open("wb") as log:
            mapping_proc = subprocess.Popen(
                ["bash", str(SIM_ROOT / "start_mapping.sh"), name],
                cwd=SIM_ROOT,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
    return jsonify(ok=True, pid=mapping_proc.pid, name=name)


@app.post("/api/v1/mapping/stop")
def stop_mapping():
    if not _running(mapping_proc):
        return jsonify(error="No mapping session is running."), 409
    os.killpg(mapping_proc.pid, signal.SIGINT)
    return jsonify(ok=True, message="Mapping stop requested; map optimization/export follows.")


@app.post("/api/v1/patrol/start")
def start_patrol():
    global patrol_proc
    body = request.get_json(silent=True) or {}
    try:
        map_name = Path(str(body.get("map", ""))).name
        route_name = _safe_name(body.get("route_name", "remote-route"))
    except ValueError as error:
        return jsonify(error=str(error)), 400
    if not (map_name.endswith(".pc2.lcm") and (MAPS / map_name).is_file()):
        return jsonify(error="Select an existing map on the robot."), 400
    route = body.get("route")
    if not isinstance(route, dict) or not route.get("waypoints"):
        return jsonify(error="Route must include at least one waypoint."), 400
    waypoints = route["waypoints"]
    if len(waypoints) > 100:
        return jsonify(error="Route may contain at most 100 waypoints."), 400
    home = route.get("return_to_start")
    if not isinstance(home, dict):
        return jsonify(error="Route must define a home point for departure and final return."), 400
    points = [home, *waypoints]
    try:
        for point in points:
            if not all(math.isfinite(float(point[key])) for key in ("x", "y")):
                raise ValueError
            if point in waypoints and not math.isfinite(float(point.get("dwell_s", 0))):
                raise ValueError
    except (KeyError, TypeError, ValueError):
        return jsonify(error="Home and waypoint x/y/dwell values must be finite numbers."), 400
    execution_mode = body.get("execution_mode", "immediate")
    start_at = str(body.get("start_at", ""))
    if execution_mode not in ("immediate", "daily"):
        return jsonify(error="execution_mode must be immediate or daily."), 400
    if execution_mode == "daily" and not re.fullmatch(r"(?:[01]\d|2[0-3]):[0-5]\d", start_at):
        return jsonify(error="Daily start_at must use HH:MM."), 400
    try:
        max_duration_s = float(body.get("max_duration_s", 1800))
        repeat_interval_s = float(body.get("repeat_interval_s", 0))
    except (TypeError, ValueError):
        return jsonify(error="Duration and repeat interval must be numeric."), 400
    repeat_enabled = body.get("repeat_enabled", False)
    if not isinstance(repeat_enabled, bool) or max_duration_s <= 0 or repeat_interval_s < 0:
        return jsonify(error="Invalid repeat mode, duration, or interval."), 400
    route_path = ROUTES / f"{route_name}.json"
    _write_json(route_path, route)
    with lock:
        if _running(mapping_proc) or _running(patrol_proc):
            return jsonify(error="Another robot task is already active."), 409
        env = os.environ.copy()
        env.pop("ROBOT_AGENT_TOKEN", None)
        env.update(
            DIMOS_MODE="real",
            ROBOT_IP=ROBOT_IP,
            PATROL_STATUS_FILE=str(STATUS_FILE),
            PATROL_CONTROL_FILE=str(CONTROL_FILE),
            PATROL_START_MODE=execution_mode,
            PATROL_START_AT=start_at if execution_mode == "daily" else "",
            PATROL_MAX_DURATION_S=str(max_duration_s),
            PATROL_REPEAT_INTERVAL_S=str(repeat_interval_s),
            PATROL_REPEAT_ENABLED="1" if repeat_enabled else "0",
        )
        if AES_KEY:
            env["UNITREE_AES_128_KEY"] = AES_KEY
        if body.get("initial_pose"):
            env["DIMOS_INITIAL_POSE"] = str(body["initial_pose"])
        current_control = _read_json(CONTROL_FILE, {})
        current_fire = current_control.get("fire_state", "NaN") if isinstance(current_control, dict) else "NaN"
        _control(stop_requested=False, paused=current_fire == "Fire_alarm")
        with PATROL_LOG.open("wb") as log:
            patrol_proc = subprocess.Popen(
                ["bash", str(SIM_ROOT / "start_patrol.sh"), str(route_path), str(MAPS / map_name)],
                cwd=SIM_ROOT,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
    return jsonify(ok=True, pid=patrol_proc.pid, route=route_name)


@app.post("/api/v1/patrol/stop")
def stop_patrol():
    if not _running(patrol_proc):
        return jsonify(error="No patrol task is running."), 409
    _control(paused=False, stop_requested=True)
    return jsonify(ok=True, message="Return-home stop requested.")


@app.post("/api/v1/patrol/pause")
def pause_patrol():
    if not _running(patrol_proc):
        return jsonify(error="No patrol task is running."), 409
    _event("Fire_alarm", "remote operator", "Pause requested from remote console")
    _control(paused=True)
    return jsonify(ok=True, state="Fire_alarm")


@app.post("/api/v1/patrol/resume")
def resume_patrol():
    if not _running(patrol_proc):
        return jsonify(error="No patrol task is running."), 409
    _event("Safe", "remote operator", "Operator confirmed scene is safe")
    _control(paused=False)
    return jsonify(ok=True, state="Safe")


@app.get("/api/v1/logs")
def logs():
    return jsonify(mapping=_tail(MAPPING_LOG), patrol=_tail(PATROL_LOG))


if __name__ == "__main__":
    host = os.environ.get("ROBOT_AGENT_HOST", "0.0.0.0")
    port = int(os.environ.get("ROBOT_AGENT_PORT", "8787"))
    if DIMOS_MODE == "real" and not TOKEN:
        raise SystemExit("ROBOT_AGENT_TOKEN is required; configure it in the robot service environment.")
    app.run(host=host, port=port, debug=False, threaded=True)
