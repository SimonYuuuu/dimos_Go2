#!/usr/bin/env python3
"""Remote operator console; proxies commands to the on-robot agent and archives telemetry."""
from __future__ import annotations

import json
import os
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

from flask import Flask, jsonify, request, send_file
from werkzeug.utils import secure_filename

PROJECT = Path(__file__).resolve().parents[1]
ROBOT_URL = os.environ.get("ROBOT_AGENT_URL", "").rstrip("/")
ROBOT_TOKEN = os.environ.get("ROBOT_AGENT_TOKEN", "")
ROBOT_COMMAND_CENTER_URL = os.environ.get("ROBOT_COMMAND_CENTER_URL", "")
ROBOT_RERUN_URL = os.environ.get("ROBOT_RERUN_URL", "")
ROUTES = PROJECT / "remote_client" / "routes"
DATA = PROJECT / "remote_client" / "data"
LOGS = PROJECT / "logs"
PAGE = PROJECT / "remote_client" / "console.html"
CURSOR_FILE = DATA / "event-cursor.json"
app = Flask(__name__)
sync_lock = threading.Lock()
sync_started = False
sync_start_lock = threading.Lock()
last_status_log = 0.0
ROUTES.mkdir(parents=True, exist_ok=True)
DATA.mkdir(parents=True, exist_ok=True)
LOGS.mkdir(parents=True, exist_ok=True)


def _agent(path: str, method: str = "GET", payload: Any = None, timeout: float = 15.0) -> tuple[Any, int]:
    if not ROBOT_URL or not ROBOT_TOKEN:
        return {"error": "Set ROBOT_AGENT_URL and ROBOT_AGENT_TOKEN on the remote client."}, 503
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Authorization": f"Bearer {ROBOT_TOKEN}", "Accept": "application/json"}
    if data is not None:
        headers["Content-Type"] = "application/json"
    url = f"{ROBOT_URL}/api/v1/{path.lstrip('/')}"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            raw = response.read()
            content_type = response.headers.get("Content-Type", "")
            if "json" in content_type:
                return json.loads(raw or b"{}"), response.status
            return raw, response.status
    except urllib.error.HTTPError as error:
        try:
            body = json.loads(error.read() or b"{}")
        except (json.JSONDecodeError, OSError):
            body = {"error": f"Robot agent returned HTTP {error.code}."}
        return body, error.code
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        return {"error": f"Robot agent unavailable: {error}"}, 502


def _append_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("a", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False) + "\n")


def sync_once() -> None:
    """Pull events and map artifacts to the remote computer for audit/archive."""
    with sync_lock:
        state = {}
        try:
            state = json.loads(CURSOR_FILE.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            pass
        cursor = int(state.get("cursor", 0)) if isinstance(state, dict) else 0
        event_result, code = _agent(f"events?after={cursor}")
        if code == 200 and isinstance(event_result, dict):
            new_events = event_result.get("events", [])
            if isinstance(new_events, list):
                _append_jsonl(LOGS / "robot-events.jsonl", new_events)
            next_cursor = int(event_result.get("cursor", cursor))
            temp = CURSOR_FILE.with_suffix(".tmp")
            temp.write_text(json.dumps({"cursor": next_cursor}) + "\n", encoding="utf-8")
            temp.replace(CURSOR_FILE)
        maps_result, map_code = _agent("maps")
        if map_code != 200 or not isinstance(maps_result, list):
            return
        archive = DATA / "maps"
        archive.mkdir(parents=True, exist_ok=True)
        for map_name in maps_result:
            if not isinstance(map_name, str) or not re.fullmatch(r"[A-Za-z0-9_-]+\.pc2\.lcm", map_name):
                continue
            stem = map_name.removesuffix(".pc2.lcm")
            for kind, suffix in (("map", ".pc2.lcm"), ("recording", ".db"), ("rerun", ".rrd")):
                target = archive / f"{stem}{suffix}"
                if target.exists():
                    continue
                if not ROBOT_URL or not ROBOT_TOKEN:
                    continue
                url = f"{ROBOT_URL}/api/v1/maps/{urllib.parse.quote(stem)}/artifact/{kind}"
                req = urllib.request.Request(url, headers={"Authorization": f"Bearer {ROBOT_TOKEN}"})
                try:
                    with urllib.request.urlopen(req, timeout=45) as response:
                        temp = target.with_suffix(target.suffix + ".tmp")
                        temp.write_bytes(response.read())
                        temp.replace(target)
                except (urllib.error.URLError, TimeoutError, OSError):
                    continue


def _sync_loop() -> None:
    while True:
        sync_once()
        time.sleep(5)


def _start_sync_thread() -> None:
    global sync_started
    with sync_start_lock:
        if sync_started:
            return
        sync_started = True
        threading.Thread(target=_sync_loop, daemon=True, name="robot-data-sync").start()


def _route_path(name: object) -> Path:
    safe = secure_filename(str(name or ""))
    if not safe:
        raise ValueError("Provide a route name.")
    if not safe.endswith(".json"):
        safe += ".json"
    return ROUTES / safe


@app.get("/")
def index():
    return send_file(PAGE)


@app.get("/command-center")
def command_center():
    if not ROBOT_COMMAND_CENTER_URL:
        return "Set ROBOT_COMMAND_CENTER_URL to the private-network Command Center URL.", 503
    from flask import redirect

    return redirect(ROBOT_COMMAND_CENTER_URL)


@app.get("/rerun")
def rerun_viewer():
    if not ROBOT_RERUN_URL:
        return "Set ROBOT_RERUN_URL to the private-network Rerun viewer URL.", 503
    from flask import redirect

    return redirect(ROBOT_RERUN_URL)


@app.get("/api/status")
def status():
    global last_status_log
    _start_sync_thread()
    result, code = _agent("status")
    if code != 200:
        return jsonify(error=result.get("error", "Robot agent unavailable.")), code
    now = time.time()
    if now - last_status_log >= 15:
        _append_jsonl(LOGS / "robot-status.jsonl", [{"received_at": now, **result}])
        last_status_log = now
    result.setdefault("mapping", {}).setdefault("log", [])
    result.setdefault("patrol", {}).setdefault("log", [])
    return jsonify(result)


@app.get("/api/maps")
def list_maps():
    result, code = _agent("maps")
    if code != 200:
        return jsonify(error=result.get("error", "Robot agent unavailable.")), code
    return jsonify(result)


@app.get("/api/map-points")
def map_points():
    name = request.args.get("map", "")
    if not re.fullmatch(r"[A-Za-z0-9_-]+\.pc2\.lcm", name):
        return jsonify(error="Select a valid point-cloud map."), 400
    result, code = _agent(f"maps/{urllib.parse.quote(name)}/points")
    if code != 200:
        return jsonify(error=result.get("error", "Could not load map.")), code
    return jsonify(result)


@app.get("/api/routes")
def list_routes():
    ROUTES.mkdir(parents=True, exist_ok=True)
    return jsonify(sorted(path.name for path in ROUTES.glob("*.json")))


@app.get("/api/route")
def get_route():
    try:
        path = _route_path(request.args.get("name", ""))
        return jsonify(json.loads(path.read_text(encoding="utf-8")))
    except (ValueError, OSError, json.JSONDecodeError) as error:
        return jsonify(error=str(error)), 400


@app.post("/api/route")
def save_route():
    body = request.get_json(silent=True) or {}
    route = body.get("route")
    if not isinstance(route, dict) or not route.get("waypoints"):
        return jsonify(error="Add at least one inspection waypoint."), 400
    try:
        path = _route_path(body.get("name", "route"))
    except ValueError as error:
        return jsonify(error=str(error)), 400
    path.write_text(json.dumps(route, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return jsonify(ok=True, name=path.name)


@app.post("/api/mapping/start")
def start_mapping():
    body = request.get_json(silent=True) or {}
    result, code = _agent("mapping/start", "POST", {"name": body.get("name", "go2-map")})
    return jsonify(result), code


@app.post("/api/mapping/stop")
def stop_mapping():
    result, code = _agent("mapping/stop", "POST", {})
    return jsonify(result), code


@app.post("/api/patrol/start")
def start_patrol():
    body = request.get_json(silent=True) or {}
    try:
        route_path = _route_path(body.get("route", ""))
        route = json.loads(route_path.read_text(encoding="utf-8"))
    except (ValueError, OSError, json.JSONDecodeError) as error:
        return jsonify(error=f"Load and save a route before starting: {error}"), 400
    payload = {
        "map": body.get("map"),
        "route_name": route_path.stem,
        "route": route,
        "execution_mode": body.get("execution_mode", "immediate"),
        "start_at": body.get("start_at", ""),
        "max_duration_s": body.get("max_duration_s", 1800),
        "repeat_enabled": body.get("repeat_enabled", False),
        "repeat_interval_s": body.get("repeat_interval_s", 0),
        "initial_pose": body.get("initial_pose", ""),
    }
    result, code = _agent("patrol/start", "POST", payload)
    return jsonify(result), code


@app.post("/api/patrol/stop")
def stop_patrol():
    result, code = _agent("patrol/stop", "POST", {})
    return jsonify(result), code


@app.post("/api/patrol/fire-alarm")
def fire_alarm():
    result, code = _agent("patrol/pause", "POST", {})
    return jsonify(result), code


@app.post("/api/patrol/pause")
def pause_patrol():
    return fire_alarm()


@app.post("/api/patrol/resume")
def resume_patrol():
    result, code = _agent("patrol/resume", "POST", {})
    return jsonify(result), code


@app.get("/api/logs")
def logs():
    return jsonify(
        events=_tail_local(LOGS / "robot-events.jsonl"),
        status=_tail_local(LOGS / "robot-status.jsonl"),
    )


def _tail_local(path: Path, count: int = 100) -> list[str]:
    try:
        return path.read_text(encoding="utf-8", errors="replace").splitlines()[-count:]
    except OSError:
        return []


if __name__ == "__main__":
    host = os.environ.get("REMOTE_CONSOLE_HOST", "127.0.0.1")
    port = int(os.environ.get("REMOTE_CONSOLE_PORT", "7781"))
    _start_sync_thread()
    app.run(host=host, port=port, debug=False, threaded=True)
