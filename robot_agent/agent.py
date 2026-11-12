#!/usr/bin/env python3
"""On-robot Go2 service: owns DimOS, mapping, navigation, alarms and local data."""
from __future__ import annotations

import hmac
import json
import math
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
LIDAR_IP = os.environ.get("LIDAR_IP", "192.168.123.20")
# DimOS 0.0.15 rejects UNITREE_AES_128_KEY; the module env is GO2CONNECTION__AES_128_KEY.
AES_KEY = os.environ.get("GO2CONNECTION__AES_128_KEY") or os.environ.get("UNITREE_AES_128_KEY", "")
DIMOS_MODE = os.environ.get("DIMOS_MODE", "real")
DIMOS_BIN = SIM_ROOT / ".venv" / "bin" / "dimos"
app = Flask(__name__)
lock = threading.RLock()
mapping_proc: subprocess.Popen[bytes] | None = None
mapping_name: str | None = None
patrol_proc: subprocess.Popen[bytes] | None = None
_reachability: dict[str, Any] = {"checked_at": 0.0}
_export_lock = threading.Lock()
_export_status: dict[str, Any] = {"name": None, "running": False, "error": None, "path": None}

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


def _dimos_stop() -> None:
    if not DIMOS_BIN.is_file():
        return
    try:
        subprocess.run(
            [str(DIMOS_BIN), "stop"],
            cwd=str(SIM_ROOT),
            timeout=60,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.TimeoutExpired):
        pass


def _export_map_async(name: str) -> None:
    db = MAPS / f"{name}.db"
    out = MAPS / f"{name}.pc2.lcm"
    if not db.is_file():
        with _export_lock:
            _export_status.update(name=name, running=False, error=f"missing {db.name}", path=None)
        return
    if out.is_file():
        with _export_lock:
            _export_status.update(name=name, running=False, error=None, path=str(out))
        return
    with _export_lock:
        if _export_status.get("running") and _export_status.get("name") == name:
            return
        _export_status.update(name=name, running=True, error=None, path=None)

    def worker() -> None:
        try:
            env = os.environ.copy()
            gomp = "/lib/aarch64-linux-gnu/libgomp.so.1"
            if Path(gomp).is_file():
                preload = env.get("LD_PRELOAD", "")
                if gomp not in preload.split(":"):
                    env["LD_PRELOAD"] = f"{gomp}:{preload}" if preload else gomp
            with MAPPING_LOG.open("ab") as log:
                log.write(f"\n=== export_map {name} ===\n".encode())
                completed = subprocess.run(
                    ["bash", str(SIM_ROOT / "export_map.sh"), str(db), str(out)],
                    cwd=str(SIM_ROOT),
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
            err = None if completed.returncode == 0 and out.is_file() else f"export exit {completed.returncode}"
            with _export_lock:
                _export_status.update(
                    name=name,
                    running=False,
                    error=err,
                    path=str(out) if out.is_file() else None,
                )
        except OSError as error:
            with _export_lock:
                _export_status.update(name=name, running=False, error=str(error), path=None)

    threading.Thread(target=worker, name=f"export-{name}", daemon=True).start()


def _safe_name(value: object) -> str:
    name = Path(str(value or "")).name
    if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        raise ValueError("Name must use only letters, digits, underscores, and hyphens.")
    return name


def _token_ok(header: str) -> bool:
    prefix = "Bearer "
    if not TOKEN or not header.startswith(prefix):
        return False
    supplied = header[len(prefix) :]
    if len(supplied) != len(TOKEN):
        return False
    return hmac.compare_digest(supplied, TOKEN)


def _tcp_open(host: str, port: int, timeout: float = 0.35) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _ping(ip: str) -> bool | None:
    if not ip:
        return None
    try:
        completed = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2,
            check=False,
        )
        return completed.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return None


def _connectivity() -> dict[str, Any]:
    now = time.time()
    cached = _reachability
    if now - float(cached.get("checked_at") or 0) < 8:
        return {k: v for k, v in cached.items() if k != "checked_at"}
    snapshot = {
        "robot_ip": ROBOT_IP,
        "robot_reachable": _ping(ROBOT_IP),
        "lidar_ip": LIDAR_IP,
        "lidar_reachable": _ping(LIDAR_IP),
        "dimos_installed": DIMOS_BIN.is_file(),
        "command_center": _tcp_open("127.0.0.1", 7779),
        "rerun": _tcp_open("127.0.0.1", 9878),
        "webrtc_signaling": bool(ROBOT_IP)
        and (_tcp_open(ROBOT_IP, 9991) or _tcp_open(ROBOT_IP, 8081)),
    }
    cached.clear()
    cached.update(checked_at=now, **snapshot)
    return snapshot


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
    if not _token_ok(request.headers.get("Authorization", "")):
        return jsonify(error="Unauthorized robot-agent request."), 401
    return None


@app.get("/health")
def health():
    return jsonify(ok=True, service="dimos-go2-robot-agent", mode=DIMOS_MODE, robot_ip=ROBOT_IP)


@app.get("/api/v1/status")
def status():
    control = _read_json(CONTROL_FILE, {})
    patrol = _read_json(STATUS_FILE, {"phase": "idle", "message": "No patrol status yet."})
    if not isinstance(control, dict):
        control = {}
    connectivity = _connectivity()
    return jsonify(
        robot={
            "mode": DIMOS_MODE,
            "robot_ip": ROBOT_IP,
            "online": bool(connectivity.get("robot_reachable")),
        },
        connectivity=connectivity,
        mapping={
            "running": _running(mapping_proc),
            "name": mapping_name,
            "phase": (
                "recording"
                if _running(mapping_proc)
                else ("exporting" if _export_status.get("running") else "idle")
            ),
            "export": dict(_export_status),
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
        from pointcloud_preview import read_xyz

        points = read_xyz(path, max_points=30000)
    except Exception as error:  # Surface decoder errors to the operator for diagnosis.
        return jsonify(error=f"Could not decode map: {error}"), 422
    return jsonify(points=points.round(3).tolist(), format="xyz")


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
    global mapping_proc, mapping_name
    body = request.get_json(silent=True) or {}
    try:
        name = _safe_name(body.get("name", "go2-map"))
    except ValueError as error:
        return jsonify(error=str(error)), 400
    if DIMOS_MODE != "real":
        return jsonify(error="Robot agent only accepts real-robot tasks; simulation stays in simulation_legacy."), 409
    if not ROBOT_IP:
        return jsonify(error="Configure ROBOT_IP on the robot agent host."), 503
    if not DIMOS_BIN.is_file():
        return jsonify(error="DimOS is not installed in simulation_legacy/.venv on the robot."), 503
    with lock:
        if _running(mapping_proc) or _running(patrol_proc):
            return jsonify(error="Another robot task is already active."), 409
        if _export_status.get("running"):
            return jsonify(error="Map export is still running; wait before starting a new capture."), 409
        env = os.environ.copy()
        env.pop("ROBOT_AGENT_TOKEN", None)
        env.pop("UNITREE_AES_128_KEY", None)
        env.update(DIMOS_MODE="real", ROBOT_IP=ROBOT_IP)
        if AES_KEY:
            env["GO2CONNECTION__AES_128_KEY"] = AES_KEY
        gomp = "/lib/aarch64-linux-gnu/libgomp.so.1"
        if Path(gomp).is_file():
            preload = env.get("LD_PRELOAD", "")
            if gomp not in preload.split(":"):
                env["LD_PRELOAD"] = f"{gomp}:{preload}" if preload else gomp
        tj_dir = SIM_ROOT / ".local-libs"
        for tj in (
            Path("/usr/lib/aarch64-linux-gnu/libturbojpeg.so.0"),
            tj_dir / "libturbojpeg.so.0",
            tj_dir / "libturbojpeg.so",
        ):
            if tj.is_file():
                lib_dir = str(tj.parent)
                ld_path = env.get("LD_LIBRARY_PATH", "")
                if lib_dir not in ld_path.split(":"):
                    env["LD_LIBRARY_PATH"] = f"{lib_dir}:{ld_path}" if ld_path else lib_dir
                env.setdefault("TURBOJPEG", str(tj))
                break
        with MAPPING_LOG.open("wb") as log:
            mapping_proc = subprocess.Popen(
                ["bash", str(SIM_ROOT / "start_mapping.sh"), name],
                cwd=SIM_ROOT,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        mapping_name = name
    return jsonify(ok=True, pid=mapping_proc.pid, name=name)


@app.post("/api/v1/mapping/stop")
def stop_mapping():
    global mapping_proc, mapping_name
    with lock:
        proc = mapping_proc
        name = mapping_name
        if not _running(proc):
            return jsonify(error="No mapping session is running."), 409
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

    stopped = False
    try:
        if proc.wait(timeout=25) is not None:
            stopped = True
    except subprocess.TimeoutExpired:
        _dimos_stop()
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=20)
            stopped = True
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            stopped = proc.poll() is not None

    with lock:
        if mapping_proc is proc and not _running(proc):
            mapping_proc = None

    export_started = False
    if name:
        db = MAPS / f"{name}.db"
        out = MAPS / f"{name}.pc2.lcm"
        if out.is_file():
            with _export_lock:
                _export_status.update(name=name, running=False, error=None, path=str(out))
        elif db.is_file():
            _export_map_async(name)
            export_started = True

    return jsonify(
        ok=True,
        stopped=stopped,
        name=name,
        export_started=export_started,
        message="Mapping stopped; map optimization/export continues on the robot if needed.",
    )


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
        env.pop("UNITREE_AES_128_KEY", None)
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
            env["GO2CONNECTION__AES_128_KEY"] = AES_KEY
        gomp = "/lib/aarch64-linux-gnu/libgomp.so.1"
        if Path(gomp).is_file():
            preload = env.get("LD_PRELOAD", "")
            if gomp not in preload.split(":"):
                env["LD_PRELOAD"] = f"{gomp}:{preload}" if preload else gomp
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
