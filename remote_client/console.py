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
# Origin of Orin Rerun web assets (via SSH tunnel). Used by /rerun-viewer/ cache proxy.
ROBOT_RERUN_WEB_ORIGIN = os.environ.get("ROBOT_RERUN_WEB_ORIGIN", "http://127.0.0.1:9878").rstrip("/")
ROUTES = PROJECT / "remote_client" / "routes"
DATA = PROJECT / "remote_client" / "data"
LOGS = PROJECT / "logs"
PAGE = PROJECT / "remote_client" / "console.html"
CURSOR_FILE = DATA / "event-cursor.json"
RERUN_WEB_CACHE = DATA / "rerun_web_cache"
app = Flask(__name__)
sync_lock = threading.Lock()
sync_started = False
sync_start_lock = threading.Lock()
last_status_log = 0.0
_rerun_cache_lock = threading.Lock()
ROUTES.mkdir(parents=True, exist_ok=True)
DATA.mkdir(parents=True, exist_ok=True)
LOGS.mkdir(parents=True, exist_ok=True)
RERUN_WEB_CACHE.mkdir(parents=True, exist_ok=True)


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


def _http_up(url: str) -> bool:
    """True if the tunnel/target accepts HTTP (any status means the service is up)."""
    if not url:
        return False
    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=2) as response:
            return int(response.status) >= 200
    except urllib.error.HTTPError as error:
        # 4xx/5xx still means the DimOS HTTP server is listening.
        return 400 <= int(error.code) < 600
    except (urllib.error.URLError, TimeoutError, OSError, ValueError):
        return False


def _viz_unavailable(name: str, url: str):
    body = (
        "<!doctype html><meta charset='utf-8'><title>"
        f"{name} 未启动</title>"
        "<body style='font:16px/1.45 system-ui;max-width:42rem;margin:3rem auto;padding:0 1rem'>"
        f"<h1>{name} 现在打不开</h1>"
        "<p>Command Center / Rerun 必须经本机 SSH 隧道访问（127.0.0.1:7779 / 9878）。"
        "请用 <code>./start_remote_console.sh</code> 启动调度台（脚本会自动建立隧道），"
        "并先点「启动采图」等 DimOS 在 Orin 上监听后再打开。</p>"
        "<p>不要直接打开 Orin 的 <code>192.168.123.18:7779/</code> 首页："
        "DimOS 页面里的 iframe 写死了 localhost，在上位机浏览器里会变成 "
        "「localhost refused to connect」。</p>"
        "<p>请回到调度台点击<strong>启动采图</strong>。DimOS 连上 Go2 后会 StandUp；"
        "周围空旷，急停握在手里。站立完成后刷新本页即可遥控。</p>"
        f"<p>当前目标地址：<code>{url or '未配置'}</code></p>"
        "<p><a href='/'>返回调度台</a></p>"
        "</body>"
    )
    return body, 503, {"Content-Type": "text/html; charset=utf-8"}


@app.get("/command-center")
def command_center():
    if not ROBOT_COMMAND_CENTER_URL:
        return _viz_unavailable("Command Center", "")
    if not _http_up(ROBOT_COMMAND_CENTER_URL):
        return _viz_unavailable("Command Center", ROBOT_COMMAND_CENTER_URL)
    from flask import redirect

    return redirect(ROBOT_COMMAND_CENTER_URL)


def _rerun_cacheable(path: str) -> bool:
    lower = path.lower()
    return lower.endswith(
        (
            ".wasm",
            ".js",
            ".css",
            ".html",
            ".ttf",
            ".woff",
            ".woff2",
            ".png",
            ".ico",
            ".svg",
            ".json",
        )
    )


def _rerun_connect_query() -> str:
    """Keep the gRPC url that start_remote_console.sh put on ROBOT_RERUN_URL."""
    incoming = request.query_string.decode("utf-8", errors="ignore")
    if incoming:
        return incoming
    return urllib.parse.urlparse(ROBOT_RERUN_URL).query


def _rerun_cache_file(rel_path: str) -> tuple[Path, Path, str]:
    cache_key = rel_path.strip("/") or "index.html"
    cache_path = RERUN_WEB_CACHE / cache_key
    return cache_path, Path(str(cache_path) + ".ctype"), cache_key


def _rerun_fetch_and_cache(rel_path: str) -> tuple[bytes, str] | tuple[None, None]:
    """Serve a cached Rerun asset from disk. Fetch the tunnel only on a miss."""
    remote = f"{ROBOT_RERUN_WEB_ORIGIN}/{rel_path.lstrip('/')}" if rel_path else f"{ROBOT_RERUN_WEB_ORIGIN}/"
    cache_path, meta_path, cache_key = _rerun_cache_file(rel_path)
    if _rerun_cacheable(cache_key) and cache_path.is_file() and meta_path.is_file():
        try:
            return cache_path.read_bytes(), meta_path.read_text(encoding="utf-8").strip()
        except OSError:
            pass
    req = urllib.request.Request(remote, headers={"Accept": "*/*"})
    try:
        with urllib.request.urlopen(req, timeout=120) as response:
            body = response.read()
            ctype = response.headers.get("Content-Type", "application/octet-stream")
    except (urllib.error.URLError, TimeoutError, OSError):
        return None, None
    if _rerun_cacheable(cache_key) and body:
        with _rerun_cache_lock:
            try:
                cache_path.parent.mkdir(parents=True, exist_ok=True)
                tmp = cache_path.with_suffix(cache_path.suffix + ".tmp")
                tmp.write_bytes(body)
                tmp.replace(cache_path)
                meta_path.write_text(ctype + "\n", encoding="utf-8")
            except OSError:
                pass
    return body, ctype


def _rerun_waiting_page(target: str) -> tuple[str, int, dict[str, str]]:
    body = (
        "<!doctype html><meta charset='utf-8'><title>Rerun 启动中</title>"
        "<body style='font:16px/1.45 system-ui;max-width:42rem;margin:3rem auto;padding:0 1rem'>"
        "<h1>Rerun 正在起来</h1>"
        "<p>这一页会留在这里，等 Orin 上的查看器一监听就自动进入。"
        "查看器程序用的是本机已经缓存的文件，不用再下那 40MB。</p>"
        f"<p><a href='{target}'>如果十几秒后还停在这里，点这里进入</a></p>"
        "<script>setTimeout(()=>location.replace(" + json.dumps(target) + "),1500)</script>"
        "</body>"
    )
    return body, 200, {"Content-Type": "text/html; charset=utf-8", "Cache-Control": "no-store"}


@app.get("/rerun")
def rerun_viewer():
    """Open the cached viewer and keep the gRPC url on the query string."""
    from flask import redirect

    qs = _rerun_connect_query()
    target = "/rerun-viewer/" + (f"?{qs}" if qs else "")
    if _http_up(ROBOT_RERUN_WEB_ORIGIN):
        return redirect(target)
    return _rerun_waiting_page("/rerun" + (f"?{qs}" if qs else ""))


@app.route("/rerun-viewer/", defaults={"path": ""})
@app.route("/rerun-viewer/<path:path>")
def rerun_viewer_proxy(path: str):
    """Serve Rerun's web UI from the host cache. The tunnel is used only on a miss."""
    from flask import Response

    cache_path, meta_path, cache_key = _rerun_cache_file(path)
    cached = _rerun_cacheable(cache_key) and cache_path.is_file() and meta_path.is_file()
    if cached:
        try:
            etag = f'"{cache_path.stat().st_size:x}-{int(cache_path.stat().st_mtime)}"'
        except OSError:
            etag = ""
        if etag and request.headers.get("If-None-Match") == etag:
            return Response(status=304, headers={"ETag": etag, "Cache-Control": "public, max-age=31536000, immutable"})
    elif not _http_up(ROBOT_RERUN_WEB_ORIGIN):
        return _viz_unavailable("Rerun", ROBOT_RERUN_WEB_ORIGIN)
    body, ctype = _rerun_fetch_and_cache(path)
    if body is None:
        return _viz_unavailable("Rerun", f"{ROBOT_RERUN_WEB_ORIGIN}/{path}")
    if (ctype or "").startswith("text/html") or cache_key.endswith(".html"):
        text = body.decode("utf-8", errors="replace")
        origin = ROBOT_RERUN_WEB_ORIGIN.rstrip("/")
        text = text.replace(origin + "/", "/rerun-viewer/")
        body = text.encode("utf-8")
        ctype = "text/html; charset=utf-8"
    headers = {"Content-Type": ctype or "application/octet-stream"}
    if _rerun_cacheable(cache_key):
        headers["Cache-Control"] = "public, max-age=31536000, immutable"
        if cached:
            headers["ETag"] = etag
    else:
        headers["Cache-Control"] = "no-cache"
    return Response(body, status=200, headers=headers)


@app.get("/api/health")
def health():
    if not ROBOT_URL:
        return jsonify(ok=False, error="ROBOT_AGENT_URL is not set."), 503
    req = urllib.request.Request(f"{ROBOT_URL}/health", headers={"Accept": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as response:
            payload = json.loads(response.read() or b"{}")
        return jsonify(ok=True, agent=payload, robot_agent_url=ROBOT_URL)
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as error:
        return jsonify(ok=False, error=str(error), robot_agent_url=ROBOT_URL), 502


_last_clock_push = 0.0


@app.get("/api/status")
def status():
    global last_status_log, _last_clock_push
    _start_sync_thread()
    result, code = _agent("status")
    if code != 200:
        return jsonify(error=result.get("error", "Robot agent unavailable."), robot_agent_url=ROBOT_URL), code
    now = time.time()
    # If Orin clock is skewed, push host UTC (throttled). Needs orin_timesync install.
    clock = result.get("clock") if isinstance(result, dict) else None
    if isinstance(clock, dict) and now - _last_clock_push >= 60:
        try:
            orin_epoch = int(clock.get("epoch") or 0)
        except (TypeError, ValueError):
            orin_epoch = 0
        skew = abs(int(now) - orin_epoch)
        if orin_epoch and skew >= 120:
            sync_result, sync_code = _agent("time/sync", "POST", {"epoch": int(now)}, timeout=20.0)
            result["clock_sync"] = {"attempted": True, "code": sync_code, "result": sync_result}
            _last_clock_push = now
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
    # Agent waits for DimOS SIGINT / dimos stop; optimized PGO export continues async.
    result, code = _agent("mapping/stop", "POST", {}, timeout=120.0)
    return jsonify(result), code


@app.post("/api/maps/export")
def export_map():
    body = request.get_json(silent=True) or {}
    raw = str(body.get("map") or body.get("name") or "")
    stem = Path(raw).name
    if stem.endswith(".pc2.lcm"):
        stem = stem[: -len(".pc2.lcm")]
    if not re.fullmatch(r"[A-Za-z0-9_-]+", stem):
        return jsonify(error="Select a valid map recording name."), 400
    result, code = _agent(f"maps/{urllib.parse.quote(stem)}/export", "POST", {}, timeout=30.0)
    return jsonify(result), code


@app.get("/api/wifi")
def wifi_status():
    result, code = _agent("wifi")
    return jsonify(result), code


@app.post("/api/wifi/prefer")
def wifi_prefer():
    body = request.get_json(silent=True) or {}
    result, code = _agent("wifi/prefer", "POST", {"profile": body.get("profile", "")}, timeout=30.0)
    return jsonify(result), code


@app.post("/api/wifi/connect")
def wifi_connect():
    body = request.get_json(silent=True) or {}
    result, code = _agent(
        "wifi/connect",
        "POST",
        {"profile": body.get("profile", "")},
        timeout=60.0,
    )
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


@app.post("/api/patrol/halt")
def halt_patrol():
    result, code = _agent("patrol/halt", "POST", {}, timeout=20.0)
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


@app.post("/api/patrol/confirm-pose")
def confirm_pose():
    result, code = _agent("patrol/confirm-pose", "POST", {})
    return jsonify(result), code


@app.post("/api/patrol/nudge-pose")
def nudge_pose():
    body = request.get_json(silent=True) or {}
    payload = {key: body.get(key, 0) for key in ("dx", "dy", "dyaw_deg")}
    result, code = _agent("patrol/nudge-pose", "POST", payload)
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
