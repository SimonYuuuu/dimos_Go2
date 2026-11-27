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
NUDGE_FILE = DATA / "pose-nudge.json"
DOG_POSE_FILE = DATA / "dog-pose.json"
SPEED_FILE = DATA / "patrol-speed.json"
KEEPOUT_FILE = DATA / "patrol-keepouts.json"
SLOW_MPS = 0.35
FAST_MPS = 0.70
MAX_MPS = 1.0
EVENTS_FILE = DATA / "fire-alarm-events.jsonl"
MAPPING_LOG = DATA / "mapping.log"
PATROL_LOG = DATA / "patrol.log"
PATROL_DIMOS_LOG = DATA / "patrol-dimos.log"
_RELOC_LOG_RE = re.compile(
    r"provisional TF|relocalize settle|relocalize fix|placed, no further|Initial pose hint",
    re.I,
)
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
mapping_profile: str = "standard"
patrol_proc: subprocess.Popen[bytes] | None = None
MAP_QUALITY_MPS = 0.25
_reachability: dict[str, Any] = {"checked_at": 0.0}
_export_lock = threading.Lock()
_export_status: dict[str, Any] = {
    "name": None,
    "recording": None,
    "running": False,
    "error": None,
    "path": None,
}

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


_LOG_NOISE_RE = re.compile(
    r"aiortc|avcodec_send_packet|Invalid data found|Zenoh endpoints|^[=\s]*$|"
    r"Rerun gRPC|Connect a viewer|dimos-viewer --connect|hostname:|Deployed module|Worker |"
    r"load back with|from dimos\.msgs|pcd = PointCloud2",
)
_LOG_PROGRESS_RE = re.compile(r"\d+%")


def _mapping_log_for_ui(count: int = 60) -> list[str]:
    """Mapping/export log without video-decode spam; progress bars keep only their last update."""
    try:
        with MAPPING_LOG.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            handle.seek(max(0, handle.tell() - 200_000))
            text = handle.read().decode("utf-8", errors="replace")
    except OSError:
        return []
    out: list[str] = []
    heartbeat = ""
    progress_at: dict[str, int] = {}
    for line in text.splitlines():
        line = line.rstrip()
        if _LOG_NOISE_RE.search(line):
            continue
        if "[heartbeat]" in line:
            heartbeat = line
            continue
        if _LOG_PROGRESS_RE.search(line):
            key = _LOG_PROGRESS_RE.split(line)[0]
            if key in progress_at:
                out[progress_at[key]] = line
                continue
            progress_at[key] = len(out)
        out.append(line)
    if heartbeat:
        out.append(heartbeat)
    return out[-count:]


def _reloc_log_tail(count: int = 50) -> list[str]:
    """Key reloc lines from DimOS log (console only showed launcher patrol.log before)."""
    hits = [line for line in _tail(PATROL_DIMOS_LOG, 800) if _RELOC_LOG_RE.search(line)]
    return hits[-count:]


def _patrol_log_for_ui() -> list[str]:
    # Launcher summary + reloc progress so operators can see provisional vs locked.
    return _tail(PATROL_LOG, 40) + _reloc_log_tail(50)


def _battery() -> dict[str, Any]:
    data = _read_json(DATA / "battery.json", {})
    if not isinstance(data, dict):
        return {"soc": None}
    try:
        soc = int(data["soc"])
        age = round(max(0.0, time.time() - float(data.get("t", 0.0))), 1)
    except (KeyError, TypeError, ValueError):
        return {"soc": None}
    if not 0 <= soc <= 100 or age > 30:
        return {"soc": None, "age_s": age}
    return {"soc": soc, "age_s": age, "low": soc < 5}


def _task_log_tail(after: int = -1, limit: int = 400) -> tuple[str | None, list[dict[str, Any]], bool]:
    """Events newer than ``after``. ``after < 0`` returns only the live tail.

    The host console archives by seq. A dropped link used to lose every event
    past the last 80 lines; passing the last seq it already has fills the gap
    once the network returns. ``limit`` keeps one status response small.
    """
    meta = _read_json(DATA / "patrol-task.json", {})
    if not isinstance(meta, dict):
        return None, [], False
    name = meta.get("name")
    path = meta.get("path")
    if not isinstance(name, str) or not isinstance(path, str):
        return None, [], False
    if after < 0:
        lines = _tail(Path(path), 80)
    else:
        try:
            lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            lines = []
    events: list[dict[str, Any]] = []
    for line in lines:
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(item, dict):
            continue
        try:
            seq = int(item.get("seq", -1))
        except (TypeError, ValueError):
            seq = -1
        if after >= 0 and seq <= after:
            continue
        events.append(item)
    more = len(events) > limit
    return name, events[:limit], more


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


def _viz_ports_busy() -> bool:
    return any(_tcp_open("127.0.0.1", port) for port in (7779, 9877, 9878, 3030))


def _pkill_patterns(*patterns: str) -> None:
    for pattern in patterns:
        subprocess.run(
            ["pkill", "-TERM", "-f", pattern],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def _pgrep(pattern: str) -> bool:
    result = subprocess.run(
        ["pgrep", "-f", pattern],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def _mapping_active_external() -> bool:
    """True when a mapping shell/DimOS is up even if this agent lost the Popen handle."""
    return _pgrep(r"start_mapping_quality\.sh|/start_mapping\.sh|unitree-go2-memory") and _viz_ports_busy()


def _mapping_is_running() -> bool:
    return _running(mapping_proc) or _mapping_active_external()


def _mapping_cmdline_db() -> str | None:
    """Best-effort db path from a live dimos/mapping process cmdline."""
    try:
        result = subprocess.run(
            ["pgrep", "-af", r"go2memory.db-path|start_mapping"],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return None
    for line in (result.stdout or "").splitlines():
        match = re.search(r"--go2memory\.db-path\s+(\S+)", line)
        if match:
            return match.group(1)
        match = re.search(r"maps/([A-Za-z0-9_-]+)\.db", line)
        if match:
            return str(MAPS / f"{match.group(1)}.db")
    return None


def _mapping_live_info() -> dict[str, Any]:
    """Extra fields so the console can show whether logs/session are actually alive."""
    info: dict[str, Any] = {
        "log_path": str(MAPPING_LOG),
        "log_age_s": None,
        "log_bytes": None,
        "db_path": _mapping_cmdline_db(),
        "heartbeat_ok": False,
        "last_log_line": None,
    }
    try:
        st = MAPPING_LOG.stat()
        info["log_bytes"] = int(st.st_size)
        info["log_age_s"] = round(max(0.0, time.time() - st.st_mtime), 1)
    except OSError:
        pass
    lines = _tail(MAPPING_LOG, 40)
    if lines:
        info["last_log_line"] = lines[-1][:240]
        recent = lines[-12:]
        info["heartbeat_ok"] = any("[heartbeat]" in line for line in recent) and (
            info["log_age_s"] is not None and info["log_age_s"] < 12
        )
    if info["db_path"]:
        try:
            info["db_name"] = Path(info["db_path"]).name
            info["db_bytes"] = int(Path(info["db_path"]).stat().st_size)
        except OSError:
            info["db_name"] = Path(info["db_path"]).name
    return info


def _clear_orphan_mapping() -> bool:
    """Stop leftover mapping DimOS only when it is not a healthy live session.

    Live sessions (start_mapping* + viz ports) are left alone so agent restarts
    do not kill an in-progress capture.
    """
    if _running(mapping_proc) or _running(patrol_proc):
        return False
    if _mapping_active_external():
        return False
    if not _viz_ports_busy() and not _pgrep("start_mapping"):
        return False
    _dimos_stop()
    _pkill_patterns(
        r"start_mapping_quality\.sh",
        r"[/ ]start_mapping\.sh",
        r"dimos --viewer rerun",
        r"unitree-go2-memory",
    )
    deadline = time.time() + 8.0
    while time.time() < deadline:
        if not _viz_ports_busy() and not _pgrep("start_mapping"):
            break
        time.sleep(0.4)
    if _viz_ports_busy():
        _pkill_patterns(
            r"start_mapping_quality\.sh",
            r"[/ ]start_mapping\.sh",
            r"dimos --viewer rerun",
            r"unitree-go2-memory",
        )
        subprocess.run(
            ["pkill", "-KILL", "-f", r"dimos --viewer rerun"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(0.5)
    return True


def _map_stem(value: object) -> str:
    """Strip .pc2.lcm / .db safely (Path.stem alone leaves '.pc2' on dual extensions)."""
    name = Path(str(value or "")).name
    for suffix in (".pc2.lcm", ".db", ".rrd"):
        if name.endswith(suffix):
            name = name[: -len(suffix)]
            break
    return name


def _recording_stem(selected: str) -> str | None:
    """Resolve which .db backs a selected map, stripping trailing _N export suffixes."""
    stem = _map_stem(selected)
    candidates = [stem]
    cur = stem
    while True:
        match = re.fullmatch(r"(.+)_([0-9]+)", cur)
        if not match:
            break
        cur = match.group(1)
        candidates.append(cur)
    for candidate in candidates:
        if (MAPS / f"{candidate}.db").is_file():
            return candidate
    return None


def _next_export_stem(recording: str) -> str:
    """Keep recording.pc2.lcm intact; write recording_1, _2, … for new optimized exports."""
    nums: list[int] = []
    if (MAPS / f"{recording}.pc2.lcm").is_file():
        nums.append(0)
    for path in MAPS.glob(f"{recording}_*.pc2.lcm"):
        match = re.fullmatch(rf"{re.escape(recording)}_([0-9]+)\.pc2\.lcm", path.name)
        if match:
            nums.append(int(match.group(1)))
    if not nums:
        return recording
    return f"{recording}_{max(nums) + 1}"


def _export_map_async(
    recording: str,
    *,
    out_stem: str | None = None,
    force: bool = True,
) -> tuple[bool, str | None]:
    """Run optimized PGO export: maps/<recording>.db → maps/<out_stem>.pc2.lcm.

    out_stem defaults to the next free incremental name so originals are never overwritten.
    """
    db = MAPS / f"{recording}.db"
    export_stem = out_stem or _next_export_stem(recording)
    out = MAPS / f"{export_stem}.pc2.lcm"
    if not db.is_file():
        with _export_lock:
            _export_status.update(
                name=export_stem,
                recording=recording,
                running=False,
                error=f"missing {db.name}",
                path=None,
            )
        return False, None
    if out.is_file() and not force:
        with _export_lock:
            _export_status.update(
                name=export_stem,
                recording=recording,
                running=False,
                error=None,
                path=str(out),
            )
        return False, export_stem
    with _export_lock:
        if _export_status.get("running"):
            return False, None
        _export_status.update(
            name=export_stem,
            recording=recording,
            running=True,
            error=None,
            path=None,
        )

    def worker() -> None:
        try:
            env = os.environ.copy()
            gomp = "/lib/aarch64-linux-gnu/libgomp.so.1"
            if Path(gomp).is_file():
                preload = env.get("LD_PRELOAD", "")
                if gomp not in preload.split(":"):
                    env["LD_PRELOAD"] = f"{gomp}:{preload}" if preload else gomp
            # Localization-ready defaults (export_map.sh also has these fallbacks).
            # Quality mapping uses a denser rebuild so walls stay sharp.
            if env.get("DIMOS_MAP_QUALITY", "0") in ("1", "true", "TRUE", "yes", "YES"):
                env.setdefault("DIMOS_MAP_VOXEL", "0.025")
                env.setdefault("DIMOS_MAP_PGO_TOL", "0.06")
                env.setdefault("DIMOS_MAP_FULL_PGO", "1")
            else:
                env.setdefault("DIMOS_MAP_VOXEL", "0.03")
                env.setdefault("DIMOS_MAP_PGO_TOL", "0.08")
            env.setdefault("DIMOS_MAP_DENOISE", "1")
            with MAPPING_LOG.open("ab") as log:
                log.write(
                    f"\n=== export_map recording={recording} out={export_stem} force={force} ===\n".encode()
                )
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
                    name=export_stem,
                    recording=recording,
                    running=False,
                    error=err,
                    path=str(out) if out.is_file() else None,
                )
        except OSError as error:
            with _export_lock:
                _export_status.update(
                    name=export_stem,
                    recording=recording,
                    running=False,
                    error=str(error),
                    path=None,
                )

    threading.Thread(target=worker, name=f"export-{export_stem}", daemon=True).start()
    return True, export_stem


def _safe_name(value: object) -> str:
    name = _map_stem(value)
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


def _orin_ipv4_addrs() -> list[dict[str, str]]:
    """Local Orin addresses (host↔Orin WiFi vs dog internal LAN)."""
    rows: list[dict[str, str]] = []
    try:
        completed = subprocess.run(
            ["ip", "-br", "-4", "addr"],
            text=True,
            capture_output=True,
            timeout=2,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return rows
    for line in (completed.stdout or "").splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        iface, state = parts[0], parts[1]
        if iface in ("lo", "docker0", "dummy0") or state == "DOWN":
            continue
        for token in parts[2:]:
            ip = token.split("/", 1)[0]
            if not re.fullmatch(r"\d+\.\d+\.\d+\.\d+", ip):
                continue
            role = "dog_lan" if ip.startswith("192.168.123.") else "host_link"
            rows.append({"iface": iface, "ip": ip, "role": role})
    return rows


def _connectivity() -> dict[str, Any]:
    now = time.time()
    cached = _reachability
    if now - float(cached.get("checked_at") or 0) < 8:
        return {k: v for k, v in cached.items() if k != "checked_at"}
    orin_addrs = _orin_ipv4_addrs()
    host_link = next((r for r in orin_addrs if r["role"] == "host_link"), None)
    dog_lan = next((r for r in orin_addrs if r["role"] == "dog_lan"), None)
    snapshot = {
        # Go2 MCU address on the dog's internal LAN (Orin eth*). Not the host WiFi IP.
        "robot_ip": ROBOT_IP,
        "robot_reachable": _ping(ROBOT_IP),
        "lidar_ip": LIDAR_IP,
        "lidar_reachable": _ping(LIDAR_IP),
        "dimos_installed": DIMOS_BIN.is_file(),
        "command_center": _tcp_open("127.0.0.1", 7779),
        "rerun": _tcp_open("127.0.0.1", 9878),
        "webrtc_signaling": bool(ROBOT_IP)
        and (_tcp_open(ROBOT_IP, 9991) or _tcp_open(ROBOT_IP, 8081)),
        "orin_addrs": orin_addrs,
        "orin_host_link": host_link,
        "orin_dog_lan": dog_lan,
    }
    cached.clear()
    cached.update(checked_at=now, **snapshot)
    return snapshot


def _wifi_snapshot() -> dict[str, Any]:
    try:
        from wifi_control import status as wifi_status

        return wifi_status()
    except Exception as error:  # Keep /status healthy even if nmcli is missing.
        return {"error": str(error), "profiles": [], "needs_install": True}


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


def _timesync_bin() -> Path:
    installed = Path("/usr/local/sbin/dimos-orin-timesync")
    if installed.is_file():
        return installed
    return PROJECT / "scripts" / "orin_timesync.sh"


def _clock_snapshot() -> dict[str, Any]:
    epoch = int(time.time())
    needs_install = not Path("/usr/local/sbin/dimos-orin-timesync").is_file()
    ntp = None
    synced = None
    zone = None
    try:
        show = subprocess.run(
            ["timedatectl", "show", "-p", "Timezone", "-p", "NTP", "-p", "NTPSynchronized", "--value"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        lines = [ln.strip() for ln in (show.stdout or "").splitlines() if ln.strip()]
        if len(lines) >= 3:
            zone, ntp, synced = lines[0], lines[1], lines[2]
        elif len(lines) >= 2:
            ntp, synced = lines[0], lines[1]
        elif len(lines) == 1:
            ntp = lines[0]
    except (OSError, subprocess.SubprocessError):
        pass
    return {
        "epoch": epoch,
        "utc": datetime.fromtimestamp(epoch, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "timezone": zone,
        "ntp": ntp,
        "synchronized": synced,
        "needs_install": needs_install,
        "install_hint": "ssh to Orin and run: sudo ./scripts/orin_timesync.sh install",
    }


def _apply_host_epoch(epoch: int) -> dict[str, Any]:
    bin_path = _timesync_bin()
    if not bin_path.is_file():
        return {
            "ok": False,
            "error": "timesync helper missing; run sudo ./scripts/orin_timesync.sh install on Orin",
            "clock": _clock_snapshot(),
        }
    before = int(time.time())
    skew = epoch - before
    cmd = ["sudo", "-n", str(bin_path), "set-epoch", str(epoch)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=20, check=False)
    except (OSError, subprocess.SubprocessError) as error:
        return {"ok": False, "error": str(error), "clock": _clock_snapshot(), "skew_before_s": skew}
    ok = proc.returncode == 0
    return {
        "ok": ok,
        "skew_before_s": skew,
        "stdout": (proc.stdout or "").strip()[-500:],
        "stderr": (proc.stderr or "").strip()[-300:],
        "clock": _clock_snapshot(),
        "error": None if ok else (proc.stderr or proc.stdout or f"exit {proc.returncode}"),
    }


@app.get("/health")
def health():
    return jsonify(ok=True, service="dimos-go2-robot-agent", mode=DIMOS_MODE, robot_ip=ROBOT_IP)


def _format_initial_pose(pose: object) -> str | None:
    """x,y,yaw in the loaded-map frame, yaw wrapped to [-pi, pi]."""
    if not isinstance(pose, dict):
        return None
    try:
        x = float(pose["x"])
        y = float(pose["y"])
        yaw = float(pose["yaw"])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return f"{x:.3f},{y:.3f},{yaw:.4f}"


def _dog_pose() -> dict | None:
    pose = _read_json(DOG_POSE_FILE, None)
    if not isinstance(pose, dict) or not _running(patrol_proc):
        return None
    pose["age_s"] = round(time.time() - float(pose.get("t", 0.0)), 1)
    return pose


@app.get("/api/v1/status")
def status():
    control = _read_json(CONTROL_FILE, {})
    patrol = _read_json(STATUS_FILE, {"phase": "idle", "message": "No patrol status yet."})
    if not isinstance(control, dict):
        control = {}
    connectivity = _connectivity()
    try:
        task_after = int(request.args.get("task_after", "-1"))
    except (TypeError, ValueError):
        task_after = -1
    task_name_filter = request.args.get("task_name", "")
    meta_name = ""
    meta = _read_json(DATA / "patrol-task.json", {})
    if isinstance(meta, dict) and isinstance(meta.get("name"), str):
        meta_name = meta["name"]
    # A cursor from the previous task must not hide the new task's early events.
    if task_name_filter and meta_name and task_name_filter != meta_name:
        task_after = 0
    task_name, task_events, task_more = _task_log_tail(task_after)
    return jsonify(
        robot={
            "mode": DIMOS_MODE,
            "robot_ip": ROBOT_IP,
            "online": bool(connectivity.get("robot_reachable")),
        },
        clock=_clock_snapshot(),
        connectivity=connectivity,
        mapping={
            "running": _mapping_is_running(),
            "name": mapping_name or ("external" if _mapping_active_external() else None),
            "profile": (
                mapping_profile
                if _running(mapping_proc)
                else ("quality" if _mapping_active_external() else "idle")
            ),
            "phase": (
                "recording"
                if _mapping_is_running()
                else (
                    "exporting"
                    if _export_status.get("running")
                    else (
                        "map_ready"
                        if _export_status.get("path") and not _export_status.get("error")
                        else "idle"
                    )
                )
            ),
            "export": dict(_export_status),
            "live": _mapping_live_info(),
            "log": _mapping_log_for_ui(60),
        },
        patrol={
            **patrol,
            "running": _running(patrol_proc),
            "log": _patrol_log_for_ui(),
            "reloc_log": _reloc_log_tail(50),
            "dog_pose": _dog_pose(),
            "walk_speed": _read_json(SPEED_FILE, {}) if _running(patrol_proc) else {},
            "task_log_name": task_name,
            "task_log": task_events,
            "task_log_more": task_more,
        },
        battery=_battery(),
        fire_alarm={
            "state": control.get("fire_state", "NaN"),
            "source": control.get("fire_source", "no signal connected"),
            "events": control.get("fire_events", []),
        },
        wifi=_wifi_snapshot(),
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
    # Newest first: the console preselects the first entry.
    found = sorted(MAPS.glob("*.pc2.lcm"), key=lambda path: (-path.stat().st_mtime, path.name))
    return jsonify([path.name for path in found])


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
    global mapping_proc, mapping_name, mapping_profile
    body = request.get_json(silent=True) or {}
    try:
        name = _safe_name(body.get("name", "go2-map"))
    except ValueError as error:
        return jsonify(error=str(error)), 400
    profile = str(body.get("profile", "standard")).strip().lower()
    if profile not in ("standard", "quality"):
        return jsonify(error="profile must be standard or quality."), 400
    if DIMOS_MODE != "real":
        return jsonify(error="Robot agent only accepts real-robot tasks; simulation stays in simulation_legacy."), 409
    if not ROBOT_IP:
        return jsonify(error="Configure ROBOT_IP on the robot agent host."), 503
    if not DIMOS_BIN.is_file():
        return jsonify(error="DimOS is not installed in simulation_legacy/.venv on the robot."), 503
    # Quality mapping lives at repo root (same level as stop_dimos.sh).
    script = (
        PROJECT / "start_mapping_quality.sh"
        if profile == "quality"
        else SIM_ROOT / "start_mapping.sh"
    )
    if not script.is_file():
        return jsonify(error=f"Missing mapping script {script.name} on the robot."), 503
    with lock:
        if _running(mapping_proc) or _mapping_active_external():
            return jsonify(
                ok=True,
                already_running=True,
                pid=mapping_proc.pid if mapping_proc and _running(mapping_proc) else None,
                name=mapping_name or name,
                profile=mapping_profile if _running(mapping_proc) else profile,
                message=(
                    f"Mapping already running ({mapping_name or name}). "
                    "Open Command Center to drive; do not start again."
                ),
            )
        if _running(patrol_proc):
            return jsonify(error="Patrol is active; stop it before mapping."), 409
        if _export_status.get("running"):
            return jsonify(error="Map export is still running; wait before starting a new capture."), 409

    # Clear dead viz leftovers only (healthy live mapping is not killed).
    if _viz_ports_busy() or _pgrep("start_mapping"):
        _clear_orphan_mapping()
        if _mapping_active_external():
            return jsonify(
                ok=True,
                already_running=True,
                name=name,
                profile=profile,
                message="Mapping already running. Open Command Center to drive.",
            )
        if _viz_ports_busy():
            return jsonify(
                error="Viz ports 7779/9877/9878 still busy. On Orin run: ./simulation_legacy/stop_mapping.sh"
            ), 409

    with lock:
        if _running(mapping_proc) or _running(patrol_proc) or _mapping_active_external():
            return jsonify(
                ok=True,
                already_running=True,
                name=mapping_name or name,
                profile=profile,
                message="Mapping already running. Open Command Center to drive.",
            )
        env = os.environ.copy()
        env.pop("ROBOT_AGENT_TOKEN", None)
        env.pop("UNITREE_AES_128_KEY", None)
        env.update(DIMOS_MODE="real", ROBOT_IP=ROBOT_IP)
        if AES_KEY:
            env["GO2CONNECTION__AES_128_KEY"] = AES_KEY
        if profile == "quality":
            mps = float(body.get("walk_mps", MAP_QUALITY_MPS))
            if not math.isfinite(mps) or mps < 0.05 or mps > 0.40:
                return jsonify(error="walk_mps for quality mapping must be from 0.05 to 0.40."), 400
            # Quality = slow teleop + OA off. Export knobs match remote console
            # so stop→export stays fast (override via env if denser PGO is needed).
            env.update(
                DIMOS_MAP_QUALITY=env.get("DIMOS_MAP_QUALITY", "0"),
                DIMOS_MAP_WALK_MPS=str(mps),
                DIMOS_MAP_VOXEL=env.get("DIMOS_MAP_VOXEL", "0.03"),
                DIMOS_MAP_PGO_TOL=env.get("DIMOS_MAP_PGO_TOL", "0.08"),
                DIMOS_MAP_DENOISE=env.get("DIMOS_MAP_DENOISE", "1"),
                DIMOS_MAP_FULL_PGO=env.get("DIMOS_MAP_FULL_PGO", "0"),
                PATROL_SPEED_FILE=str(SPEED_FILE),
            )
            _write_json(SPEED_FILE, {"mps": mps, "locked": True, "mode": "map_quality"})
        # Script owns mapping.log (heartbeat + DimOS). Agent only seeds/truncates.
        env["MAPPING_LOG_FILE"] = str(MAPPING_LOG)
        env["MAPPING_LOG_ATTACHED"] = "1"
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
        MAPPING_LOG.parent.mkdir(parents=True, exist_ok=True)
        # Truncate for a fresh session, keep fd open for the child (do not use `with`).
        log_fh = MAPPING_LOG.open("wb")
        try:
            log_fh.write(f"==== agent mapping/start name={name} profile={profile} ====\n".encode())
            log_fh.flush()
            mapping_proc = subprocess.Popen(
                ["bash", str(script), name],
                cwd=SIM_ROOT if profile != "quality" else PROJECT,
                env=env,
                stdout=log_fh,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        finally:
            # Child keeps a dup; parent copy can close.
            try:
                log_fh.close()
            except OSError:
                pass
        mapping_name = name
        mapping_profile = profile
        proc = mapping_proc
    # Detect immediate exit (port busy / preflight fail) so the UI gets a real error.
    time.sleep(1.2)
    if not _running(proc):
        with lock:
            if mapping_proc is proc:
                mapping_proc = None
                mapping_profile = "standard"
        tail = "\n".join(_tail(MAPPING_LOG, 12))
        return jsonify(
            error="Mapping process exited immediately. " + (tail or "Check robot_data/mapping.log on Orin."),
            log_tail=tail,
        ), 500
    return jsonify(ok=True, pid=proc.pid, name=name, profile=profile)


@app.post("/api/v1/mapping/stop")
def stop_mapping():
    global mapping_proc, mapping_name, mapping_profile
    with lock:
        proc = mapping_proc
        name = mapping_name
        profile = mapping_profile
        owned = _running(proc)
        external = (not owned) and _mapping_active_external()
        if not owned and not external:
            return jsonify(error="No mapping session is running."), 409
        if owned:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    stopped = False
    if owned and proc is not None:
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
    else:
        # Agent restarted and lost Popen; stop the external mapping shell/DimOS.
        _dimos_stop()
        _pkill_patterns(
            r"start_mapping_quality\.sh",
            r"[/ ]start_mapping\.sh",
            r"dimos --viewer rerun",
            r"unitree-go2-memory",
        )
        deadline = time.time() + 20.0
        while time.time() < deadline:
            if not _mapping_active_external() and not _viz_ports_busy():
                stopped = True
                break
            time.sleep(0.5)
        if not stopped:
            subprocess.run(
                ["pkill", "-KILL", "-f", r"dimos --viewer rerun"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.5)
            stopped = not _mapping_active_external()

    with lock:
        if mapping_proc is proc and not _running(proc):
            mapping_proc = None
            mapping_profile = "standard"
        if not _mapping_is_running():
            mapping_proc = None
            mapping_profile = "standard"

    # Export uses console-standard PGO (same as remote console「结束采集并导出」).
    os.environ.setdefault("DIMOS_MAP_VOXEL", "0.03")
    os.environ.setdefault("DIMOS_MAP_PGO_TOL", "0.08")
    os.environ.setdefault("DIMOS_MAP_FULL_PGO", "0")
    os.environ.setdefault("DIMOS_MAP_DENOISE", "1")

    export_started = False
    export_name = None
    if name and (MAPS / f"{name}.db").is_file():
        export_started, export_name = _export_map_async(name, force=True)

    return jsonify(
        ok=True,
        stopped=stopped,
        name=name,
        profile=profile,
        export_started=export_started,
        export_name=export_name,
        message=(
            f"Mapping stopped; optimized PGO export writing {export_name}.pc2.lcm and {export_name}.map.db."
            if export_name
            else "Mapping stopped; no recording found to export."
        ),
    )


@app.post("/api/v1/maps/<name>/export")
def export_map(name: str):
    """Re-run optimized PGO export from an existing recording (.db) into a new _N file."""
    try:
        selected = _safe_name(name)
    except ValueError as error:
        return jsonify(error=str(error)), 400
    recording = _recording_stem(selected)
    if not recording:
        return jsonify(error=f"Recording .db not found for {selected}."), 404
    if _running(mapping_proc) or _running(patrol_proc):
        return jsonify(error="Stop mapping/patrol before re-exporting a map."), 409
    with _export_lock:
        if _export_status.get("running"):
            return jsonify(error="Another map export is already running."), 409
    started, export_name = _export_map_async(recording, force=True)
    if not started or not export_name:
        return jsonify(error="Could not start map export (busy or missing recording)."), 409
    return jsonify(
        ok=True,
        recording=recording,
        name=export_name,
        map=f"{export_name}.pc2.lcm",
        message=f"Optimized PGO export started → {export_name}.pc2.lcm (original kept). Refresh when idle.",
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
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
        safe_route = re.sub(r"[^A-Za-z0-9._-]", "_", route_name)[:80] or "route"
        task_log = PROJECT / "logs" / f"patrol-{stamp}-{safe_route}.jsonl"
        task_log.parent.mkdir(parents=True, exist_ok=True)
        _write_json(
            DATA / "patrol-task.json",
            {
                "name": task_log.name,
                "path": str(task_log),
                "route": route_name,
                "started_at": datetime.now(timezone.utc).isoformat(),
            },
        )
        env = os.environ.copy()
        env.pop("ROBOT_AGENT_TOKEN", None)
        env.pop("UNITREE_AES_128_KEY", None)
        env.update(
            DIMOS_MODE="real",
            ROBOT_IP=ROBOT_IP,
            PATROL_STATUS_FILE=str(STATUS_FILE),
            PATROL_CONTROL_FILE=str(CONTROL_FILE),
            PATROL_TRANSFORM_FILE=str(DATA / "map-to-world.json"),
            PATROL_NUDGE_FILE=str(NUDGE_FILE),
            PATROL_DOG_POSE_FILE=str(DOG_POSE_FILE),
            PATROL_DIMOS_LOG_FILE=str(DATA / "patrol-dimos.log"),
            PATROL_START_MODE=execution_mode,
            PATROL_START_AT=start_at if execution_mode == "daily" else "",
            PATROL_MAX_DURATION_S=str(max_duration_s),
            PATROL_REPEAT_INTERVAL_S=str(repeat_interval_s),
            PATROL_REPEAT_ENABLED="1" if repeat_enabled else "0",
            PATROL_KEEPOUT_FILE=str(KEEPOUT_FILE),
            PATROL_SPEED_FILE=str(SPEED_FILE),
            PATROL_BATTERY_FILE=str(DATA / "battery.json"),
            PATROL_TASK_LOG=str(task_log),
        )
        # Prefer explicit agent env; fall back to robot_agent/.env via start_patrol.sh.
        if os.environ.get("PATROL_LOCALIZE_ONLY"):
            env["PATROL_LOCALIZE_ONLY"] = os.environ["PATROL_LOCALIZE_ONLY"]
        if os.environ.get("DIMOS_PATROL_BLUEPRINT"):
            env["DIMOS_PATROL_BLUEPRINT"] = os.environ["DIMOS_PATROL_BLUEPRINT"]
        if os.environ.get("DIMOS_RELOC_MODULE"):
            env["DIMOS_RELOC_MODULE"] = os.environ["DIMOS_RELOC_MODULE"]
        if AES_KEY:
            env["GO2CONNECTION__AES_128_KEY"] = AES_KEY
        gomp = "/lib/aarch64-linux-gnu/libgomp.so.1"
        if Path(gomp).is_file():
            preload = env.get("LD_PRELOAD", "")
            if gomp not in preload.split(":"):
                env["LD_PRELOAD"] = f"{gomp}:{preload}" if preload else gomp
        # Console value: dog pose in the loaded map frame (x, y meters, yaw radians).
        # Same frame as the route waypoints. Not odometry, not world←map.
        initial_pose = str(body.get("initial_pose") or "").strip()
        if initial_pose:
            env["DIMOS_INITIAL_POSE"] = initial_pose
        elif os.environ.get("DIMOS_INITIAL_POSE"):
            env["DIMOS_INITIAL_POSE"] = os.environ["DIMOS_INITIAL_POSE"]
        elif os.environ.get("DIMOS_MODE", "real") == "real":
            # Rectangular rooms: global ICP alone false-locks at 90°/180°.
            env["DIMOS_INITIAL_POSE"] = "0,0,0"
        current_control = _read_json(CONTROL_FILE, {})
        current_fire = current_control.get("fire_state", "NaN") if isinstance(current_control, dict) else "NaN"
        try:
            keepouts = _keepouts_from_route(route)
        except ValueError as error:
            return jsonify(error=str(error)), 400
        _write_json(SPEED_FILE, {"mps": SLOW_MPS, "locked": False})
        _write_json(KEEPOUT_FILE, keepouts)
        _control(stop_requested=False, halt=False, paused=current_fire == "Fire_alarm")
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
    """Stop patrol/localize. Soft-stop first (waypoint return-home), then kill the
    process group — localize-only never runs waypoint_patrol, so control-file alone
    would leave DimOS running forever and block the next start with 409.
    """
    global patrol_proc
    with lock:
        proc = patrol_proc
        if not _running(proc):
            return jsonify(error="No patrol task is running."), 409
        _control(paused=False, stop_requested=True)

    # Localize-only: no return-home; kill quickly.
    # Full patrol: waypoint_patrol navigates to return_to_start — allow time.
    localize_only = str(os.environ.get("PATROL_LOCALIZE_ONLY", "0")).lower() in (
        "1",
        "true",
        "yes",
    )
    grace_s = 3 if localize_only else int(os.environ.get("PATROL_STOP_GRACE_S", "180"))
    try:
        if proc.wait(timeout=grace_s) is not None:
            with lock:
                if patrol_proc is proc:
                    patrol_proc = None
            _write_json(STATUS_FILE, {"phase": "idle", "message": "Patrol stopped."})
            return jsonify(ok=True, message="Patrol stopped.")
    except subprocess.TimeoutExpired:
        pass

    _dimos_stop()
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=8 if sig != signal.SIGKILL else 5)
            break
        except subprocess.TimeoutExpired:
            continue

    with lock:
        if patrol_proc is proc and not _running(proc):
            patrol_proc = None
    _write_json(STATUS_FILE, {"phase": "idle", "message": "Patrol/localize stopped."})
    return jsonify(ok=True, message="Patrol/localize force-stopped.")


def _kill_patrol_group(proc: subprocess.Popen[bytes] | None) -> None:
    _dimos_stop()
    if proc is not None and _running(proc):
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=3)
                break
            except subprocess.TimeoutExpired:
                continue
    # The agent may have been restarted and lost the process handle.
    subprocess.run(
        ["pkill", "-f", "simulation_legacy/start_patrol.sh"],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    subprocess.run(
        ["pkill", "-f", "dimos --viewer rerun"],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


@app.post("/api/v1/patrol/halt")
def halt_patrol():
    """Stop where it stands. Do not walk home, then drop the stack so the dog lies down."""
    global patrol_proc
    with lock:
        proc = patrol_proc
        _control(paused=False, stop_requested=True, halt=True)
    if proc is not None and _running(proc):
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass
    _kill_patrol_group(proc)
    with lock:
        if patrol_proc is proc:
            patrol_proc = None
    _write_json(STATUS_FILE, {"phase": "idle", "message": "Halted in place. The dog will lie down."})
    return jsonify(ok=True, message="Halted in place. The dog will lie down.")


@app.post("/api/v1/patrol/pause")
def pause_patrol():
    if not _running(patrol_proc):
        return jsonify(error="No patrol task is running."), 409
    _event("Fire_alarm", "remote operator", "Pause requested from remote console")
    _control(paused=True)
    return jsonify(ok=True, state="Fire_alarm")


@app.post("/api/v1/patrol/resume")
def resume_patrol():
    control = _read_json(CONTROL_FILE, {})
    alarmed = isinstance(control, dict) and control.get("fire_state") == "Fire_alarm"
    # An alarm that arrived with no patrol running must still be clearable,
    # or the next patrol starts paused and the console keeps flashing.
    if not _running(patrol_proc) and not alarmed:
        return jsonify(error="No patrol task is running."), 409
    _event("Safe", "remote operator", "Operator confirmed scene is safe")
    _control(paused=False)
    return jsonify(ok=True, state="Safe")


def _keepouts_from_route(route: dict) -> list:
    zones = route.get("keepouts") or []
    if not isinstance(zones, list) or len(zones) > 20:
        raise ValueError("A route may contain at most 20 keepout zones.")
    cleaned = []
    for zone in zones:
        points = zone.get("points") if isinstance(zone, dict) else zone
        if not isinstance(points, list) or len(points) != 4:
            raise ValueError("Each keepout zone needs four corners.")
        quad = []
        for point in points:
            x = float(point["x"])
            y = float(point["y"])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError("Keepout corners must be finite.")
            quad.append({"x": x, "y": y})
        cleaned.append({"points": quad})
    return cleaned


def _walk_speed(body: dict) -> float:
    mode = str(body.get("speed_mode", "slow"))
    if mode == "slow":
        return SLOW_MPS
    if mode == "fast":
        return FAST_MPS
    if mode == "custom":
        value = float(body.get("speed_mps"))
        if not math.isfinite(value) or value < 0.05 or value > MAX_MPS:
            raise ValueError("Custom speed must be from 0.05 to 1 m/s.")
        return value
    raise ValueError("speed_mode must be slow, fast, or custom.")


@app.post("/api/v1/patrol/confirm-pose")
def confirm_pose():
    """Operator verified Rerun overlap; lock the walk speed and allow waypoints."""
    if not _running(patrol_proc):
        return jsonify(error="No patrol task is running."), 409
    current = _read_json(SPEED_FILE, {})
    if isinstance(current, dict) and current.get("locked"):
        return jsonify(error="Walk speed is already locked for this patrol."), 409
    body = request.get_json(silent=True) or {}
    try:
        mps = _walk_speed(body)
    except (TypeError, ValueError) as error:
        return jsonify(error=str(error)), 400
    _write_json(SPEED_FILE, {"mps": mps, "locked": True, "mode": body.get("speed_mode", "slow")})
    _control(pose_confirmed=True)
    saved = _format_initial_pose(_read_json(DOG_POSE_FILE, None))
    return jsonify(
        ok=True,
        message="Pose confirmed; navigation may start.",
        mps=mps,
        initial_pose=saved,
    )


@app.post("/api/v1/patrol/nudge-pose")
def nudge_pose():
    """Shift the locked dog pose on the loaded map by a small operator step."""
    body = request.get_json(silent=True) or {}
    try:
        dx = float(body.get("dx", 0.0))
        dy = float(body.get("dy", 0.0))
        dyaw_deg = float(body.get("dyaw_deg", 0.0))
    except (TypeError, ValueError):
        return jsonify(error="dx, dy and dyaw_deg must be numbers."), 400
    if not all(math.isfinite(v) for v in (dx, dy, dyaw_deg)):
        return jsonify(error="dx, dy and dyaw_deg must be finite."), 400
    if abs(dx) > 0.5 or abs(dy) > 0.5 or abs(dyaw_deg) > 15:
        return jsonify(error="One step is at most 0.5 m and 15 degrees."), 400
    current = _read_json(NUDGE_FILE, {})
    seq = int(current.get("seq", 0)) + 1 if isinstance(current, dict) else 1
    _write_json(NUDGE_FILE, {"seq": seq, "dx": dx, "dy": dy, "dyaw_deg": dyaw_deg})
    return jsonify(ok=True, seq=seq)


@app.get("/api/v1/logs")
def logs():
    return jsonify(
        mapping=_tail(MAPPING_LOG),
        patrol=_patrol_log_for_ui(),
        reloc=_reloc_log_tail(80),
    )


@app.get("/api/v1/wifi")
def wifi_status():
    from wifi_control import status as wifi_status_fn

    return jsonify(wifi_status_fn())


@app.post("/api/v1/wifi/prefer")
def wifi_prefer():
    body = request.get_json(silent=True) or {}
    profile = str(body.get("profile") or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9_-]+", profile or ""):
        return jsonify(error="profile must be a simple id (letters/digits/_/-)."), 400
    from wifi_control import prefer as wifi_prefer_fn

    result = wifi_prefer_fn(profile)
    code = 200 if result.get("ok") else 500
    return jsonify(result), code


@app.post("/api/v1/wifi/connect")
def wifi_connect():
    body = request.get_json(silent=True) or {}
    profile = str(body.get("profile") or "").strip()
    if profile and not re.fullmatch(r"[A-Za-z0-9_-]+", profile):
        return jsonify(error="profile must be a simple id (letters/digits/_/-)."), 400
    from wifi_control import connect as wifi_connect_fn

    result = wifi_connect_fn(profile or None)
    code = 200 if result.get("ok") else 500
    return jsonify(result), code


@app.get("/api/v1/time")
def time_status():
    return jsonify(ok=True, clock=_clock_snapshot())


@app.post("/api/v1/time/sync")
def time_sync():
    """Accept host UTC epoch and set Orin wall clock when skew is large."""
    body = request.get_json(silent=True) or {}
    try:
        epoch = int(body.get("epoch"))
    except (TypeError, ValueError):
        return jsonify(error="epoch must be a Unix timestamp integer"), 400
    # Reject absurd futures (>1 day ahead of what Python thinks) only after install;
    # before install the board clock may be years behind, so trust the host epoch.
    if epoch < 1_600_000_000:  # before ~2020-09
        return jsonify(error="epoch looks too old; send current host UTC seconds"), 400
    result = _apply_host_epoch(epoch)
    code = 200 if result.get("ok") else 503
    return jsonify(result), code


if __name__ == "__main__":
    host = os.environ.get("ROBOT_AGENT_HOST", "0.0.0.0")
    port = int(os.environ.get("ROBOT_AGENT_PORT", "8787"))
    if DIMOS_MODE == "real" and not TOKEN:
        raise SystemExit("ROBOT_AGENT_TOKEN is required; configure it in the robot service environment.")
    app.run(host=host, port=port, debug=False, threaded=True)
