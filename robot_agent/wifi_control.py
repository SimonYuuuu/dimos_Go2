"""Preferred WiFi profile helpers for the Orin robot agent."""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(os.environ.get("DIMOS_WIFI_CONFIG", str(ROOT / "robot_agent" / "wifi_profiles.json")))
HELPER_CANDIDATES = (
    Path("/usr/local/sbin/dimos-orin-wifi"),
    ROOT / "scripts" / "orin_wifi.sh",
)


def _helper() -> Path | None:
    for path in HELPER_CANDIDATES:
        if path.is_file() and os.access(path, os.X_OK):
            return path
    return None


def _load() -> dict[str, Any]:
    try:
        data = json.loads(CONFIG.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {"preferred": "", "profiles": {}}
    if not isinstance(data, dict):
        return {"preferred": "", "profiles": {}}
    profiles = data.get("profiles")
    if not isinstance(profiles, dict):
        profiles = {}
    return {"preferred": str(data.get("preferred") or ""), "profiles": profiles}


def _run(args: list[str], timeout: float = 45.0) -> tuple[int, str]:
    helper = _helper()
    if helper is None:
        return 2, "orin_wifi helper missing; sync scripts/orin_wifi.sh to the Orin."
    cmd = [str(helper), *args]
    # Prefer installed NOPASSWD helper.
    if helper == Path("/usr/local/sbin/dimos-orin-wifi"):
        cmd = ["sudo", "-n", *cmd]
    try:
        completed = subprocess.run(
            cmd,
            cwd=str(ROOT),
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return 1, str(error)
    text = ((completed.stdout or "") + (completed.stderr or "")).strip()
    return int(completed.returncode), text


def _active_wifi() -> dict[str, str] | None:
    try:
        completed = subprocess.run(
            ["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "connection", "show", "--active"],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    for line in (completed.stdout or "").splitlines():
        parts = line.split(":")
        if len(parts) >= 3 and parts[1] == "802-11-wireless":
            return {"name": parts[0], "device": parts[2]}
    return None


def _nm_exists(name: str) -> bool:
    try:
        completed = subprocess.run(
            ["nmcli", "-t", "-f", "NAME", "connection", "show"],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return name in {(line.split(":", 1)[0]) for line in (completed.stdout or "").splitlines()}


def status() -> dict[str, Any]:
    data = _load()
    preferred = data["preferred"]
    profiles_out = []
    for pid, meta in (data["profiles"] or {}).items():
        if not isinstance(meta, dict):
            continue
        nm_name = str(meta.get("nm_name") or "")
        profiles_out.append(
            {
                "id": pid,
                "label": str(meta.get("label") or pid),
                "nm_name": nm_name,
                "priority": int(meta.get("priority") or 0),
                "preferred": pid == preferred,
                "configured": bool(nm_name) and _nm_exists(nm_name),
            }
        )
    helper = _helper()
    installed = helper == Path("/usr/local/sbin/dimos-orin-wifi")
    code, text = (0, "")
    if installed:
        code, text = _run(["status"], timeout=15)
    return {
        "preferred": preferred,
        "active": _active_wifi(),
        "profiles": profiles_out,
        "helper": str(helper) if helper else None,
        "installed": installed,
        "needs_install": not installed,
        "helper_status": text if code == 0 else text,
        "install_hint": (
            "On Orin (once, over wire): sudo /home/unitree/dimos_Go2/scripts/orin_wifi.sh install"
            if not installed
            else ""
        ),
    }


def prefer(profile_id: str) -> dict[str, Any]:
    code, text = _run(["prefer", profile_id])
    if code != 0:
        return {"ok": False, "error": text or f"prefer failed ({code})"}
    return {"ok": True, "preferred": profile_id, "detail": text}


def connect(profile_id: str | None = None) -> dict[str, Any]:
    args = ["connect"]
    if profile_id:
        args.append(profile_id)
    code, text = _run(args, timeout=60)
    if code != 0:
        return {"ok": False, "error": text or f"connect failed ({code})"}
    return {"ok": True, "detail": text, "active": _active_wifi()}
