#!/usr/bin/env python3
"""Create gitignored .env files from *.env.example templates.

Does not print secrets. Existing keys are preserved unless --force.
"""
from __future__ import annotations

import argparse
import secrets
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
AGENT_EXAMPLE = ROOT / "robot_agent" / ".env.example"
CLIENT_EXAMPLE = ROOT / "remote_client" / ".env.example"
AGENT_ENV = ROOT / "robot_agent" / ".env"
CLIENT_ENV = ROOT / "remote_client" / ".env"


def parse_env(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        values[key.strip()] = value
    return values


def load_env_file(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    return parse_env(path.read_text(encoding="utf-8"))


def render_from_example(example: Path, values: dict[str, str]) -> str:
    lines: list[str] = []
    for raw in example.read_text(encoding="utf-8").splitlines():
        stripped = raw.strip()
        if not stripped or stripped.startswith("#") or "=" not in raw:
            lines.append(raw)
            continue
        key, _, _ = raw.partition("=")
        key = key.strip()
        lines.append(f"{key}={values.get(key, '')}")
    if lines and lines[-1] != "":
        lines.append("")
    return "\n".join(lines)


def write_env(path: Path, example: Path, values: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_from_example(example, values), encoding="utf-8")
    path.chmod(0o600)


def merge_values(
    example: Path,
    existing: dict[str, str],
    overrides: dict[str, str],
    *,
    force: bool,
) -> dict[str, str]:
    base = parse_env(example.read_text(encoding="utf-8"))
    out = dict(base)
    if not force:
        out.update({k: v for k, v in existing.items() if v != ""})
    out.update({k: v for k, v in overrides.items() if v is not None and v != ""})
    return out


def ensure_token(values: dict[str, str], token: str | None, *, force: bool) -> str:
    current = values.get("ROBOT_AGENT_TOKEN", "").strip()
    placeholder = not current or current.startswith("replace-with")
    if token:
        values["ROBOT_AGENT_TOKEN"] = token
        return token
    if force or placeholder:
        generated = secrets.token_hex(24)
        values["ROBOT_AGENT_TOKEN"] = generated
        return generated
    return current


def read_token_via_ssh(target: str) -> str:
    import subprocess

    remote = (
        "set -euo pipefail; "
        "f=/home/unitree/dimos_Go2/robot_agent/.env; "
        "test -f \"$f\" || f=~/dimos_Go2/robot_agent/.env; "
        "grep -E '^ROBOT_AGENT_TOKEN=' \"$f\" | head -1 | cut -d= -f2-"
    )
    proc = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", target, remote],
        check=False,
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        raise SystemExit(
            f"Failed to pull token from {target}: {proc.stderr.strip() or proc.stdout.strip()}"
        )
    token = proc.stdout.strip()
    if not token or token.startswith("replace-with"):
        raise SystemExit(f"No usable ROBOT_AGENT_TOKEN on {target}")
    return token


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--role",
        choices=("host", "orin", "both"),
        default="both",
        help="Which .env files to write (default: both)",
    )
    parser.add_argument("--force", action="store_true", help="Overwrite existing non-empty values")
    parser.add_argument("--token", default="", help="Shared bearer token (otherwise generate/keep)")
    parser.add_argument(
        "--pull-token-from",
        default="",
        help="SSH target to copy ROBOT_AGENT_TOKEN from (e.g. go2-orin)",
    )
    parser.add_argument("--orin-host", default="", help="Override ORIN_AGENT_HOST / ROBOT_AGENT_URL host")
    parser.add_argument("--ssh-target", default="", help="Override ORIN_SSH_TARGET")
    parser.add_argument("--robot-ip", default="", help="Override ROBOT_IP on Orin env")
    args = parser.parse_args()

    if not AGENT_EXAMPLE.is_file() or not CLIENT_EXAMPLE.is_file():
        print("Missing .env.example templates", file=sys.stderr)
        return 2

    token = args.token.strip() or None
    if args.pull_token_from:
        token = read_token_via_ssh(args.pull_token_from.strip())

    wrote: list[str] = []

    if args.role in ("orin", "both"):
        agent_existing = load_env_file(AGENT_ENV)
        agent_overrides = {}
        if args.robot_ip:
            agent_overrides["ROBOT_IP"] = args.robot_ip
        agent_values = merge_values(AGENT_EXAMPLE, agent_existing, agent_overrides, force=args.force)
        ensure_token(agent_values, token, force=args.force)
        token = agent_values["ROBOT_AGENT_TOKEN"]
        write_env(AGENT_ENV, AGENT_EXAMPLE, agent_values)
        wrote.append(str(AGENT_ENV))

    if args.role in ("host", "both"):
        client_existing = load_env_file(CLIENT_ENV)
        client_overrides = {}
        if args.ssh_target:
            client_overrides["ORIN_SSH_TARGET"] = args.ssh_target
        if args.orin_host:
            host = args.orin_host
            client_overrides["ORIN_AGENT_HOST"] = host
            port = client_existing.get("ORIN_AGENT_PORT") or parse_env(
                CLIENT_EXAMPLE.read_text(encoding="utf-8")
            ).get("ORIN_AGENT_PORT", "8787")
            client_overrides["ROBOT_AGENT_URL"] = f"http://{host}:{port}"
        client_values = merge_values(
            CLIENT_EXAMPLE, client_existing, client_overrides, force=args.force
        )
        # Prefer Orin token when writing both, else generate/keep.
        ensure_token(client_values, token, force=args.force)
        if args.role == "both" and token:
            client_values["ROBOT_AGENT_TOKEN"] = token
        write_env(CLIENT_ENV, CLIENT_EXAMPLE, client_values)
        wrote.append(str(CLIENT_ENV))

        # Keep tokens matched when both exist on this machine.
        if AGENT_ENV.is_file() and CLIENT_ENV.is_file():
            agent_vals = load_env_file(AGENT_ENV)
            client_vals = load_env_file(CLIENT_ENV)
            if agent_vals.get("ROBOT_AGENT_TOKEN") != client_vals.get("ROBOT_AGENT_TOKEN"):
                agent_vals["ROBOT_AGENT_TOKEN"] = client_vals["ROBOT_AGENT_TOKEN"]
                write_env(AGENT_ENV, AGENT_EXAMPLE, merge_values(AGENT_EXAMPLE, agent_vals, {}, force=True))

    print("Wrote " + ", ".join(wrote))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
