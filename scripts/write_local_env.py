#!/usr/bin/env python3
"""Create gitignored .env files with a shared token. Does not print the token."""
from __future__ import annotations

import secrets
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOKEN = secrets.token_hex(24)
AGENT = ROOT / "robot_agent" / ".env"
CLIENT = ROOT / "remote_client" / ".env"

AGENT.write_text(
    "\n".join(
        [
            f"ROBOT_AGENT_TOKEN={TOKEN}",
            "ROBOT_AGENT_HOST=0.0.0.0",
            "ROBOT_AGENT_PORT=8787",
            "ROBOT_IP=192.168.123.161",
            "LIDAR_IP=192.168.123.20",
            "DIMOS_MODE=real",
            "",
        ]
    ),
    encoding="utf-8",
)
CLIENT.write_text(
    "\n".join(
        [
            f"ROBOT_AGENT_TOKEN={TOKEN}",
            "ROBOT_AGENT_URL=http://192.168.123.18:8787",
            "ORIN_SSH_TARGET=go2-orin",
            "ORIN_AGENT_HOST=192.168.123.18",
            "ORIN_AGENT_PORT=8787",
            "LOCAL_CC_PORT=17779",
            "LOCAL_RERUN_WEB_PORT=19878",
            "LOCAL_RERUN_GRPC_PORT=19877",
            "LOCAL_RERUN_WS_PORT=13030",
            "REMOTE_CONSOLE_HOST=127.0.0.1",
            "REMOTE_CONSOLE_PORT=7781",
            "",
        ]
    ),
    encoding="utf-8",
)
AGENT.chmod(0o600)
CLIENT.chmod(0o600)
print(f"Wrote {AGENT} and {CLIENT}")
