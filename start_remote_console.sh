#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${ROBOT_AGENT_URL:-}" || -z "${ROBOT_AGENT_TOKEN:-}" ]]; then
  echo "Set ROBOT_AGENT_URL and ROBOT_AGENT_TOKEN on the remote client." >&2
  exit 2
fi
exec "$ROOT/remote_client/.venv/bin/python" "$ROOT/remote_client/console.py"
