#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${ROBOT_AGENT_TOKEN:-}" ]]; then
  echo "Set ROBOT_AGENT_TOKEN in the service environment; do not commit it." >&2
  exit 2
fi
if [[ -z "${ROBOT_IP:-}" ]]; then
  echo "Set ROBOT_IP to the Go2's private-network address." >&2
  exit 2
fi
export DIMOS_MODE=real
exec "$ROOT/simulation_legacy/.venv/bin/python" "$ROOT/robot_agent/agent.py"
