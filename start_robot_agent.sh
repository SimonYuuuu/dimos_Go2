#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/load_env.sh"
_dimos_load_env "$ROOT/.env" "$ROOT/robot_agent/.env"

export DIMOS_MODE="${DIMOS_MODE:-real}"
export ROBOT_AGENT_HOST="${ROBOT_AGENT_HOST:-0.0.0.0}"
export ROBOT_AGENT_PORT="${ROBOT_AGENT_PORT:-8787}"
export ROBOT_IP="${ROBOT_IP:-192.168.123.161}"
# DimOS 0.0.15 rejects UNITREE_AES_128_KEY as a global env name.
if [[ -n "${UNITREE_AES_128_KEY:-}" && -z "${GO2CONNECTION__AES_128_KEY:-}" ]]; then
  export GO2CONNECTION__AES_128_KEY="$UNITREE_AES_128_KEY"
fi
unset UNITREE_AES_128_KEY || true

if [[ -z "${ROBOT_AGENT_TOKEN:-}" ]]; then
  echo "Set ROBOT_AGENT_TOKEN in robot_agent/.env; do not commit it." >&2
  exit 2
fi
if [[ -z "${ROBOT_IP:-}" ]]; then
  echo "Set ROBOT_IP to the Go2 motion-control address (usually 192.168.123.161)." >&2
  exit 2
fi

PYTHON="$ROOT/simulation_legacy/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "Missing DimOS venv at $PYTHON" >&2
  exit 2
fi

echo "Robot agent listening on ${ROBOT_AGENT_HOST}:${ROBOT_AGENT_PORT}; Go2 MCU ${ROBOT_IP}"
exec "$PYTHON" "$ROOT/robot_agent/agent.py"
