#!/usr/bin/env bash
# Orin: stop a background mapping session (start_mapping*.sh / unitree-go2-memory).
# Prefer UI「结束采集并导出」so agent runs PGO export. This script is for hung jobs.
# Does NOT kill robot agent or patrol. Dog lies down when DimOS WebRTC drops.
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$ROOT/.." && pwd)"

echo "Stopping Orin mapping DimOS (agent kept alive). Dog will lie down if connected."

stop_match() {
  local pattern="$1"
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    pkill -TERM -f "$pattern" >/dev/null 2>&1 || true
  fi
}

force_match() {
  local pattern="$1"
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    pkill -KILL -f "$pattern" >/dev/null 2>&1 || true
  fi
}

# Try agent API first so export can still run (token from robot_agent/.env).
if [[ -f "$REPO/robot_agent/.env" ]]; then
  # shellcheck disable=SC1091
  set -a
  # shellcheck disable=SC1091
  source "$REPO/robot_agent/.env"
  set +a
fi
AGENT_HOST="${ROBOT_AGENT_HOST:-127.0.0.1}"
[[ "$AGENT_HOST" == "0.0.0.0" ]] && AGENT_HOST="127.0.0.1"
AGENT_PORT="${ROBOT_AGENT_PORT:-8787}"
if [[ -n "${ROBOT_AGENT_TOKEN:-}" ]]; then
  if curl -fsS --max-time 30 \
    -X POST \
    -H "Authorization: Bearer ${ROBOT_AGENT_TOKEN}" \
    -H "Content-Type: application/json" \
    -d '{}' \
    "http://${AGENT_HOST}:${AGENT_PORT}/api/v1/mapping/stop" >/tmp/dimos-mapping-stop.json 2>/tmp/dimos-mapping-stop.err; then
    echo "Agent mapping/stop OK: $(tr '\n' ' ' </tmp/dimos-mapping-stop.json)"
    echo "If export_started, wait for .pc2.lcm / .map.db under simulation_legacy/maps/."
    exit 0
  fi
  echo "Agent mapping/stop not available; falling back to process kill."
  [[ -s /tmp/dimos-mapping-stop.err ]] && sed -n '1,3p' /tmp/dimos-mapping-stop.err >&2 || true
fi

# Shell wrappers first so they cannot respawn DimOS.
stop_match '[s]tart_mapping_quality\.sh'
stop_match '[s]tart_mapping\.sh'
# SIGINT-like stop for DimOS memory recorder (same family as mapping).
stop_match '[d]imos --viewer'
stop_match '[d]imos run'
stop_match 'dimos.core.coordination.watchdog_main'
sleep 2

force_match '[s]tart_mapping_quality\.sh'
force_match '[s]tart_mapping\.sh'
force_match '[d]imos --viewer'
force_match '[d]imos run'
force_match 'dimos.core.coordination.watchdog_main'
force_match 'simulation_legacy/.venv/bin/python -c from multiprocessing.forkserver'
force_match 'simulation_legacy/.venv/bin/python -c from multiprocessing.resource_tracker'
sleep 0.5

left="$(pgrep -af '[s]tart_mapping|[d]imos --viewer|[d]imos run' || true)"
# Ignore patrol-only leftovers if any dimos still tied to patrol — user should use stop_dimos / stop patrol.
if echo "$left" | grep -E 'start_mapping|unitree-go2-memory' >/dev/null 2>&1; then
  echo "Some mapping-related processes are still running:"
  echo "$left"
  exit 1
fi

echo "Mapping processes stopped. Robot agent left running."
echo "To export a leftover .db: mapping console「优化并导出」, or agent maps/<name>/export."
