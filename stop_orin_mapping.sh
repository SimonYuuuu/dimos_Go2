#!/usr/bin/env bash
# Host helper: stop Orin mapping DimOS (not the host console, not the agent).
# Use this when 「关建图台」后 Orin 上点云/DimOS 还在跑。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/load_env.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/link_profile.sh"

dimos_parse_link_args "$@"
LINK_PROFILE="${LINK_CLI:-${LINK_PROFILE:-wire}}"
_dimos_load_env "$ROOT/.env" "$ROOT/remote_client/.env"
dimos_apply_link_profile "$LINK_PROFILE"
dimos_ensure_link_ssh_aliases

echo "Stopping Orin mapping via ${ORIN_SSH_TARGET} (agent kept)..."
ssh -o BatchMode=yes -o ConnectTimeout=12 "$ORIN_SSH_TARGET" bash -s <<'REMOTE'
set -u
cd /home/unitree/dimos_Go2

# Prefer agent API so export can run if a tracked session exists.
if [[ -f robot_agent/.env ]]; then
  set -a
  # shellcheck disable=SC1091
  source robot_agent/.env
  set +a
fi
AGENT_HOST=127.0.0.1
AGENT_PORT="${ROBOT_AGENT_PORT:-8787}"
if [[ -n "${ROBOT_AGENT_TOKEN:-}" ]]; then
  curl -fsS --max-time 25 -X POST \
    -H "Authorization: Bearer ${ROBOT_AGENT_TOKEN}" \
    -H "Content-Type: application/json" \
    -d '{}' \
    "http://${AGENT_HOST}:${AGENT_PORT}/api/v1/mapping/stop" \
    >/tmp/dimos-mapping-stop.json 2>/tmp/dimos-mapping-stop.err \
    && echo "agent mapping/stop: $(tr '\n' ' ' </tmp/dimos-mapping-stop.json)" \
    || echo "agent mapping/stop skipped ($(tr '\n' ' ' </tmp/dimos-mapping-stop.err 2>/dev/null))"
fi

stop_match() {
  local pattern="$1"
  pgrep -f "$pattern" >/dev/null 2>&1 && pkill -TERM -f "$pattern" >/dev/null 2>&1 || true
}
force_match() {
  local pattern="$1"
  pgrep -f "$pattern" >/dev/null 2>&1 && pkill -KILL -f "$pattern" >/dev/null 2>&1 || true
}

stop_match '[s]tart_mapping_quality'
stop_match '[s]tart_mapping\.sh'
stop_match '[d]imos --viewer'
stop_match '[d]imos run'
stop_match 'dimos.core.coordination.watchdog_main'
stop_match 'unitree-go2-memory'
sleep 2
force_match '[s]tart_mapping_quality'
force_match '[s]tart_mapping\.sh'
force_match '[d]imos --viewer'
force_match '[d]imos run'
force_match 'dimos.core.coordination.watchdog_main'
force_match 'unitree-go2-memory'
for p in 7779 9877 9878 3030; do
  fuser -k "${p}/tcp" >/dev/null 2>&1 || true
done
sleep 0.5

left="$(pgrep -af '[s]tart_mapping|[d]imos --viewer|[d]imos run|unitree-go2-memory|watchdog_main' || true)"
ports="$(ss -ltnH 2>/dev/null | awk '$4 ~ /:(7779|9877|9878|3030)$/' || true)"
if [[ -n "$left" || -n "$ports" ]]; then
  echo "WARN: leftovers:"
  [[ -n "$left" ]] && echo "$left"
  [[ -n "$ports" ]] && echo "$ports"
  exit 1
fi
echo "Orin mapping stopped. Viz ports free. Agent not touched."
REMOTE
