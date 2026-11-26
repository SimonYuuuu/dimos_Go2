#!/usr/bin/env bash
# Host helper: make sure Orin robot agent is listening on :8787.
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

AGENT_URL="http://${ORIN_AGENT_HOST}:${ORIN_AGENT_PORT:-8787}/health"
if curl -fsS --max-time 3 "$AGENT_URL" >/dev/null 2>&1; then
  echo "Orin agent already up: $AGENT_URL"
  curl -fsS --max-time 3 "$AGENT_URL"; echo
  exit 0
fi

echo "Orin agent down; starting on ${ORIN_SSH_TARGET}..."
# Double-fork so SSH does not wait on the agent.
ssh -o BatchMode=yes -o ConnectTimeout=12 "$ORIN_SSH_TARGET" \
  'cd /home/unitree/dimos_Go2 && (setsid nohup ./start_robot_agent.sh >/tmp/robot_agent.log 2>&1 </dev/null & echo $! > /tmp/robot_agent.pid)'

for _ in 1 2 3 4 5 6 7 8 9 10; do
  if curl -fsS --max-time 2 "$AGENT_URL" >/dev/null 2>&1; then
    echo "Orin agent ready: $AGENT_URL"
    curl -fsS --max-time 3 "$AGENT_URL"; echo
    exit 0
  fi
  sleep 0.5
done

echo "ERROR: agent did not become healthy. Tail on Orin:" >&2
ssh -o BatchMode=yes -o ConnectTimeout=12 "$ORIN_SSH_TARGET" 'tail -40 /tmp/robot_agent.log; pgrep -af agent.py | grep -v grep || true' >&2 || true
exit 1
