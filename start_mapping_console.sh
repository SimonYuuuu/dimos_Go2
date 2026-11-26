#!/usr/bin/env bash
# Host: slow-mapping console (separate from patrol console).
# Opens http://127.0.0.1:7782/ — quality mapping + route/keepout edit + export.
# Orin DimOS is still started via robot agent → start_mapping_quality.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/load_env.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/link_profile.sh"

dimos_parse_link_args "$@"
if [[ ${#DIMOS_PASSTHROUGH_ARGS[@]} -gt 0 ]]; then
  echo "Unexpected arguments: ${DIMOS_PASSTHROUGH_ARGS[*]}" >&2
  echo "Usage: ./start_mapping_console.sh [--link wire|iphone|guest|dog_ap]" >&2
  exit 2
fi

_dimos_load_env "$ROOT/.env" "$ROOT/remote_client/.env"

LINK_PROFILE="${LINK_CLI:-${LINK_PROFILE:-wire}}"
dimos_apply_link_profile "$LINK_PROFILE"
dimos_ensure_link_ssh_aliases

LOCAL_CC_PORT="${LOCAL_CC_PORT:-7779}"
LOCAL_RERUN_WEB_PORT="${LOCAL_RERUN_WEB_PORT:-9878}"
LOCAL_RERUN_GRPC_PORT="${LOCAL_RERUN_GRPC_PORT:-9877}"
LOCAL_RERUN_WS_PORT="${LOCAL_RERUN_WS_PORT:-3030}"

export DIMOS_CONSOLE_ROLE=mapping
export REMOTE_CONSOLE_HOST="${REMOTE_CONSOLE_HOST:-127.0.0.1}"
# Always separate from patrol console (7781); ignore REMOTE_CONSOLE_PORT from .env.
export REMOTE_CONSOLE_PORT="${MAPPING_CONSOLE_PORT:-7782}"
export NO_PROXY="${NO_PROXY:-127.0.0.1,localhost,${ORIN_AGENT_HOST}}"
export no_proxy="$NO_PROXY"

if [[ -z "${ROBOT_AGENT_TOKEN:-}" ]]; then
  echo "Missing ROBOT_AGENT_TOKEN. Run: ./scripts/bootstrap.sh --role host" >&2
  echo "If Orin is already set up: ./scripts/bootstrap.sh --role host --pull-token-from ${ORIN_SSH_TARGET}" >&2
  exit 2
fi

if ! dimos_preflight_link "$ORIN_AGENT_HOST" "$ORIN_SSH_TARGET" "$ORIN_AGENT_PORT"; then
  echo "Trying ./ensure_orin_agent.sh to bring agent back..." >&2
  if ! "$ROOT/ensure_orin_agent.sh" --link "$LINK_PROFILE"; then
    exit 1
  fi
fi

port_listener_cmd() {
  local port="$1"
  ss -ltnp 2>/dev/null | grep -E ":${port}\\b" | sed -n 's/.*users:((\"\([^\"]*\)\".*/\1/p' | head -1
}

port_open() {
  local port="$1"
  ss -ltnH 2>/dev/null | awk '{print $4}' | grep -Eq ":${port}$"
}

pick_free_port() {
  local preferred="$1"
  shift
  local candidate
  for candidate in "$preferred" "$@"; do
    if ! port_open "$candidate"; then
      echo "$candidate"
      return 0
    fi
    local owner
    owner="$(port_listener_cmd "$candidate" || true)"
    if [[ "$owner" == "ssh" ]]; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

ensure_local_forward() {
  local var_name="$1"
  local remote_port="$2"
  shift 2
  local preferred="${!var_name}"
  local local_port owner
  if ! local_port="$(pick_free_port "$preferred" "$@")"; then
    echo "ERROR: no free local port for Orin:${remote_port} (tried ${preferred} $*)." >&2
    return 1
  fi
  if [[ "$local_port" != "$preferred" ]]; then
    echo "Port ${preferred} busy; using ${local_port} -> Orin:${remote_port}"
  fi
  printf -v "$var_name" '%s' "$local_port"

  if port_open "$local_port"; then
    owner="$(port_listener_cmd "$local_port" || true)"
    if forward_is_ours "$local_port" "$remote_port"; then
      echo "Reusing local port ${local_port} (${owner:-unknown}) -> Orin:${remote_port}"
      return 0
    fi
    # A forward left over from another link/Orin address silently swallows keyboard commands.
    if project_forward "$local_port"; then
      echo "Replacing stale forward on ${local_port} (not -> ${ORIN_SSH_TARGET}:${remote_port})"
      kill "$(port_listener_pid "$local_port")" 2>/dev/null || true
      sleep 0.5
    else
      echo "WARN: local port ${local_port} is an unrelated ssh; reusing it -> Orin:${remote_port} unverified." >&2
      return 0
    fi
  fi
  start_forward "$local_port" "$remote_port"
}

port_listener_pid() {
  ss -ltnpH 2>/dev/null | awk -v p=":${1}$" '$4 ~ p' | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | head -1
}

forward_is_ours() {
  local pid cmd
  pid="$(port_listener_pid "$1")"
  [[ -n "$pid" ]] || return 1
  cmd="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  [[ "$cmd" == *"127.0.0.1:${1}:127.0.0.1:${2} "* && "$cmd" == *" ${ORIN_SSH_TARGET} " ]]
}

# True for forwards this script family creates (ssh -fNT ... -L 127.0.0.1:<port>:127.0.0.1:<n>).
project_forward() {
  local pid cmd
  pid="$(port_listener_pid "$1")"
  [[ -n "$pid" ]] || return 1
  cmd="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  [[ "$cmd" == *"ssh -fNT "* && "$cmd" == *"-L 127.0.0.1:${1}:127.0.0.1:"* ]]
}

start_forward() {
  local local_port="$1" remote_port="$2"
  echo "Forwarding 127.0.0.1:${local_port} -> ${ORIN_SSH_TARGET}:127.0.0.1:${remote_port}"
  ssh -fNT \
    -o BatchMode=yes \
    -o ConnectTimeout=8 \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=5 \
    -o ServerAliveCountMax=3 \
    -L "127.0.0.1:${local_port}:127.0.0.1:${remote_port}" \
    "$ORIN_SSH_TARGET"
}

# ssh -f forwards exit after a WiFi drop and are not restarted; bring them back while the console runs.
tunnel_watchdog() {
  local owner_pid="$1"
  while kill -0 "$owner_pid" 2>/dev/null; do
    sleep 5
    kill -0 "$owner_pid" 2>/dev/null || return 0
    local pair
    for pair in "${LOCAL_CC_PORT}:7779" "${LOCAL_RERUN_WEB_PORT}:9878" "${LOCAL_RERUN_GRPC_PORT}:9877"; do
      if ! port_open "${pair%%:*}"; then
        start_forward "${pair%%:*}" "${pair##*:}" 2>&1 || true
      fi
    done
  done
}

if port_open "$REMOTE_CONSOLE_PORT"; then
  echo "ERROR: mapping console port ${REMOTE_CONSOLE_PORT} already in use." >&2
  echo "Stop with: ./stop_mapping_console.sh" >&2
  exit 1
fi

ensure_local_forward LOCAL_CC_PORT 7779 17779 27779
ensure_local_forward LOCAL_RERUN_WEB_PORT 9878 19878 29878
ensure_local_forward LOCAL_RERUN_GRPC_PORT 9877 19877 29877
ensure_local_forward LOCAL_RERUN_WS_PORT 3030 13030 23030 || true

if [[ "$LOCAL_CC_PORT" != "7779" ]]; then
  echo "WARN: Command Center tunnel is on ${LOCAL_CC_PORT}, but the page hardcodes ws://localhost:7779." >&2
  echo "      Free local 7779 and restart, or CC will look empty." >&2
fi

export ROBOT_COMMAND_CENTER_URL="http://127.0.0.1:${LOCAL_CC_PORT}/command-center"
export ROBOT_RERUN_WEB_ORIGIN="http://127.0.0.1:${LOCAL_RERUN_WEB_PORT}"
export ROBOT_RERUN_URL="http://127.0.0.1:${REMOTE_CONSOLE_PORT}/rerun-viewer/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A${LOCAL_RERUN_GRPC_PORT}%2Fproxy"
echo "Rerun: console cache proxy → ${ROBOT_RERUN_URL}"

PYTHON="$ROOT/remote_client/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "Missing remote-client venv at $PYTHON" >&2
  echo "Run: ./scripts/bootstrap.sh --role host" >&2
  exit 2
fi

mkdir -p "$ROOT/remote_client/data"
# Mark that this process owns viz tunnels for stop_mapping_console (shared with patrol if both up).
printf '%s\n' "$$" >"$ROOT/remote_client/data/mapping_console_launcher.pid"

echo "Mapping console on http://${REMOTE_CONSOLE_HOST}:${REMOTE_CONSOLE_PORT}/"
echo "Command Center: ${ROBOT_COMMAND_CENTER_URL}"
echo "Rerun:          ${ROBOT_RERUN_URL}"
echo "Stop with:      ./stop_mapping_console.sh"
echo "Orin mapping:   agent → ./start_mapping_quality.sh (UI 启动慢速建图)"

if [[ -n "${ROBOT_AGENT_TOKEN:-}" && -n "${ORIN_AGENT_HOST:-}" ]]; then
  _epoch="$(date -u +%s)"
  _agent_base="http://${ORIN_AGENT_HOST}:${ORIN_AGENT_PORT:-8787}"
  if curl -fsS --max-time 8 \
    -H "Authorization: Bearer ${ROBOT_AGENT_TOKEN}" \
    -H "Content-Type: application/json" \
    -d "{\"epoch\":${_epoch}}" \
    "${_agent_base}/api/v1/time/sync" >/tmp/dimos-time-sync.json 2>/tmp/dimos-time-sync.err; then
    echo "Orin clock sync: $(tr '\n' ' ' </tmp/dimos-time-sync.json)"
  else
    echo "WARN: Orin clock sync skipped."
  fi
fi

# Same PID after exec, so the watchdog ends with the console.
tunnel_watchdog "$$" >>"$ROOT/remote_client/data/mapping_tunnel_watchdog.log" 2>&1 &

exec "$PYTHON" "$ROOT/remote_client/console.py"
