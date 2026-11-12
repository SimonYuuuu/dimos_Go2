#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/load_env.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/link_profile.sh"

dimos_parse_link_args "$@"
if [[ ${#DIMOS_PASSTHROUGH_ARGS[@]} -gt 0 ]]; then
  echo "Unexpected arguments: ${DIMOS_PASSTHROUGH_ARGS[*]}" >&2
  echo "Usage: ./start_remote_console.sh [--link wire|guest|dog_ap]" >&2
  exit 2
fi

_dimos_load_env "$ROOT/.env" "$ROOT/remote_client/.env"

LINK_PROFILE="${LINK_CLI:-${LINK_PROFILE:-wire}}"
dimos_apply_link_profile "$LINK_PROFILE"
dimos_ensure_link_ssh_aliases

# Prefer ports that do NOT collide with VS Code Remote-SSH (it often grabs 7779/9877/9878/3030).
LOCAL_CC_PORT="${LOCAL_CC_PORT:-17779}"
LOCAL_RERUN_WEB_PORT="${LOCAL_RERUN_WEB_PORT:-19878}"
LOCAL_RERUN_GRPC_PORT="${LOCAL_RERUN_GRPC_PORT:-19877}"
LOCAL_RERUN_WS_PORT="${LOCAL_RERUN_WS_PORT:-13030}"

export REMOTE_CONSOLE_HOST="${REMOTE_CONSOLE_HOST:-127.0.0.1}"
export REMOTE_CONSOLE_PORT="${REMOTE_CONSOLE_PORT:-7781}"
export NO_PROXY="${NO_PROXY:-127.0.0.1,localhost,${ORIN_AGENT_HOST}}"
export no_proxy="$NO_PROXY"

if [[ -z "${ROBOT_AGENT_TOKEN:-}" ]]; then
  echo "Missing ROBOT_AGENT_TOKEN. Run: ./scripts/bootstrap.sh --role host" >&2
  echo "If Orin is already set up: ./scripts/bootstrap.sh --role host --pull-token-from ${ORIN_SSH_TARGET}" >&2
  exit 2
fi

if ! dimos_preflight_link "$ORIN_AGENT_HOST" "$ORIN_SSH_TARGET" "$ORIN_AGENT_PORT"; then
  exit 1
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
  # Usage: ensure_local_forward <varname_for_local_port> <remote_port> <fallback...>
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
    echo "Reusing local port ${local_port} (${owner:-unknown}) -> Orin:${remote_port}"
    return 0
  fi
  echo "Forwarding 127.0.0.1:${local_port} -> ${ORIN_SSH_TARGET}:127.0.0.1:${remote_port}"
  ssh -fNT \
    -o BatchMode=yes \
    -o ConnectTimeout=8 \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=15 \
    -o ServerAliveCountMax=3 \
    -L "127.0.0.1:${local_port}:127.0.0.1:${remote_port}" \
    "$ORIN_SSH_TARGET"
}

ensure_local_forward LOCAL_CC_PORT 7779 17779 27779
ensure_local_forward LOCAL_RERUN_WEB_PORT 9878 19878 29878
ensure_local_forward LOCAL_RERUN_GRPC_PORT 9877 19877 29877
ensure_local_forward LOCAL_RERUN_WS_PORT 3030 13030 23030 || true

# Always use host-local tunnel ports (VS Code often owns 7779/9877/9878).
export ROBOT_COMMAND_CENTER_URL="http://127.0.0.1:${LOCAL_CC_PORT}/command-center"
export ROBOT_RERUN_URL="http://127.0.0.1:${LOCAL_RERUN_WEB_PORT}/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A${LOCAL_RERUN_GRPC_PORT}%2Fproxy"

PYTHON="$ROOT/remote_client/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "Missing remote-client venv at $PYTHON" >&2
  echo "Run: ./scripts/bootstrap.sh --role host" >&2
  exit 2
fi

echo "Remote console on http://${REMOTE_CONSOLE_HOST}:${REMOTE_CONSOLE_PORT}/"
echo "Command Center: ${ROBOT_COMMAND_CENTER_URL}"
echo "Rerun:          ${ROBOT_RERUN_URL}"
exec "$PYTHON" "$ROOT/remote_client/console.py"
