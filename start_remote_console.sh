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

# Command Center hardcodes ws://localhost:7779 — local forward MUST be 7779 when possible.
# Rerun web works with alternate ports if ?url= points at the matching local gRPC forward.
# Fallbacks avoid VS Code Remote-SSH collisions on 7779/9877/9878/3030.
LOCAL_CC_PORT="${LOCAL_CC_PORT:-7779}"
LOCAL_RERUN_WEB_PORT="${LOCAL_RERUN_WEB_PORT:-9878}"
LOCAL_RERUN_GRPC_PORT="${LOCAL_RERUN_GRPC_PORT:-9877}"
LOCAL_RERUN_WS_PORT="${LOCAL_RERUN_WS_PORT:-3030}"

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

if [[ "$LOCAL_CC_PORT" != "7779" ]]; then
  echo "WARN: Command Center tunnel is on ${LOCAL_CC_PORT}, but the page hardcodes ws://localhost:7779." >&2
  echo "      Free local 7779 (often VS Code port-forward) and restart, or CC will look empty." >&2
fi

# Command Center keeps host-local tunnel (needs ws://localhost:7779).
export ROBOT_COMMAND_CENTER_URL="http://127.0.0.1:${LOCAL_CC_PORT}/command-center"

# Rerun gRPC CORS only allows Origin http://127.0.0.1:<web-port> (not raw Orin LAN/WiFi IPs).
# Tunnel web+gRPC locally; serve the web UI through the console cache proxy so the ~40MB
# wasm is fetched once on the host (Orin's clock is often wrong and defeats browser cache).
export ROBOT_RERUN_WEB_ORIGIN="http://127.0.0.1:${LOCAL_RERUN_WEB_PORT}"
export ROBOT_RERUN_URL="http://127.0.0.1:${REMOTE_CONSOLE_PORT}/rerun-viewer/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A${LOCAL_RERUN_GRPC_PORT}%2Fproxy"
echo "Rerun: console cache proxy → ${ROBOT_RERUN_URL}"
echo "      First load caches re_viewer_bg.wasm under remote_client/data/rerun_web_cache/."
if [[ "${LINK_PROFILE}" == "iphone" || "${LINK_PROFILE}" == "guest" || "${LINK_PROFILE}" == "dog_ap" ]]; then
  echo "      Wireless: streams follow the link bandwidth (no fixed rate cap unless DIMOS_RERUN_LITE=1)."
fi

PYTHON="$ROOT/remote_client/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "Missing remote-client venv at $PYTHON" >&2
  echo "Run: ./scripts/bootstrap.sh --role host" >&2
  exit 2
fi

echo "Remote console on http://${REMOTE_CONSOLE_HOST}:${REMOTE_CONSOLE_PORT}/"
echo "Command Center: ${ROBOT_COMMAND_CENTER_URL}"
echo "Rerun:          ${ROBOT_RERUN_URL}"

# Push host UTC to Orin so Rerun cache / TLS work even before NTP is online.
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
    echo "WARN: Orin clock sync skipped (install timesync on Orin, or agent down)."
    echo "      sudo ./scripts/orin_timesync.sh install"
    [[ -s /tmp/dimos-time-sync.err ]] && sed -n '1,3p' /tmp/dimos-time-sync.err >&2 || true
  fi
fi

exec "$PYTHON" "$ROOT/remote_client/console.py"
