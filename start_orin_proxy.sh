#!/usr/bin/env bash
set -euo pipefail

TARGET="${ORIN_SSH_TARGET:-go2-orin}"
PORT="${ORIN_SOCKS_PORT:-1080}"
PROXY="socks5h://127.0.0.1:${PORT}"
TUNNEL_PID=""

proxy_ready() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$TARGET" \
    "curl -fsS --proxy '$PROXY' --max-time 8 https://pypi.org/pypi/pip/json -o /dev/null" \
    >/dev/null 2>&1
}

cleanup() {
  if [[ -n "$TUNNEL_PID" ]]; then
    kill "$TUNNEL_PID" 2>/dev/null || true
    wait "$TUNNEL_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if proxy_ready; then
  echo "Reusing the existing Orin SOCKS tunnel on 127.0.0.1:${PORT}."
else
  echo "Starting SSH SOCKS tunnel to ${TARGET}..."
  ssh -NT \
    -o BatchMode=yes \
    -o ConnectTimeout=8 \
    -o ControlMaster=no \
    -o ControlPath=none \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=20 \
    -o ServerAliveCountMax=3 \
    -R "127.0.0.1:${PORT}" \
    "$TARGET" &
  TUNNEL_PID=$!

  ready=false
  for _ in {1..20}; do
    if proxy_ready; then
      ready=true
      break
    fi
    if ! kill -0 "$TUNNEL_PID" 2>/dev/null; then
      wait "$TUNNEL_PID" || true
      echo "Could not establish the SSH tunnel." >&2
      exit 1
    fi
    sleep 0.25
  done
  if [[ "$ready" != true ]]; then
    echo "Tunnel started, but the external connectivity check failed." >&2
    exit 1
  fi
fi

echo "Proxy ready. curl and pip in this Orin shell will use the host connection."
ssh -tt -o ConnectTimeout=8 "$TARGET" \
  "export ALL_PROXY='$PROXY' all_proxy='$PROXY' HTTP_PROXY='$PROXY' HTTPS_PROXY='$PROXY' PIP_PROXY='$PROXY'; exec bash -l"