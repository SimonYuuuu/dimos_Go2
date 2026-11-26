#!/usr/bin/env bash
# Stop the patrol remote console and (by default) local SSH tunnels from start_remote_console.sh.
# Does not stop the mapping console on 7782 — use ./stop_mapping_console.sh for that.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="$ROOT/remote_client/data"
PID_FILE="$DATA/patrol_console.pid"
LAUNCHER_PID_FILE="$DATA/patrol_console_launcher.pid"
MAPPING_PID_FILE="$DATA/mapping_console.pid"
PORT="${REMOTE_CONSOLE_PORT:-7781}"

kill_port() {
  local port="$1"
  if command -v fuser >/dev/null 2>&1; then
    fuser -k "${port}/tcp" >/dev/null 2>&1 || true
  fi
  if command -v ss >/dev/null 2>&1; then
    local pids
    pids="$(ss -ltnp 2>/dev/null | awk -v p=":${port}" '$4 ~ p"$"{print}' | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | sort -u)"
    for pid in $pids; do
      kill "$pid" 2>/dev/null || true
    done
  fi
}

kill_pidfile() {
  local file="$1"
  if [[ -f "$file" ]]; then
    local pid
    pid="$(tr -d '[:space:]' <"$file" || true)"
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      sleep 0.3
      kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$file"
  fi
}

echo "Stopping patrol console + viz tunnels..."
kill_pidfile "$PID_FILE"
kill_pidfile "$LAUNCHER_PID_FILE"
pkill -f 'start_remote_console\.sh' 2>/dev/null || true
if command -v pgrep >/dev/null 2>&1; then
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    envf="/proc/$pid/environ"
    if [[ -r "$envf" ]] && tr '\0' '\n' <"$envf" 2>/dev/null | grep -qx 'DIMOS_CONSOLE_ROLE=mapping'; then
      continue
    fi
    # Default / missing role = patrol console.
    kill "$pid" 2>/dev/null || true
  done < <(pgrep -f '/remote_client/console\.py' || true)
fi
kill_port "$PORT"

mapping_up=0
if [[ -f "$MAPPING_PID_FILE" ]]; then
  mpid="$(tr -d '[:space:]' <"$MAPPING_PID_FILE" || true)"
  if [[ "$mpid" =~ ^[0-9]+$ ]] && kill -0 "$mpid" 2>/dev/null; then
    mapping_up=1
  fi
fi
if [[ "$mapping_up" -eq 0 ]] && ss -ltnH 2>/dev/null | awk '{print $4}' | grep -Eq ':7782$'; then
  mapping_up=1
fi

if [[ "$mapping_up" -eq 0 ]]; then
  for port in 7779 9878 9877 3030 17779 19878 19877 13030 27779 29878 29877 23030; do
    kill_port "$port"
  done
  pkill -f 'ssh -fNT .*go2-orin' 2>/dev/null || true
else
  echo "Mapping console still up on 7782 — leaving shared SSH tunnels alone."
fi

sleep 0.5
left="$(ss -ltnH 2>/dev/null | awk '$4 ~ /:(7781)$/' || true)"
if [[ -n "$left" ]]; then
  echo "WARN: port 7781 still listening:"
  echo "$left"
  echo "Try: fuser -k 7781/tcp"
  exit 1
fi

echo "Patrol console stopped. Ports 7781 free (tunnels kept if mapping console live)."
echo "Start again:"
echo "  ./start_remote_console.sh --link iphone   # or --link wire"
echo "Mapping console: ./start_mapping_console.sh --link iphone"
