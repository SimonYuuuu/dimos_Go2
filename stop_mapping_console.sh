#!/usr/bin/env bash
# Stop the host slow-mapping console started by start_mapping_console.sh.
# Does not stop the patrol console (7781) or the Orin robot agent.
# Viz tunnels are freed only when no patrol console is still running.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="$ROOT/remote_client/data"
PID_FILE="$DATA/mapping_console.pid"
LAUNCHER_PID_FILE="$DATA/mapping_console_launcher.pid"
PATROL_PID_FILE="$DATA/patrol_console.pid"
PORT="${REMOTE_CONSOLE_PORT:-7782}"

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

echo "Stopping mapping console (port ${PORT})..."
kill_pidfile "$PID_FILE"
kill_pidfile "$LAUNCHER_PID_FILE"
pkill -f 'start_mapping_console\.sh' 2>/dev/null || true
# Only kill console.py instances that advertise mapping role via env in /proc.
if command -v pgrep >/dev/null 2>&1; then
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    if tr '\0' '\n' <"/proc/$pid/environ" 2>/dev/null | grep -qx 'DIMOS_CONSOLE_ROLE=mapping'; then
      kill "$pid" 2>/dev/null || true
    fi
  done < <(pgrep -f '/remote_client/console\.py' || true)
fi
kill_port "$PORT"
sleep 0.4

patrol_up=0
if [[ -f "$PATROL_PID_FILE" ]]; then
  ppid="$(tr -d '[:space:]' <"$PATROL_PID_FILE" || true)"
  if [[ "$ppid" =~ ^[0-9]+$ ]] && kill -0 "$ppid" 2>/dev/null; then
    patrol_up=1
  fi
fi
if [[ "$patrol_up" -eq 0 ]] && ss -ltnH 2>/dev/null | awk '{print $4}' | grep -Eq ':7781$'; then
  patrol_up=1
fi

if [[ "$patrol_up" -eq 0 ]]; then
  echo "No patrol console detected; freeing viz tunnels 7779/9878/9877/3030..."
  for port in 7779 9878 9877 3030 17779 19878 19877 13030 27779 29878 29877 23030; do
    kill_port "$port"
  done
  pkill -f 'ssh -fNT .*go2-orin' 2>/dev/null || true
else
  echo "Patrol console still up — leaving shared SSH tunnels alone."
fi

left="$(ss -ltnH 2>/dev/null | awk -v p=":${PORT}" '$4 ~ p"$"' || true)"
if [[ -n "$left" ]]; then
  echo "WARN: port ${PORT} still listening:"
  echo "$left"
  echo "Try: fuser -k ${PORT}/tcp"
  exit 1
fi

echo "Mapping console stopped (host UI only)."
echo "NOTE: This does NOT stop Orin DimOS. If the dog/viz still runs:"
echo "  ./stop_orin_mapping.sh --link iphone"
echo "If start_mapping_console says health failed:"
echo "  ./ensure_orin_agent.sh --link iphone"
echo "Restart UI: ./start_mapping_console.sh --link iphone"
