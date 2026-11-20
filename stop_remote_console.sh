#!/usr/bin/env bash
# Stop the remote console and local SSH tunnels started by start_remote_console.sh
# (including ones launched in the background by an agent).
set -euo pipefail

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

echo "Stopping remote console + viz tunnels..."
pkill -f '/remote_client/console.py' 2>/dev/null || true
pkill -f 'start_remote_console.sh' 2>/dev/null || true

# Local forwards used by start_remote_console.sh
for port in 7781 7779 9878 9877 3030 17779 19878 19877 13030 27779 29878 29877 23030; do
  kill_port "$port"
done

# Detached ssh -fNT tunnels to Orin (wire / iphone / guest / ap aliases)
pkill -f 'ssh -fNT .*go2-orin' 2>/dev/null || true

sleep 0.5
left="$(ss -ltnH 2>/dev/null | awk '$4 ~ /:(7781|7779|9878|9877|3030)$/' || true)"
if [[ -n "$left" ]]; then
  echo "WARN: some ports still listening:"
  echo "$left"
  echo "Try: fuser -k 7781/tcp 7779/tcp 9878/tcp 9877/tcp 3030/tcp"
  exit 1
fi

echo "Stopped. Ports 7781/7779/9878/9877/3030 are free."
echo "Start again in the foreground with:"
echo "  ./start_remote_console.sh --link iphone   # or --link wire"
