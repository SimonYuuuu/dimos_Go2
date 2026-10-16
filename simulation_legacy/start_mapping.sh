#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS="$ROOT/.venv/bin/dimos"
NAME="${1:-warehouse}"
DB="$ROOT/maps/${NAME}.db"
MAP="$ROOT/maps/${NAME}.pc2.lcm"
MODE="${DIMOS_MODE:-simulation}"
GLOBAL_ARGS=()

if [[ ! "$NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "Map name must contain only letters, digits, underscores or hyphens." >&2
  exit 2
fi

mkdir -p "$ROOT/maps"
mkdir -p "$ROOT/logs"

if command -v ss >/dev/null 2>&1 && ss -ltnH | awk '{print $4}' | grep -Eq ':(7779|9877|9878|3030)$'; then
  echo "A DimOS/visualization service is already listening on 7779/9877/9878/3030. Stop that run before starting mapping." >&2
  exit 3
fi

case "$MODE" in
  simulation)
    GLOBAL_ARGS+=(--simulation mujoco)
    ;;
  real)
    if [[ -z "${ROBOT_IP:-}" ]]; then
      echo "Set ROBOT_IP to the Go2 address before starting real mapping." >&2
      exit 2
    fi
    GLOBAL_ARGS+=(--robot-ip "$ROBOT_IP")
    if [[ -n "${UNITREE_AES_128_KEY:-}" ]]; then
      GLOBAL_ARGS+=(--unitree-aes-128-key "$UNITREE_AES_128_KEY")
    fi
    ;;
  *)
    echo "DIMOS_MODE must be 'simulation' or 'real'." >&2
    exit 2
    ;;
esac

finish() {
  local exit_code=$?
  trap - INT TERM
  trap - EXIT
  if [[ ! -f "$DB" ]]; then
    echo "No recording found at $DB; skipping export." >&2
    return "$exit_code"
  fi
  if [[ "$exit_code" != "0" && "$exit_code" != "130" && "$exit_code" != "143" ]]; then
    echo "Mapping process exited with code $exit_code; preserving the DB without automatic export." >&2
    return "$exit_code"
  fi
  echo
  echo "Recording stopped. Building optimized map..."
  "$ROOT/export_map.sh" "$DB" "$MAP"
  echo "Map saved: $MAP"
  return "$exit_code"
}
trap finish EXIT

cd "$ROOT"
echo "Mapping $NAME ($MODE). Drive from http://127.0.0.1:7779/; press Ctrl+C when coverage is complete."
"$DIMOS" "${GLOBAL_ARGS[@]}" --rerun-open web run unitree-go2-memory \
  --disable cost-mapper \
  --disable replanning-a-star-planner \
  --disable patrolling-module \
  --disable wavefront-frontier-explorer \
  -o "go2memory.db_path=$DB"