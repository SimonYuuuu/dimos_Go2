#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$ROOT/dimos_cli.sh"
DIMOS="$ROOT/.venv/bin/dimos"
NAME="${1:-warehouse}"
DB="$ROOT/maps/${NAME}.db"
MAP="$ROOT/maps/${NAME}.pc2.lcm"
MODE="${DIMOS_MODE:-simulation}"

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

if ! "$ROOT/.venv/bin/python" -c "import yaml" >/dev/null 2>&1; then
  echo "DimOS venv is missing PyYAML. On Orin run: simulation_legacy/.venv/bin/python -m pip install PyYAML" >&2
  exit 2
fi

if [[ "${DIMOS_MODE:-}" == "real" ]]; then
  echo "Checking Go2 ${ROBOT_IP:-} WebRTC/DDS before starting DimOS..."
  "$ROOT/.venv/bin/python" "$ROOT/../robot_agent/preflight_go2.py" || exit $?
fi

dimos_aarch64_preload
dimos_global_args || exit $?
dimos_aes_args
dimos_viewer_args

finish() {
  local exit_code=$?
  trap - INT TERM
  trap - EXIT
  # Export is owned by robot_agent stop_mapping (async) so SIGINT can return quickly
  # and large PGO jobs do not block the shell EXIT trap under killpg.
  if [[ -f "$DB" ]]; then
    echo "Recording stopped (exit $exit_code). DB kept at $DB; agent will run export_map.sh."
  else
    echo "No recording found at $DB; nothing to export." >&2
  fi
  return "$exit_code"
}
trap finish EXIT

cd "$ROOT"
echo "Mapping $NAME ($MODE). DimOS will StandUp the Go2, then open Command Center on port 7779."
echo "Drive from Command Center; press Ctrl+C / 结束采集 when coverage is complete."
# Keep live costmap (Command Center) + voxel map. Disable heavy planners only.
# pgo-voxel-mapper needs gtsam (installed on Orin); export_map.sh still does full offline PGO.
"$DIMOS" "${DIMOS_GLOBAL_ARGS[@]}" run \
  --disable replanning-a-star-planner \
  --disable patrolling-module \
  --disable wavefront-frontier-explorer \
  unitree-go2-memory \
  "${DIMOS_VIEWER_ARGS[@]}" \
  "${DIMOS_AES_ARGS[@]}" \
  --go2memory.db-path "$DB"
