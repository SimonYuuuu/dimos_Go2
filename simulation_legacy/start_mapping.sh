#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Pick up Orin agent .env (e.g. DIMOS_RERUN_LITE) even if agent was started earlier.
if [[ -f "$ROOT/../robot_agent/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/../robot_agent/.env"
  set +a
fi
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

# Mapping still wants the accumulating cloud and the camera. Patrol leaves this on.
export DIMOS_RERUN_TRACK=0
dimos_aarch64_preload
dimos_global_args || exit $?
dimos_aes_args
dimos_viewer_args

# Command Center only: skip the Rerun bridge/viewer (set DIMOS_MAP_RERUN=1 to bring it back).
if [[ "${DIMOS_MAP_RERUN:-0}" != "1" ]]; then
  _g=()
  for _a in "${DIMOS_GLOBAL_ARGS[@]}"; do
    [[ "$_a" == rerun ]] && _a=none
    _g+=("$_a")
  done
  DIMOS_GLOBAL_ARGS=("${_g[@]}")
  DIMOS_VIEWER_ARGS=(websocket-vis-module --websocketvismodule.open-browser false)
fi

# Keyboard speed: the Orin clamps Move to the walk-speed file (0.5 m/s here); the page sends 0.5.
REPO_ROOT="$(cd "$ROOT/.." && pwd)"
SITE="$ROOT/.venv/lib/python3.12/site-packages"
SPEED_FILE="$REPO_ROOT/robot_data/mapping-speed.json"
mkdir -p "$(dirname "$SPEED_FILE")"
printf '{"mps":%s,"locked":true,"mode":"mapping"}\n' "${DIMOS_MAP_WALK_MPS:-0.5}" >"$SPEED_FILE"
export PATROL_SPEED_FILE="$SPEED_FILE"
[[ -f "$REPO_ROOT/scripts/apply_cmd_speed_cap_patch.py" ]] &&
  "$ROOT/.venv/bin/python" "$REPO_ROOT/scripts/apply_cmd_speed_cap_patch.py" \
    "$SITE/dimos/robot/unitree/connection.py" >/dev/null 2>&1 || true
[[ -f "$REPO_ROOT/scripts/apply_walk_speed_keepout_patch.py" ]] &&
  "$ROOT/.venv/bin/python" "$REPO_ROOT/scripts/apply_walk_speed_keepout_patch.py" >/dev/null 2>&1 || true
[[ -f "$REPO_ROOT/scripts/apply_cc_keyboard_patch.py" ]] &&
  "$ROOT/.venv/bin/python" "$REPO_ROOT/scripts/apply_cc_keyboard_patch.py" >&2 || true
# Teleop: re-send the last command at 25 Hz (a late WiFi packet no longer trips the 0.2 s deadman),
# stand fully before the gait switch, and log commanded vs actual velocity/gait every 5 s.
[[ -f "$REPO_ROOT/scripts/apply_teleop_hold_patch.py" ]] &&
  "$ROOT/.venv/bin/python" "$REPO_ROOT/scripts/apply_teleop_hold_patch.py" >&2 || true
export DIMOS_MOVE_HOLD_S="${DIMOS_MOVE_HOLD_S:-0.6}"
# MCF Move is served by the FreeWalk FSM; BalanceStand leaves the dog at gait 0 (set 0 to disable).
export DIMOS_FREEWALK="${DIMOS_FREEWALK:-1}"

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

# OA off + Sport Move. Virtual stick is ignored when the dog reports 4G.
GO2_ARGS=(--go2connection.obstacle-avoidance false)
case "${DIMOS_ENABLE_MCU_OBSTACLE_AVOIDANCE:-0}" in
  1|true|TRUE|yes|YES) GO2_ARGS=(--go2connection.obstacle-avoidance true) ;;
esac
case "${DIMOS_VELOCITY_API:-1}" in
  0|false|FALSE|no|NO) ;;
  *) GO2_ARGS+=(--go2connection.velocity-api true) ;;
esac
# Mapping needs lidar only; the camera stream loads the same event loop that sends Move.
case "${DIMOS_MAP_CAMERA:-0}" in
  1|true|TRUE|yes|YES) ;;
  *) GO2_ARGS+=(--go2connection.camera false) ;;
esac

REPO="$(cd "$ROOT/.." && pwd)"
LOG="${MAPPING_LOG_FILE:-$REPO/robot_data/mapping.log}"
mkdir -p "$(dirname "$LOG")"
if [[ -z "${MAPPING_LOG_ATTACHED:-}" ]]; then
  export MAPPING_LOG_ATTACHED=1
  exec >>"$LOG" 2>&1
fi

cd "$ROOT"
echo "==== mapping start name=$NAME mode=$MODE $(date '+%F %T') ===="
echo "Mapping $NAME ($MODE). DimOS will StandUp the Go2, then open Command Center on port 7779."
echo "Drive from Command Center; press Ctrl+C / 结束采集 when coverage is complete. MCU OA off."
echo "DB=$DB LOG=$LOG"
# Keep live costmap (Command Center) + voxel map. Disable heavy planners only.
# pgo-voxel-mapper needs gtsam (installed on Orin); export_map.sh still does full offline PGO.
heartbeat() {
  while true; do
    echo "[heartbeat] $(date '+%H:%M:%S') mapping=$NAME db=$(basename "$DB")"
    sleep 5
  done
}
heartbeat &
HB_PID=$!
kill_hb() { kill "$HB_PID" 2>/dev/null || true; }
trap 'kill_hb; finish' EXIT
trap 'kill_hb; exit 130' INT
trap 'kill_hb; exit 143' TERM

"$DIMOS" "${DIMOS_GLOBAL_ARGS[@]}" run \
  --disable replanning-a-star-planner \
  --disable patrolling-module \
  --disable wavefront-frontier-explorer \
  unitree-go2-memory \
  "${DIMOS_VIEWER_ARGS[@]}" \
  "${DIMOS_AES_ARGS[@]}" \
  "${GO2_ARGS[@]}" \
  --go2memory.db-path "$DB"
kill_hb
