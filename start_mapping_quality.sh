#!/usr/bin/env bash
# Orin (repo root, same level as stop_dimos.sh): slow teleop mapping for
# localization-quality point clouds. Caps Command Center speed, then runs
# unitree-go2-memory under simulation_legacy/.
#
# Usual path: host ./start_mapping_console.sh → UI「启动慢速建图」→ agent runs this.
# Direct:     DIMOS_MODE=real ./start_mapping_quality.sh <map_name>
# Stop:       ./simulation_legacy/stop_mapping.sh  (or agent mapping/stop for export)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$REPO/simulation_legacy"
if [[ -f "$REPO/robot_agent/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$REPO/robot_agent/.env"
  set +a
fi
# shellcheck disable=SC1091
source "$SIM/dimos_cli.sh"
DIMOS="$SIM/.venv/bin/dimos"
NAME="${1:-warehouse}"
DB="$SIM/maps/${NAME}.db"
MODE="${DIMOS_MODE:-simulation}"
SITE="$SIM/.venv/lib/python3.12/site-packages"
SCRIPTS="$REPO/scripts"
MAP_WALK_MPS="${DIMOS_MAP_WALK_MPS:-0.25}"

if [[ ! "$NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "Map name must contain only letters, digits, underscores or hyphens." >&2
  exit 2
fi

mkdir -p "$SIM/maps" "$SIM/logs"

if command -v ss >/dev/null 2>&1 && ss -ltnH | awk '{print $4}' | grep -Eq ':(7779|9877|9878|3030)$'; then
  echo "A DimOS/visualization service is already listening on 7779/9877/9878/3030. Stop that run before starting mapping." >&2
  exit 3
fi

if ! "$SIM/.venv/bin/python" -c "import yaml" >/dev/null 2>&1; then
  echo "DimOS venv is missing PyYAML. On Orin run: simulation_legacy/.venv/bin/python -m pip install PyYAML" >&2
  exit 2
fi

if [[ "${DIMOS_MODE:-}" == "real" ]]; then
  echo "Checking Go2 ${ROBOT_IP:-} WebRTC/DDS before starting DimOS..."
  "$SIM/.venv/bin/python" "$REPO/robot_agent/preflight_go2.py" || exit $?
fi

if [[ -f "$SCRIPTS/apply_cmd_speed_cap_patch.py" ]]; then
  "$SIM/.venv/bin/python" "$SCRIPTS/apply_cmd_speed_cap_patch.py" \
    "$SITE/dimos/robot/unitree/connection.py" >/dev/null 2>&1 || true
fi
if [[ -f "$SCRIPTS/apply_walk_speed_keepout_patch.py" ]]; then
  "$SIM/.venv/bin/python" "$SCRIPTS/apply_walk_speed_keepout_patch.py" >/dev/null 2>&1 || true
fi
# Keyboard over WiFi: hold the last command instead of the 0.2 s deadman; wait for a full stand.
for p in apply_teleop_hold_patch.py apply_cc_keyboard_patch.py; do
  if [[ -f "$SCRIPTS/$p" ]]; then
    "$SIM/.venv/bin/python" "$SCRIPTS/$p" >&2 || echo "WARN: $p failed; keyboard keeps stock behaviour." >&2
  fi
done
export DIMOS_MOVE_HOLD_S="${DIMOS_MOVE_HOLD_S:-0.6}"
# MCF Move only walks properly in the FreeWalk FSM (BalanceStand stays at gait 0).
export DIMOS_FREEWALK="${DIMOS_FREEWALK:-1}"

SPEED_FILE="${PATROL_SPEED_FILE:-$REPO/robot_data/patrol-speed.json}"
mkdir -p "$(dirname "$SPEED_FILE")"
printf '{"mps":%s,"locked":true,"mode":"map_quality"}\n' "$MAP_WALK_MPS" >"$SPEED_FILE"
export PATROL_SPEED_FILE="$SPEED_FILE"

# Slow walk only. Export uses the same PGO defaults as remote console
# (voxel 0.03 / pgo-tol 0.08 / no full-pgo) so「结束并导出」is not slower.
export DIMOS_MAP_QUALITY="${DIMOS_MAP_QUALITY:-0}"
export DIMOS_MAP_VOXEL="${DIMOS_MAP_VOXEL:-0.03}"
export DIMOS_MAP_PGO_TOL="${DIMOS_MAP_PGO_TOL:-0.08}"
export DIMOS_MAP_DENOISE="${DIMOS_MAP_DENOISE:-1}"
export DIMOS_MAP_FULL_PGO="${DIMOS_MAP_FULL_PGO:-0}"

export DIMOS_RERUN_TRACK=0
dimos_aarch64_preload
dimos_global_args || exit $?
dimos_aes_args
dimos_viewer_args

finish() {
  local exit_code=$?
  trap - INT TERM
  trap - EXIT
  if [[ -f "$DB" ]]; then
    echo "Quality mapping stopped (exit $exit_code). DB at $DB; agent will export .pc2.lcm + .map.db."
  else
    echo "No recording found at $DB; nothing to export." >&2
  fi
  return "$exit_code"
}
trap finish EXIT

# OA off. Keyboard goes out as Sport Move (velocity-api), not a virtual stick.
# The stick path is dropped when the dog logs "Robot Connection Mode: 4G".
GO2_ARGS=(--go2connection.obstacle-avoidance false)
case "${DIMOS_ENABLE_MCU_OBSTACLE_AVOIDANCE:-0}" in
  1|true|TRUE|yes|YES) GO2_ARGS=(--go2connection.obstacle-avoidance true) ;;
esac
case "${DIMOS_VELOCITY_API:-1}" in
  0|false|FALSE|no|NO) ;;
  *) GO2_ARGS+=(--go2connection.velocity-api true) ;;
esac

# Always append to robot_data/mapping.log so the console can show live status
# even if this script was started outside the agent.
LOG="${MAPPING_LOG_FILE:-$REPO/robot_data/mapping.log}"
mkdir -p "$(dirname "$LOG")"
if [[ -z "${MAPPING_LOG_ATTACHED:-}" ]]; then
  export MAPPING_LOG_ATTACHED=1
  exec >>"$LOG" 2>&1
fi

cd "$SIM"
echo "==== quality mapping start name=$NAME mode=$MODE mps=${MAP_WALK_MPS} $(date '+%F %T') ===="
echo "Quality mapping $NAME ($MODE). Teleop capped at ${MAP_WALK_MPS} m/s. MCU OA off."
echo "Drive slowly from Command Center; cover walls twice if possible; then 结束采集."
echo "DB=$DB LOG=$LOG"

# Heartbeat so the console log keeps moving even when DimOS is quiet.
heartbeat() {
  while true; do
    echo "[heartbeat] $(date '+%H:%M:%S') mapping=$NAME db=$(basename "$DB") oa=off"
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
