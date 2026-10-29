#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS="$ROOT/.venv/bin/dimos"
ROUTE="${1:-$ROOT/patrol_route.json}"
MAP="${2:-$ROOT/maps/warehouse.pc2.lcm}"
MODE="${DIMOS_MODE:-simulation}"
GLOBAL_ARGS=()
RELOCALIZATION_ARGS=(-o "relocalizationmodule.map_file=$MAP" -o relocalizationmodule.publish_loaded_map=true)
STATUS_FILE="${PATROL_STATUS_FILE:-$ROOT/maps/patrol-status.json}"
TRANSFORM_FILE="${PATROL_TRANSFORM_FILE:-$ROOT/maps/map-to-world.json}"
CONTROL_FILE="${PATROL_CONTROL_FILE:-$ROOT/maps/patrol-control.json}"
LOG_FILE="${PATROL_DIMOS_LOG_FILE:-$ROOT/logs/patrol-dimos.log}"
DIMOS_PID=""
STOPPING=0

mkdir -p "$ROOT/logs" "$(dirname "$STATUS_FILE")"

case "$MODE" in
  simulation)
    GLOBAL_ARGS+=(--simulation mujoco)
    RELOCALIZATION_ARGS+=(-o relocalizationmodule.initial_pose=identity -o relocalizationmodule.min_local_points=8000)
    ;;
  real)
    if [[ -z "${ROBOT_IP:-}" ]]; then
      echo "Set ROBOT_IP to the Go2 address before starting a real patrol." >&2
      exit 2
    fi
    GLOBAL_ARGS+=(--robot-ip "$ROBOT_IP")
    if [[ -n "${UNITREE_AES_128_KEY:-}" ]]; then
      GLOBAL_ARGS+=(--unitree-aes-128-key "$UNITREE_AES_128_KEY")
    fi
    if [[ -n "${DIMOS_INITIAL_POSE:-}" ]]; then
      RELOCALIZATION_ARGS+=(-o "relocalizationmodule.initial_pose=$DIMOS_INITIAL_POSE")
    fi
    RELOCALIZATION_ARGS+=(-o "relocalizationmodule.min_local_points=${DIMOS_MIN_LOCAL_POINTS:-50000}")
    ;;
  *)
    echo "DIMOS_MODE must be 'simulation' or 'real'." >&2
    exit 2
    ;;
esac

if [[ ! -f "$ROUTE" ]]; then
  echo "Route file not found: $ROUTE" >&2
  exit 1
fi
if [[ ! -f "$MAP" ]]; then
  echo "Map file not found: $MAP (pass an explicit .pc2.lcm path as the second argument)" >&2
  exit 1
fi
if command -v ss >/dev/null 2>&1 && ss -ltnH | awk '{print $4}' | grep -Eq ':(7779|9877|9878|3030)$'; then
  echo "A DimOS/visualization service is already listening on 7779/9877/9878/3030. Stop that run before starting patrol." >&2
  exit 3
fi

cd "$ROOT"
rm -f "$TRANSFORM_FILE"
stop_runtime() {
  if [[ "$STOPPING" == "1" ]]; then
    return
  fi
  STOPPING=1
  "$DIMOS" stop >/dev/null 2>&1 || true
  if [[ -n "$DIMOS_PID" ]]; then
    for _ in {1..30}; do
      if ! kill -0 "$DIMOS_PID" 2>/dev/null; then
        wait "$DIMOS_PID" 2>/dev/null || true
        DIMOS_PID=""
        return
      fi
      sleep 1
    done
    kill -TERM "$DIMOS_PID" 2>/dev/null || true
    for _ in {1..5}; do
      if ! kill -0 "$DIMOS_PID" 2>/dev/null; then
        wait "$DIMOS_PID" 2>/dev/null || true
        DIMOS_PID=""
        return
      fi
      sleep 1
    done
    kill -KILL "$DIMOS_PID" 2>/dev/null || true
    wait "$DIMOS_PID" 2>/dev/null || true
    DIMOS_PID=""
  fi
}
cleanup() {
  local exit_code=$?
  trap - INT TERM EXIT
  stop_runtime
  if [[ "$exit_code" != "0" ]]; then
    local phase="error"
    [[ "$exit_code" == "130" || "$exit_code" == "143" ]] && phase="stopped"
    printf '{"phase":"%s","message":"Patrol launcher exited","exit_code":%s}\n' "$phase" "$exit_code" > "$STATUS_FILE"
  fi
  exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting $MODE localization for map $MAP"
printf '{"phase":"localizing","message":"Loading map and waiting for a valid localization match"}\n' > "$STATUS_FILE"
"$DIMOS" "${GLOBAL_ARGS[@]}" --rerun-open web run unitree-go2-relocalization \
  "${RELOCALIZATION_ARGS[@]}" -o "relocalizationmodule.transform_file=$TRANSFORM_FILE" \
  --disable patrolling-module --disable wavefront-frontier-explorer >"$LOG_FILE" 2>&1 &
DIMOS_PID=$!

echo "Waiting for map and localization. UI: http://127.0.0.1:7779/"
localized=false
for _ in {1..180}; do
  if grep -Eq 'relocalize: fitness=' "$LOG_FILE"; then
    localized=true
    break
  fi
  if ! kill -0 "$DIMOS_PID" 2>/dev/null; then
    tail -80 "$LOG_FILE" >&2
    exit 1
  fi
  sleep 1
done
if [[ "$localized" != true ]]; then
  echo "Localization did not succeed within 180s. See $LOG_FILE; refusing to start movement." >&2
  printf '{"phase":"error","message":"Localization timed out; patrol not started"}\n' > "$STATUS_FILE"
  exit 1
fi

transform_ready=false
for _ in {1..10}; do
  if [[ -s "$TRANSFORM_FILE" ]]; then
    transform_ready=true
    break
  fi
  sleep 1
done
if [[ "$transform_ready" != true ]]; then
  echo "A successful match was logged but no map-to-world transform was written. Refusing to dispatch route." >&2
  printf '{"phase":"error","message":"Localization transform file missing; patrol not started"}\n' > "$STATUS_FILE"
  exit 1
fi

printf '{"phase":"localized","message":"Localization accepted; starting patrol schedule"}\n' > "$STATUS_FILE"
echo "Localization confirmed. Opening patrol task. UI: http://127.0.0.1:7781/"
PATROL_ARGS=()
case "${PATROL_START_MODE:-}" in
  immediate)
    PATROL_ARGS+=(--immediate)
    ;;
  daily)
    PATROL_ARGS+=(--start-at "${PATROL_START_AT:?Daily execution requires PATROL_START_AT}")
    ;;
esac
if [[ -n "${PATROL_MAX_DURATION_S:-}" ]]; then
  PATROL_ARGS+=(--max-duration-s "$PATROL_MAX_DURATION_S")
fi
if [[ -n "${PATROL_REPEAT_INTERVAL_S:-}" ]]; then
  PATROL_ARGS+=(--repeat-interval-s "$PATROL_REPEAT_INTERVAL_S")
fi
if [[ "${PATROL_REPEAT_ENABLED:-0}" == "1" ]]; then
  PATROL_ARGS+=(--repeat-enabled)
else
  PATROL_ARGS+=(--single-run)
fi
"$ROOT/.venv/bin/python" "$ROOT/waypoint_patrol.py" --auto --route "$ROUTE" \
  --status-file "$STATUS_FILE" --transform-file "$TRANSFORM_FILE" \
  --control-file "$CONTROL_FILE" "${PATROL_ARGS[@]}"