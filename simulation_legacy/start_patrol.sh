#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Pick up Orin agent .env (AES, Rerun lite, patrol blueprint overrides).
# Preserve caller-provided pose: sourcing .env must not clobber console/agent hint.
_POSE_FROM_CALLER="${DIMOS_INITIAL_POSE-__UNSET__}"
if [[ -f "$ROOT/../robot_agent/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/../robot_agent/.env"
  set +a
fi
if [[ "$_POSE_FROM_CALLER" != "__UNSET__" ]]; then
  export DIMOS_INITIAL_POSE="$_POSE_FROM_CALLER"
fi
unset _POSE_FROM_CALLER
# The ±20° refine often lands 0.7–1.0 m from the typed point. The old 0.70 m
# gate threw those away and the later in-gate sample was the one still offset.
# Yaw stays inside the refine window so a second heading cannot lock.
# Gates of the 10:00 Nov 20 lock that sat on the costmap. Yaw matches the ±20°
# seeds; a 12° gate rejected the stable fine-yaw answer at -15°.
export DIMOS_RELOC_HINT_XY_M=0.70
export DIMOS_RELOC_HINT_YAW_DEG=20
export DIMOS_RELOC_SETTLE_N=3
# 1: first lock is the typed initial pose (operator places the dog, then nudges
# in the console). 0: lidar match against the loaded map.
export DIMOS_RELOC_TRUST_HINT="${DIMOS_RELOC_TRUST_HINT:-1}"
# Loaded map stays off the Rerun view. The live cloud is matched against it
# in the background; global map and costmap remain.
export DIMOS_SHOW_LOADED_MAP=0
# shellcheck disable=SC1091
source "$ROOT/dimos_cli.sh"
DIMOS="$ROOT/.venv/bin/dimos"
ROUTE="${1:-$ROOT/patrol_route.json}"
MAP="${2:-$ROOT/maps/warehouse.pc2.lcm}"
MODE="${DIMOS_MODE:-simulation}"
# Default: WebRTC Go2 stack + lidar-window relocalization (same sensor path as mapping).
# unitree-go2-nav-3d-relocalization needs nix-built Mid360/PointLIO/MLS natives and will
# fail hard on Orin images without nix ("nix: not found"), so CC/Rerun never come up.
PATROL_BLUEPRINT="${DIMOS_PATROL_BLUEPRINT:-unitree-go2}"
RELOC_MODULE="${DIMOS_RELOC_MODULE:-lidar-window-relocalization}"
# CLI flag prefix follows DimOS module id (hyphens stripped), e.g. LidarWindowRelocalization
RELOC_PREFIX="${DIMOS_RELOC_PREFIX:-lidarwindowrelocalization}"
STATUS_FILE="${PATROL_STATUS_FILE:-$ROOT/maps/patrol-status.json}"
TRANSFORM_FILE="${PATROL_TRANSFORM_FILE:-$ROOT/maps/map-to-world.json}"
export PATROL_TRANSFORM_FILE="$TRANSFORM_FILE"
CONTROL_FILE="${PATROL_CONTROL_FILE:-$ROOT/maps/patrol-control.json}"
LOG_FILE="${PATROL_DIMOS_LOG_FILE:-$ROOT/logs/patrol-dimos.log}"
# 1 = keep DimOS/CC/Rerun up for pose checks; do not start waypoint navigation.
LOCALIZE_ONLY="${PATROL_LOCALIZE_ONLY:-0}"
DIMOS_PID=""
STOPPING=0

mkdir -p "$ROOT/logs" "$(dirname "$STATUS_FILE")"

dimos_aarch64_preload
dimos_global_args || exit $?
dimos_aes_args
dimos_viewer_args

# relocalize-once=false keeps scan-to-map ICP running for the whole patrol.
# Settle V9: first lock is a global match (hint only rejects a corridor flip).
# After that, one local ICP from the last pose is published immediately.
# A single step may move 0.6 m / 20 deg. A 90/180 deg jump is dropped.
# DimOS rejects *partial* nested RelocalizeConfig overrides — pass a full preset.
# Go2 WebRTC lidar often sits at 0.66–0.73; 0.74 starved settle for ~1 min in field logs.
# Settle N-agree + optional DIMOS_INITIAL_POSE yaw hint covers false locks.
# With pose hint, 0.72 is reachable; without hint rectangular rooms false-lock at any threshold.
RELOC_FITNESS="${DIMOS_RELOC_FITNESS:-0.72}"
RELOC_RESTARTS="${DIMOS_RELOC_RANSAC_RESTARTS:-3}"
RELOC_ONCE="${DIMOS_RELOC_ONCE:-false}"
# 0.3 s: initial attempts are still RANSAC-bound (~1 s). After lock, local ICP
# finishes well inside this interval, so the pose is corrected several times a second.
RELOC_INTERVAL="${DIMOS_RELOC_INTERVAL:-0.3}"
RELOC_MIN_FRAMES="${DIMOS_RELOC_MIN_FRAMES:-2}"
RELOC_MAX_FRAMES="${DIMOS_RELOC_MAX_FRAMES:-7}"
# 0: do not stream the full prior cloud again on every fix. The eager patch
# publishes it a few times at start, and Rerun keeps that copy as static.
REPUBLISH_MAP="${DIMOS_REPUBLISH_LOADED_MAP:-0}"
# Real robot: default start-of-map pose. Pure global ICP cannot break 180° room symmetry.
if [[ -z "${DIMOS_INITIAL_POSE:-}" && "$MODE" == "real" && "${DIMOS_RELOC_NO_DEFAULT_POSE:-0}" != "1" ]]; then
  export DIMOS_INITIAL_POSE="0,0,0"
  echo "DIMOS_INITIAL_POSE defaulted to 0,0,0 in the loaded map frame. Set the console pose or DIMOS_RELOC_NO_DEFAULT_POSE=1 to disable."
fi
RELOCALIZATION_ARGS=(
  --"${RELOC_PREFIX}.map-file" "$MAP"
  --"${RELOC_PREFIX}.republish-loaded-map" "$REPUBLISH_MAP"
  --"${RELOC_PREFIX}.relocalize-once" "$RELOC_ONCE"
  --"${RELOC_PREFIX}.reloc-interval" "$RELOC_INTERVAL"
  --"${RELOC_PREFIX}.relocalize.voxel-coarse" 0.59
  --"${RELOC_PREFIX}.relocalize.voxel-fine" 0.3
  --"${RELOC_PREFIX}.relocalize.normal-radius-factor" 1.72
  --"${RELOC_PREFIX}.relocalize.fpfh-radius-factor" 4.13
  --"${RELOC_PREFIX}.relocalize.coarse-dist-factor" 2.73
  --"${RELOC_PREFIX}.relocalize.ransac-iters" 1578291
  --"${RELOC_PREFIX}.relocalize.mutual-filter" true
  --"${RELOC_PREFIX}.relocalize.edge-length" 0.7
  --"${RELOC_PREFIX}.relocalize.icp-dist-factor" 0.55
  --"${RELOC_PREFIX}.relocalize.icp-stages" 2
  --"${RELOC_PREFIX}.relocalize.orient-normals" true
  --"${RELOC_PREFIX}.relocalize.ransac-restarts" "$RELOC_RESTARTS"
  --"${RELOC_PREFIX}.relocalize.fitness-threshold" "$RELOC_FITNESS"
  --"${RELOC_PREFIX}.relocalize.min-frames" "$RELOC_MIN_FRAMES"
  --"${RELOC_PREFIX}.relocalize.max-frames" "$RELOC_MAX_FRAMES"
  --"${RELOC_PREFIX}.relocalize.normal-max-nn" 30
  --"${RELOC_PREFIX}.relocalize.fpfh-max-nn" 100
  --"${RELOC_PREFIX}.relocalize.ransac-confidence" 0.999
  --"${RELOC_PREFIX}.relocalize.ransac-n" 3
  --"${RELOC_PREFIX}.relocalize.icp-max-iter" 200
)
# Firmware avoidance watches only the forward cone. On this dog it treats empty
# space ahead as an obstacle, zeros forward speed, and still allows reverse,
# so the dog backs into things. Patrol avoidance is the DimOS costmap.
# Escape hatch: DIMOS_ENABLE_MCU_OBSTACLE_AVOIDANCE=1.
DISABLE_MODULES=(patrolling-module wavefront-frontier-explorer)
GO2_ARGS=()
case "${DIMOS_VELOCITY_API:-1}" in
  1|true|TRUE|yes|YES) GO2_ARGS+=(--go2connection.velocity-api true) ;;
esac
case "${DIMOS_ENABLE_MCU_OBSTACLE_AVOIDANCE:-0}" in
  1|true|TRUE|yes|YES) GO2_ARGS+=(--go2connection.obstacle-avoidance true) ;;
  *) GO2_ARGS+=(--go2connection.obstacle-avoidance false) ;;
esac
if [[ "$LOCALIZE_ONLY" == "1" || "$LOCALIZE_ONLY" == "true" || "$LOCALIZE_ONLY" == "TRUE" ]]; then
  DISABLE_MODULES+=(replanning-a-star-planner)
fi
DISABLE_ARGS=()
for _mod in "${DISABLE_MODULES[@]}"; do
  DISABLE_ARGS+=(--disable "$_mod")
done
if [[ "$MODE" == "simulation" ]]; then
  RELOCALIZATION_ARGS+=(--"${RELOC_PREFIX}.min-local-points" 8000)
else
  # Go2 WebRTC lidar sweeps are much sparser than Mid-360 local maps.
  RELOCALIZATION_ARGS+=(--"${RELOC_PREFIX}.min-local-points" "${DIMOS_MIN_LOCAL_POINTS:-2000}")
fi

if [[ ! -f "$ROUTE" ]]; then
  echo "Route file not found: $ROUTE" >&2
  exit 1
fi
if [[ ! -f "$MAP" ]]; then
  echo "Map file not found: $MAP (pass an explicit .pc2.lcm path as the second argument)" >&2
  exit 1
fi
# Same recording exports both files. pc2.lcm is the localization cloud.
# map.db is the pose-graph the live costmap loads so distant waypoints are
# inside the planning grid before the dog walks there. save-map stays off.
PLAN_MAP="${MAP%.pc2.lcm}.map.db"
PGO_ARGS=()
if [[ -f "$PLAN_MAP" ]]; then
  PGO_ARGS+=(
    --pgovoxelmapper.map-file "$PLAN_MAP"
    --pgovoxelmapper.save-map false
  )
  echo "Costmap prior: $PLAN_MAP"
else
  echo "No planning map at $PLAN_MAP." >&2
  echo "Re-export the recording with --export so long-range goals are on the costmap. This run only plans inside the live scan." >&2
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

# Best-effort DimOS site-packages patches (idempotent).
if [[ -f "$ROOT/../scripts/apply_relocalize_settle_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_relocalize_settle_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/mapping/relocalization/lidar/module.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_loaded_map_eager_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_loaded_map_eager_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/mapping/relocalization/module.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_rerun_static_map_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_rerun_static_map_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/visualization/rerun/bridge.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_rerun_drop_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_rerun_drop_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/visualization/rerun/bridge.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_rerun_points_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_rerun_points_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/msgs/sensor_msgs/PointCloud2.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_live_view_track_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_live_view_track_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_rerun_track_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_rerun_track_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_one_map_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_one_map_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_transform_file_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_transform_file_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/mapping/relocalization/module.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_relocalize_hint_seed_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_relocalize_hint_seed_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/mapping/relocalization/lidar/relocalize.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_reloc_track_bound_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_reloc_track_bound_patch.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_reloc_lock_quality_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_reloc_lock_quality_patch.py" \
    >/dev/null 2>&1 || true
fi
# Lidar relocalization owns world←map. PGO's own TF rotates that same edge.
# global_map stays unpublished until that lock is written, then uses the same pose.
export DIMOS_PGO_PUBLISH_TF=0
export DIMOS_PGO_WAIT_FOR_LIDAR=1
if [[ -f "$ROOT/../scripts/apply_pgo_tf_guard.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_pgo_tf_guard.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/navigation/go2/loop_closure/module.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_pgo_wait_lock_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_pgo_wait_lock_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/navigation/go2/loop_closure/module.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_prior_costmap_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_prior_costmap_patch.py" \
    >/dev/null 2>&1 || true
fi
# Frozen Unitree odom stamps leave robot_body stuck on the first pose in Rerun.
if [[ -f "$ROOT/../scripts/apply_odom_stamp_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_odom_stamp_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/robot/unitree/go2/connection.py" \
    >/dev/null 2>&1 || true
fi
# Sport Move never enters the avoidance service. Route velocity through it.
if [[ -f "$ROOT/../scripts/apply_avoid_move_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_avoid_move_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/robot/unitree/connection.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_cmd_speed_cap_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_cmd_speed_cap_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/robot/unitree/connection.py" \
    >/dev/null 2>&1 || true
fi
if [[ -f "$ROOT/../scripts/apply_reloc_hold_still_patch.py" ]]; then
  "$ROOT/.venv/bin/python" "$ROOT/../scripts/apply_reloc_hold_still_patch.py" \
    "$ROOT/.venv/lib/python3.12/site-packages/dimos/mapping/relocalization/lidar/module.py" \
    >/dev/null 2>&1 || true
fi
# Navigation patches live in the venv. Reapply on every start so a DimOS reinstall
# cannot drop them. Idempotent: each script exits 0 when its marker is present.
for _nav_patch in \
  apply_nav_rotation_stuck_patch.py \
  apply_nav_skip_spin_patch.py \
  apply_nav_creep_turn_patch.py \
  apply_nav_wide_path_patch.py \
  apply_nav_short_path_patch.py \
  apply_nav_slow_turn_patch.py \
  apply_nav_yaw_fallback_patch.py \
  apply_nav_speed_cap_patch.py \
  apply_nav_obstacle_hold_patch.py \
  apply_path_mask_tolerance_patch.py \
  apply_nav_hold_guard_patch.py \
  apply_nav_live_cloud_patch.py \
  apply_live_costmap_fuse_patch.py \
  apply_track_fitness_height_patch.py \
  apply_nav_planner_survive_patch.py \
  apply_reloc_recover_patch.py \
  apply_nav_smooth_patch.py \
  apply_reloc_track_dog_bound_patch.py
do
  if [[ -f "$ROOT/../scripts/$_nav_patch" ]]; then
    "$ROOT/.venv/bin/python" "$ROOT/../scripts/$_nav_patch" >/dev/null 2>&1 || true
  fi
done

echo "Starting $MODE localization for map $MAP (blueprint=$PATROL_BLUEPRINT + $RELOC_MODULE)"
if [[ -n "${DIMOS_INITIAL_POSE:-}" ]]; then
  echo "Initial pose on the loaded map: $DIMOS_INITIAL_POSE (x, y meters, yaw radians)"
fi
printf '{"phase":"localizing","message":"Loading map and waiting for Command Center / localization"}\n' > "$STATUS_FILE"

EXTRA_MODULES=()
if [[ -n "$RELOC_MODULE" && "$PATROL_BLUEPRINT" != *"relocalization"* ]]; then
  EXTRA_MODULES+=("$RELOC_MODULE")
fi

"$DIMOS" "${DIMOS_GLOBAL_ARGS[@]}" run \
  "${DISABLE_ARGS[@]}" \
  "$PATROL_BLUEPRINT" \
  "${EXTRA_MODULES[@]}" \
  "${DIMOS_VIEWER_ARGS[@]}" \
  "${DIMOS_AES_ARGS[@]}" \
  "${GO2_ARGS[@]}" \
  "${PGO_ARGS[@]}" \
  "${RELOCALIZATION_ARGS[@]}" >"$LOG_FILE" 2>&1 &
DIMOS_PID=$!

echo "Waiting for visualization ports. UI: http://127.0.0.1:7779/"
viz_up=false
for _ in {1..90}; do
  if ! kill -0 "$DIMOS_PID" 2>/dev/null; then
    tail -80 "$LOG_FILE" >&2
    exit 1
  fi
  if command -v ss >/dev/null 2>&1 && ss -ltnH | awk '{print $4}' | grep -Eq ':7779$'; then
    viz_up=true
    break
  fi
  sleep 1
done
if [[ "$viz_up" != true ]]; then
  echo "Command Center port 7779 did not open within 90s. See $LOG_FILE" >&2
  printf '{"phase":"error","message":"Command Center failed to start"}\n' > "$STATUS_FILE"
  exit 1
fi
printf '{"phase":"localizing","message":"Command Center up; waiting for a valid localization match"}\n' > "$STATUS_FILE"

echo "Waiting for localization match. UI: http://127.0.0.1:7779/"
localized=false
for _ in {1..180}; do
  # Require an accepted fix (settle 3/3 → submit), not merely the first ICP candidate.
  if grep -Eq 'relocalize fix' "$LOG_FILE"; then
    localized=true
    break
  fi
  if ! kill -0 "$DIMOS_PID" 2>/dev/null; then
    tail -80 "$LOG_FILE" >&2
    exit 1
  fi
  sleep 1
done

if [[ "$LOCALIZE_ONLY" == "1" || "$LOCALIZE_ONLY" == "true" || "$LOCALIZE_ONLY" == "TRUE" ]]; then
  if [[ "$localized" == true ]]; then
    printf '{"phase":"localized","message":"Localization accepted; localize-only (no patrol navigation)"}\n' > "$STATUS_FILE"
  else
    printf '{"phase":"localizing","message":"Localize-only: CC/Rerun up; no accepted fix yet — drive slowly and watch pose"}\n' > "$STATUS_FILE"
  fi
  echo "Localize-only mode: keeping DimOS up for Command Center / Rerun. Stop from the console when done."
  # Poll stop flag — agent stop used to only set this; without polling DimOS never exits.
  while kill -0 "$DIMOS_PID" 2>/dev/null; do
    if [[ -f "$CONTROL_FILE" ]] && grep -Eq '"stop_requested"[[:space:]]*:[[:space:]]*true' "$CONTROL_FILE"; then
      echo "Stop requested; shutting down localize-only DimOS."
      break
    fi
    sleep 1
  done
  stop_runtime
  printf '{"phase":"idle","message":"Localize-only stopped"}\n' > "$STATUS_FILE"
  exit 0
fi

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
  echo "A successful match was logged but no map-to-world transform was written." >&2
  echo "Keeping DimOS up for visualization; refusing to dispatch route (set PATROL_LOCALIZE_ONLY=1 to skip this check)." >&2
  printf '{"phase":"localized","message":"Fix logged but map-to-world.json missing; navigation withheld — use CC/Rerun to inspect pose"}\n' > "$STATUS_FILE"
  wait "$DIMOS_PID" || true
  exit 0
fi

# Real robot: wait for operator to confirm Rerun overlap before any motion.
if [[ "$MODE" == "real" && "${PATROL_REQUIRE_POSE_CONFIRM:-1}" == "1" ]]; then
  echo "Localization locked — confirm pose in Rerun, then click「确认位姿并开始行走」."
  printf '{"phase":"awaiting_pose_confirm","message":"定位已锁定：请在 Rerun 看狗的位置和实时点云是否贴着周围障碍，再点「确认位姿并开始行走」"}\n' > "$STATUS_FILE"
  # Clear stale confirm from a previous run.
  if [[ -f "$CONTROL_FILE" ]]; then
    "$ROOT/.venv/bin/python" - "$CONTROL_FILE" <<'PY' || true
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
try:
    data = json.loads(p.read_text())
except Exception:
    data = {}
if not isinstance(data, dict):
    data = {}
data["pose_confirmed"] = False
p.write_text(json.dumps(data, indent=2) + "\n")
PY
  fi
  while kill -0 "$DIMOS_PID" 2>/dev/null; do
    if [[ -f "$CONTROL_FILE" ]] && grep -Eq '"stop_requested"[[:space:]]*:[[:space:]]*true' "$CONTROL_FILE"; then
      echo "Stop requested before pose confirm."
      stop_runtime
      printf '{"phase":"idle","message":"Stopped before pose confirmation"}\n' > "$STATUS_FILE"
      exit 0
    fi
    if [[ -f "$CONTROL_FILE" ]] && grep -Eq '"pose_confirmed"[[:space:]]*:[[:space:]]*true' "$CONTROL_FILE"; then
      echo "Pose confirmed by operator; starting patrol schedule."
      break
    fi
    sleep 1
  done
  if ! kill -0 "$DIMOS_PID" 2>/dev/null; then
    printf '{"phase":"error","message":"DimOS exited while waiting for pose confirmation"}\n' > "$STATUS_FILE"
    exit 1
  fi
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
