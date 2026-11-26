#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Pick up Orin agent .env so console "导出" uses the same localization-ready defaults.
if [[ -f "$ROOT/../robot_agent/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/../robot_agent/.env"
  set +a
fi
# shellcheck disable=SC1091
source "$ROOT/dimos_cli.sh"
dimos_aarch64_preload
DIMOS="$ROOT/.venv/bin/dimos"
DB="${1:?Usage: export_map.sh RECORDING.db OUTPUT.pc2.lcm}"
OUT="${2:?Usage: export_map.sh RECORDING.db OUTPUT.pc2.lcm}"

if [[ ! -f "$DB" ]]; then
  echo "Recording not found: $DB" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUT")"
cd "$ROOT"
STEM="$(basename "${DB%.db}")"
RRD="${OUT%.pc2.lcm}.rrd"
# libgomp is already on the Orin. The Jetson error is static TLS load order,
# fixed by LD_PRELOAD in dimos_aarch64_preload, not by reinstalling the library.
# Official map export uses CUDA:0. A leftover DIMOS_MAP_DEVICE=CPU:0 must not
# keep the rebuild on CPU when this Open3D can see the GPU.
# Set DIMOS_MAP_FORCE_CPU=1 to force CPU anyway.
if [[ "${DIMOS_MAP_FORCE_CPU:-0}" == "1" ]]; then
  DEVICE="CPU:0"
else
  DEVICE="$("$ROOT/.venv/bin/python" -c "
import open3d as o3d
cuda = o3d.core.cuda.is_available() and o3d.core.cuda.device_count() > 0
print('CUDA:0' if cuda else 'CPU:0')
" 2>/dev/null || echo "${DIMOS_MAP_DEVICE:-CPU:0}")"
fi
if [[ "$DEVICE" != "CUDA:0" ]]; then
  echo "Open3D CUDA is not available in this venv; exporting on ${DEVICE}." >&2
  echo "Installing libgomp again will not enable the GPU. The wheel has no CUDA device." >&2
else
  echo "Open3D CUDA is available; exporting on CUDA:0."
fi
# Defaults tuned for Mid-360 localization: denser keyframes + finer voxels + denoise.
# Previous 0.05 / 0.3 kept ~10% of frames and produced ghosted walls.
VOXEL="${DIMOS_MAP_VOXEL:-0.03}"
PGO_TOL="${DIMOS_MAP_PGO_TOL:-0.08}"
EXTRA_MAP_ARGS=()
case "${DIMOS_MAP_FULL_PGO:-0}" in
  1|true|TRUE|yes|YES) EXTRA_MAP_ARGS+=(--full-pgo) ;;
esac
case "${DIMOS_MAP_DENOISE:-1}" in
  1|true|TRUE|yes|YES) EXTRA_MAP_ARGS+=(--denoise) ;;
esac

# --export-pc2 implies --pgo in DimOS 0.0.15, which requires gtsam.
if ! "$ROOT/.venv/bin/python" -c "import gtsam" >/dev/null 2>&1; then
  echo "gtsam is required for .pc2.lcm export. On Orin: simulation_legacy/.venv/bin/python -m pip install 'gtsam==4.3.0'" >&2
  exit 2
fi
# Open3D on Jetson needs libgomp preload (dimos_aarch64_preload above). Do not call bare `dimos`.
echo "Exporting localization cloud and planning graph (voxel=${VOXEL}, pgo-tol=${PGO_TOL}, device=${DEVICE}, extras=${EXTRA_MAP_ARGS[*]:-none})..."
REPO_ROOT="$(cd "$ROOT/.." && pwd)"
"$ROOT/.venv/bin/python" "$REPO_ROOT/scripts/apply_map_skip_raw_patch.py" >&2 || true
MAP_ARGS=(--no-gui --device "$DEVICE" --voxel "$VOXEL" --pgo-tol "$PGO_TOL")
if [[ "${DIMOS_MAP_SERIAL:-0}" == "1" ]]; then
  "$DIMOS" map global "$DB" --pgo --export --export-pc2 "${MAP_ARGS[@]}" \
    "${EXTRA_MAP_ARGS[@]}" \
    --out "$RRD"
else
  # The cloud (.pc2.lcm) and the pose graph (.map.db) do not depend on each other: two processes.
  # The raw preview map is not needed by either (.rrd is only a preview).
  export DIMOS_MAP_SKIP_RAW="${DIMOS_MAP_SKIP_RAW:-1}"
  GRAPH_RRD="$(mktemp --suffix=.rrd)"
  "$DIMOS" map global "$DB" --export "${MAP_ARGS[@]}" --out "$GRAPH_RRD" \
    > >(sed -u 's/^/[map.db] /') 2>&1 &
  GRAPH_PID=$!
  rc=0
  "$DIMOS" map global "$DB" --pgo --export-pc2 "${MAP_ARGS[@]}" \
    "${EXTRA_MAP_ARGS[@]}" \
    --out "$RRD" || rc=$?
  wait "$GRAPH_PID" || rc=$?
  rm -f "$GRAPH_RRD"
  [[ "$rc" -eq 0 ]] || exit "$rc"
fi

SOURCE="$ROOT/${STEM}.pc2.lcm"
if [[ -f "$SOURCE" && "$SOURCE" != "$OUT" ]]; then
  mv -f "$SOURCE" "$OUT"
fi
if [[ ! -f "$OUT" ]]; then
  echo "DimOS did not write $OUT (looked for $SOURCE)." >&2
  exit 1
fi
# dimos writes ./<recording>.map.db in the working directory (unitree-go2 --map-file).
MAPDB_SRC="$ROOT/${STEM}.map.db"
MAPDB_OUT="${OUT%.pc2.lcm}.map.db"
if [[ -f "$MAPDB_SRC" && "$MAPDB_SRC" != "$MAPDB_OUT" ]]; then
  mv -f "$MAPDB_SRC" "$MAPDB_OUT"
fi
if [[ ! -f "$MAPDB_OUT" ]]; then
  echo "DimOS did not write $MAPDB_OUT (looked for $MAPDB_SRC)." >&2
  echo "Long-range patrol planning needs this pose graph next to the point cloud." >&2
  exit 1
fi
echo "Exported $OUT"
echo "Exported $MAPDB_OUT"
