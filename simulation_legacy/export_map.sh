#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
DEVICE="${DIMOS_MAP_DEVICE:-CPU:0}"

# --export-pc2 implies --pgo in DimOS 0.0.15, which requires gtsam.
if ! "$ROOT/.venv/bin/python" -c "import gtsam" >/dev/null 2>&1; then
  echo "gtsam is required for .pc2.lcm export. On Orin: simulation_legacy/.venv/bin/python -m pip install 'gtsam==4.3.0'" >&2
  exit 2
fi
echo "Exporting with PGO (--export-pc2)..."
"$DIMOS" map global "$DB" --pgo --export-pc2 --no-gui --device "$DEVICE" --out "$RRD"

SOURCE="$ROOT/${STEM}.pc2.lcm"
if [[ -f "$SOURCE" && "$SOURCE" != "$OUT" ]]; then
  mv "$SOURCE" "$OUT"
fi
if [[ ! -f "$OUT" ]]; then
  echo "DimOS did not write $OUT (looked for $SOURCE)." >&2
  exit 1
fi
echo "Exported $OUT"
