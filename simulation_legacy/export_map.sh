#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIMOS="$ROOT/.venv/bin/dimos"
DB="${1:?Usage: export_map.sh RECORDING.db OUTPUT.pc2.lcm}"
OUT="${2:?Usage: export_map.sh RECORDING.db OUTPUT.pc2.lcm}"

if [[ ! -f "$DB" ]]; then
  echo "Recording not found: $DB" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUT")"
cd "$ROOT"
"$DIMOS" map global "$DB" --pgo --export --no-gui
SOURCE="$ROOT/$(basename "${DB%.db}").pc2.lcm"
RRD="$ROOT/$(basename "${DB%.db}").rrd"
mv "$SOURCE" "$OUT"
[[ -f "$RRD" ]] && mv "$RRD" "${OUT%.pc2.lcm}.rrd"