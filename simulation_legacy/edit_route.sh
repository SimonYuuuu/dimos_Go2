#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP="${1:-$ROOT/maps/warehouse.pc2.lcm}"
ROUTE="${2:-$ROOT/patrol_route.json}"

if [[ ! -f "$MAP" ]]; then
  echo "Map file not found: $MAP (pass an explicit .pc2.lcm path as the first argument)" >&2
  exit 1
fi

exec "$ROOT/.venv/bin/python" "$ROOT/edit_route.py" --map "$MAP" --out "$ROUTE"