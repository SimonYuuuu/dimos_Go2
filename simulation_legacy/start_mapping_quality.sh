#!/usr/bin/env bash
# Compatibility wrapper — canonical script lives next to stop_dimos.sh:
#   ../start_mapping_quality.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/start_mapping_quality.sh" "$@"
