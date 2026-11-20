#!/usr/bin/env bash
# Run DimOS CLI with Jetson-safe LD_PRELOAD (fixes Open3D libgomp TLS ImportError).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$ROOT/dimos_cli.sh"
dimos_aarch64_preload
exec "$ROOT/.venv/bin/dimos" "$@"
