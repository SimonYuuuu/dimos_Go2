#!/usr/bin/env bash
# Run from the operator PC. Injects a correct UTC epoch into the Orin fetch
# script so SSL/login work even when the Jetson clock is stuck in 2022.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 1 ]]; then
  echo "Usage: $0 --email YOUR_UNITREE_APP_EMAIL [--region cn|global] [--sn SN]" >&2
  exit 2
fi
EPOCH="$(date -u +%s)"
rsync -az "$ROOT/scripts/fetch_go2_aes_key.py" go2-orin:/home/unitree/dimos_Go2/scripts/fetch_go2_aes_key.py
exec ssh -t go2-orin \
  "CORRECT_UTC_EPOCH=${EPOCH} /home/unitree/dimos_Go2/simulation_legacy/.venv/bin/python /home/unitree/dimos_Go2/scripts/fetch_go2_aes_key.py --epoch ${EPOCH} $*"
