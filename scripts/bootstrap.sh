#!/usr/bin/env bash
# One-shot setup after git clone/pull so host or Orin can run with local .env + venvs.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ROLE=""
FORCE=0
ORIN_HOST=""
SSH_TARGET=""
ROBOT_IP=""
TOKEN=""
PULL_TOKEN_FROM=""
SKIP_VENV=0
INSTALL_DIMOS=0

usage() {
  cat <<'EOF'
Usage: ./scripts/bootstrap.sh --role host|orin|both [options]

Options:
  --role host|orin|both   Required. host=operator PC, orin=robot, both=single machine / first setup
  --force                 Regenerate/overwrite .env values from examples
  --orin-host IP          Operator PC: Orin address (default 192.168.123.18)
  --ssh-target NAME       Operator PC: SSH Host alias (default go2-orin)
  --robot-ip IP           Orin: Go2 MCU IP (default 192.168.123.161)
  --token TOKEN           Shared ROBOT_AGENT_TOKEN
  --pull-token-from HOST  Operator PC: copy token from Orin over SSH
  --install-dimos         Orin: pip-install DimOS into simulation_legacy/.venv if missing
  --skip-venv             Only write env files / dirs, skip Python venvs
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --role) ROLE="${2:-}"; shift 2 ;;
    --force) FORCE=1; shift ;;
    --orin-host) ORIN_HOST="${2:-}"; shift 2 ;;
    --ssh-target) SSH_TARGET="${2:-}"; shift 2 ;;
    --robot-ip) ROBOT_IP="${2:-}"; shift 2 ;;
    --token) TOKEN="${2:-}"; shift 2 ;;
    --pull-token-from) PULL_TOKEN_FROM="${2:-}"; shift 2 ;;
    --install-dimos) INSTALL_DIMOS=1; shift ;;
    --skip-venv) SKIP_VENV=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$ROLE" ]]; then
  if [[ "$(uname -m)" == "aarch64" ]]; then
    ROLE="orin"
    echo "No --role given; detected aarch64 → using --role orin"
  else
    ROLE="host"
    echo "No --role given; using --role host"
  fi
fi

case "$ROLE" in host|orin|both) ;; *) echo "--role must be host|orin|both" >&2; exit 2 ;; esac

mkdir -p \
  "$ROOT/robot_data" \
  "$ROOT/logs" \
  "$ROOT/simulation_legacy/maps" \
  "$ROOT/simulation_legacy/logs" \
  "$ROOT/remote_client/data/maps"

ENV_ARGS=(--role "$ROLE")
[[ "$FORCE" -eq 1 ]] && ENV_ARGS+=(--force)
[[ -n "$TOKEN" ]] && ENV_ARGS+=(--token "$TOKEN")
[[ -n "$PULL_TOKEN_FROM" ]] && ENV_ARGS+=(--pull-token-from "$PULL_TOKEN_FROM")
[[ -n "$ORIN_HOST" ]] && ENV_ARGS+=(--orin-host "$ORIN_HOST")
[[ -n "$SSH_TARGET" ]] && ENV_ARGS+=(--ssh-target "$SSH_TARGET")
[[ -n "$ROBOT_IP" ]] && ENV_ARGS+=(--robot-ip "$ROBOT_IP")

python3 "$ROOT/scripts/write_local_env.py" "${ENV_ARGS[@]}"

# shellcheck disable=SC1091
source "$ROOT/scripts/link_profile.sh"
if [[ "$ROLE" == "host" || "$ROLE" == "both" ]]; then
  dimos_seed_link_profiles
  dimos_ensure_link_ssh_aliases
fi

ensure_ssh_alias() {
  local target="${1:-go2-orin}"
  local host="${2:-192.168.123.18}"
  local cfg="${HOME}/.ssh/config"
  mkdir -p "${HOME}/.ssh"
  chmod 700 "${HOME}/.ssh" 2>/dev/null || true
  if [[ -f "$cfg" ]] && grep -Eq "^[[:space:]]*Host[[:space:]]+${target}([[:space:]]|$)" "$cfg"; then
    echo "SSH Host ${target} already present in ~/.ssh/config"
    return 0
  fi
  cat >>"$cfg" <<EOF

Host ${target}
  HostName ${host}
  User unitree
  StrictHostKeyChecking accept-new
EOF
  chmod 600 "$cfg" 2>/dev/null || true
  echo "Appended SSH Host ${target} → ${host} (User unitree) to ~/.ssh/config"
}

setup_host_venv() {
  local py="$ROOT/remote_client/.venv/bin/python"
  if [[ ! -x "$py" ]]; then
    python3 -m venv "$ROOT/remote_client/.venv"
  fi
  "$ROOT/remote_client/.venv/bin/python" -m pip install -q --upgrade pip
  "$ROOT/remote_client/.venv/bin/python" -m pip install -q -r "$ROOT/remote_client/requirements.txt"
  echo "remote_client venv ready: $py"
}

setup_orin_venv() {
  bash "$ROOT/scripts/ensure_turbojpeg.sh"
  local venv="$ROOT/simulation_legacy/.venv"
  local py="$venv/bin/python"
  if [[ ! -x "$py" ]]; then
    if [[ "$INSTALL_DIMOS" -ne 1 ]]; then
      cat >&2 <<'EOF'
Missing simulation_legacy/.venv (DimOS).
Re-run with:  ./scripts/bootstrap.sh --role orin --install-dimos
Or create it manually (needs network / SOCKS proxy on Orin):
  cd simulation_legacy && python3 -m venv .venv
  .venv/bin/python -m pip install -U pip
  .venv/bin/python -m pip install 'dimos[base,unitree,mapping]' Flask numpy PyYAML
EOF
      return 2
    fi
    python3 -m venv "$venv"
    "$py" -m pip install -q --upgrade pip
    "$py" -m pip install -q 'dimos[base,unitree,mapping]' Flask numpy PyYAML
  else
    "$py" -m pip install -q -r "$ROOT/robot_agent/requirements.txt"
  fi
  if [[ ! -x "$venv/bin/dimos" ]]; then
    echo "DimOS CLI missing at $venv/bin/dimos after install." >&2
    return 2
  fi
  echo "Orin DimOS venv ready: $py"
}

if [[ "$SKIP_VENV" -eq 0 ]]; then
  if [[ "$ROLE" == "host" || "$ROLE" == "both" ]]; then
    setup_host_venv
    # Legacy alias + link-profile aliases (wire/guest/dog_ap).
    ensure_ssh_alias "${SSH_TARGET:-go2-orin}" "${ORIN_HOST:-192.168.123.18}"
    dimos_ensure_link_ssh_aliases
  fi
  if [[ "$ROLE" == "orin" || "$ROLE" == "both" ]]; then
    setup_orin_venv
  fi
fi

cat <<EOF

Bootstrap complete (--role ${ROLE}).

Next:
EOF
if [[ "$ROLE" == "orin" || "$ROLE" == "both" ]]; then
  cat <<'EOF'
  Orin:  ./start_robot_agent.sh
  AES:   if Go2 firmware ≥1.1.15 needs a key, run scripts/fetch_go2_aes_key.sh from the operator PC
         (writes GO2CONNECTION__AES_128_KEY into robot_agent/.env).
EOF
fi
if [[ "$ROLE" == "host" || "$ROLE" == "both" ]]; then
  cat <<'EOF'
  Host:  ./start_remote_console.sh --link wire    # Ethernet (default)
         ./start_remote_console.sh --link guest   # after editing link_profiles/guest.env
         ./start_remote_console.sh --link dog_ap  # join Go2 WiFi AP first
         then open http://127.0.0.1:7781/
  Tip:   if Orin already has a token: ./scripts/bootstrap.sh --role host --pull-token-from go2-orin-wire
EOF
fi
