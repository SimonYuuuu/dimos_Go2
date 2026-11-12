# Link profile helpers for host↔Orin. Sourced by start_remote_console.sh / bootstrap.
# Profiles: wire (default) | guest | dog_ap

_dimos_link_profiles_dir() {
  echo "${ROOT}/remote_client/link_profiles"
}

dimos_seed_link_profiles() {
  local dir profile src dst
  dir="$(_dimos_link_profiles_dir)"
  mkdir -p "$dir"
  for profile in wire guest dog_ap; do
    src="${dir}/${profile}.env.example"
    dst="${dir}/${profile}.env"
    if [[ -f "$src" && ! -f "$dst" ]]; then
      cp "$src" "$dst"
      chmod 600 "$dst" 2>/dev/null || true
      echo "Seeded ${dst} from example (edit IPs for guest/dog_ap as needed)."
    fi
  done
}

_dimos_append_ssh_host() {
  local target="$1"
  local host="$2"
  local cfg="${HOME}/.ssh/config"
  mkdir -p "${HOME}/.ssh"
  chmod 700 "${HOME}/.ssh" 2>/dev/null || true
  if [[ ! -f "$cfg" ]]; then
    if ! touch "$cfg" 2>/dev/null; then
      echo "WARN: cannot write ${cfg}; add Host ${target} → ${host} manually." >&2
      return 0
    fi
  fi
  chmod 600 "$cfg" 2>/dev/null || true
  if grep -Eq "^[[:space:]]*Host[[:space:]]+${target}([[:space:]]|$)" "$cfg" 2>/dev/null; then
    return 0
  fi
  if ! cat >>"$cfg" <<EOF

Host ${target}
  HostName ${host}
  User unitree
  StrictHostKeyChecking accept-new
EOF
  then
    echo "WARN: cannot append Host ${target} to ${cfg}; add it manually → ${host}." >&2
    return 0
  fi
  echo "Appended SSH Host ${target} → ${host}"
}

dimos_ensure_link_ssh_aliases() {
  local dir profile host target
  dir="$(_dimos_link_profiles_dir)"

  for profile in wire guest dog_ap; do
    host=""
    target=""
    if [[ -f "${dir}/${profile}.env" ]]; then
      host="$(grep -E '^ORIN_AGENT_HOST=' "${dir}/${profile}.env" | head -1 | cut -d= -f2- || true)"
      target="$(grep -E '^ORIN_SSH_TARGET=' "${dir}/${profile}.env" | head -1 | cut -d= -f2- || true)"
    elif [[ -f "${dir}/${profile}.env.example" ]]; then
      host="$(grep -E '^ORIN_AGENT_HOST=' "${dir}/${profile}.env.example" | head -1 | cut -d= -f2- || true)"
      target="$(grep -E '^ORIN_SSH_TARGET=' "${dir}/${profile}.env.example" | head -1 | cut -d= -f2- || true)"
    fi
    [[ -n "$target" && -n "$host" ]] || continue
    case "$host" in SET_ME_*|"") continue ;; esac
    _dimos_append_ssh_host "$target" "$host"
  done

  # Keep legacy alias go2-orin → wire host for older docs/scripts.
  local wire_host="192.168.123.18"
  if [[ -f "${dir}/wire.env" ]]; then
    wire_host="$(grep -E '^ORIN_AGENT_HOST=' "${dir}/wire.env" | head -1 | cut -d= -f2- || echo "$wire_host")"
  fi
  _dimos_append_ssh_host "go2-orin" "$wire_host"
}

dimos_parse_link_args() {
  LINK_CLI=""
  DIMOS_PASSTHROUGH_ARGS=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --link)
        LINK_CLI="${2:-}"
        shift 2
        ;;
      --link=*)
        LINK_CLI="${1#*=}"
        shift
        ;;
      -h|--help)
        cat <<'EOF'
Usage: ./start_remote_console.sh [--link wire|guest|dog_ap]

  --link   Host↔Orin path (default: LINK_PROFILE in remote_client/.env, else wire)
           wire    = Ethernet 192.168.123.18
           guest   = ZF/warehouse guest WiFi (edit link_profiles/guest.env)
           dog_ap  = Go2 body WiFi AP (near range)

Edit IPs in remote_client/link_profiles/<name>.env (seeded from *.env.example).
EOF
        exit 0
        ;;
      *)
        DIMOS_PASSTHROUGH_ARGS+=("$1")
        shift
        ;;
    esac
  done
}

dimos_apply_link_profile() {
  local profile="${1:-wire}"
  local dir file
  case "$profile" in
    wire|guest|dog_ap) ;;
    *)
      echo "Unknown link profile '${profile}'. Use wire|guest|dog_ap." >&2
      return 2
      ;;
  esac

  dir="$(_dimos_link_profiles_dir)"
  dimos_seed_link_profiles
  file="${dir}/${profile}.env"
  if [[ ! -f "$file" ]]; then
    echo "Missing link profile ${file}" >&2
    return 2
  fi

  # shellcheck disable=SC1090
  set -a
  # shellcheck disable=SC1090
  source "$file"
  set +a

  export LINK_PROFILE="$profile"
  ORIN_SSH_TARGET="${ORIN_SSH_TARGET:-go2-orin-${profile}}"
  ORIN_AGENT_HOST="${ORIN_AGENT_HOST:-}"
  ORIN_AGENT_PORT="${ORIN_AGENT_PORT:-8787}"

  if [[ -z "$ORIN_AGENT_HOST" || "$ORIN_AGENT_HOST" == SET_ME_* ]]; then
    echo "Link profile '${profile}' has unset ORIN_AGENT_HOST in ${file}" >&2
    echo "Edit that file with Orin's address on this network, then retry." >&2
    return 2
  fi

  export ORIN_SSH_TARGET ORIN_AGENT_HOST ORIN_AGENT_PORT
  # Always rebuild URL from resolved host (profile examples may contain SET_ME_*).
  export ROBOT_AGENT_URL="http://${ORIN_AGENT_HOST}:${ORIN_AGENT_PORT}"
  echo "Link profile: ${profile} → agent ${ROBOT_AGENT_URL} (ssh ${ORIN_SSH_TARGET})"
}

dimos_preflight_link() {
  local host="$1"
  local ssh_target="$2"
  local port="${3:-8787}"

  if command -v ping >/dev/null 2>&1; then
    if ! ping -c 1 -W 2 "$host" >/dev/null 2>&1; then
      echo "WARN: ping ${host} failed (ICMP may be blocked). Continuing with TCP checks…" >&2
    fi
  fi

  if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$ssh_target" "true" 2>/dev/null; then
    echo "Cannot SSH to ${ssh_target} (${host})." >&2
    case "${LINK_PROFILE:-}" in
      guest)
        echo "Guest WiFi tip: client isolation often blocks host↔Orin. Need IT allow-list or a non-guest SSID." >&2
        ;;
      dog_ap)
        echo "Dog AP tip: join Go2 WiFi first; confirm Orin IP (often still 192.168.123.18) with ping/ip." >&2
        ;;
      *)
        echo "Wire tip: confirm Ethernet to Go2 LAN and that ${host} is the Orin." >&2
        ;;
    esac
    return 1
  fi

  if ! curl -fsS --max-time 5 "http://${host}:${port}/health" >/dev/null 2>&1; then
    echo "SSH works but http://${host}:${port}/health failed." >&2
    echo "On Orin run: ./start_robot_agent.sh" >&2
    return 1
  fi
  return 0
}
