# Shared DimOS 0.0.15 launch flags. Sourced by start_mapping.sh / start_patrol.sh.
# Global options must come before `run`. Module flags such as rerun-open come after.

dimos_aarch64_preload() {
  # Open3D/NumPy on Jetson often fail with: libgomp.so.1 cannot allocate memory in static TLS block
  local gomp="/lib/aarch64-linux-gnu/libgomp.so.1"
  if [[ "$(uname -m)" == "aarch64" && -f "$gomp" ]]; then
    case ":${LD_PRELOAD:-}:" in
      *":$gomp:"*) ;;
      *) export LD_PRELOAD="${gomp}${LD_PRELOAD:+:$LD_PRELOAD}" ;;
    esac
  fi
  # PyTurboJPEG needs libturbojpeg; prefer distro lib, else vendored copy under simulation_legacy.
  local root
  root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  local tj_dir="$root/.local-libs"
  local tj
  for tj in /usr/lib/aarch64-linux-gnu/libturbojpeg.so.0 "$tj_dir/libturbojpeg.so.0" "$tj_dir/libturbojpeg.so"; do
    if [[ -f "$tj" ]]; then
      case ":${LD_LIBRARY_PATH:-}:" in
        *":$(dirname "$tj"):"*) ;;
        *) export LD_LIBRARY_PATH="$(dirname "$tj")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
      esac
      export TURBOJPEG="${TURBOJPEG:-$tj}"
      break
    fi
  done
}

dimos_global_args() {
  DIMOS_GLOBAL_ARGS=(--viewer rerun --listen-host "${DIMOS_LISTEN_HOST:-0.0.0.0}")
  case "${DIMOS_MODE:-simulation}" in
    simulation)
      DIMOS_GLOBAL_ARGS+=(--simulation mujoco)
      ;;
    real)
      if [[ -z "${ROBOT_IP:-}" ]]; then
        echo "Set ROBOT_IP to the Go2 MCU address (usually 192.168.123.161)." >&2
        return 2
      fi
      DIMOS_GLOBAL_ARGS+=(--robot-ip "$ROBOT_IP")
      ;;
    *)
      echo "DIMOS_MODE must be 'simulation' or 'real'." >&2
      return 2
      ;;
  esac
}

dimos_aes_args() {
  # DimOS 0.0.15 rejects the old UNITREE_AES_128_KEY env name.
  # Prefer the module env var so the key does not appear in `ps` argv.
  if [[ -n "${UNITREE_AES_128_KEY:-}" && -z "${GO2CONNECTION__AES_128_KEY:-}" ]]; then
    export GO2CONNECTION__AES_128_KEY="$UNITREE_AES_128_KEY"
  fi
  unset UNITREE_AES_128_KEY || true
  DIMOS_AES_ARGS=()
  if [[ -z "${GO2CONNECTION__AES_128_KEY:-}" ]]; then
    echo "Warning: GO2CONNECTION__AES_128_KEY is unset; Go2 ≥1.1.15 WebRTC will fail." >&2
  fi
}

dimos_viewer_args() {
  # Serve web UIs on Orin, but do NOT auto-open a browser there: DimOS would open
  # http://localhost:7779/ whose iframes hardcode broken localhost:9090/9876 URLs.
  # The host console opens the correct tunnel URLs once viz ports are up.
  local advertise="${DIMOS_RERUN_ADVERTISE_HOST:-${ORIN_AGENT_HOST:-192.168.123.18}}"
  local rerun_connect="${DIMOS_RERUN_CONNECT_URL:-rerun+http://${advertise}:9877/proxy}"
  DIMOS_VIEWER_ARGS=(
    websocket-vis-module
    --websocketvismodule.open-browser false
    --rerunbridgemodule.rerun-open none
    --rerunbridgemodule.rerun-web true
    --rerunbridgemodule.connect-url "$rerun_connect"
  )
}
