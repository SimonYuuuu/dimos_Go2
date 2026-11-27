#!/usr/bin/env bash
# Orin wall-clock helper: NTP when online, or set from host epoch via agent.
# Install once (needs sudo): ./scripts/orin_timesync.sh install
set -euo pipefail

DIMOS_ROOT="${DIMOS_ROOT:-/home/unitree/dimos_Go2}"
if [[ -f "$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)/robot_agent/agent.py" ]]; then
  ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
else
  ROOT="$DIMOS_ROOT"
fi
INSTALLED_BIN="/usr/local/sbin/dimos-orin-timesync"
SUDOERS_FILE="/etc/sudoers.d/dimos-orin-timesync"
UNIT_FILE="/etc/systemd/system/dimos-timesync.service"
# Skew larger than this (seconds) is treated as "clock wrong".
SKEW_WARN_S="${DIMOS_TIME_SKEW_WARN_S:-120}"

usage() {
  cat <<'EOF'
Usage: orin_timesync.sh <command> [args]

  status              Show local time, NTP state, RTC
  sync-ntp            Enable NTP and wait briefly for sync
  set-epoch <unix>    Set UTC from a Unix epoch (host push)
  boot                Boot hook: enable NTP, best-effort sync
  install             Install root helper, sudoers, boot unit

After install, unitree can run without a password:
  sudo /usr/local/sbin/dimos-orin-timesync set-epoch $(date -u +%s)
EOF
}

run_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
    return
  fi
  if [[ -x "$INSTALLED_BIN" ]] && sudo -n "$INSTALLED_BIN" status >/dev/null 2>&1; then
    sudo -n "$INSTALLED_BIN" "$@"
    return
  fi
  echo "Need root for time changes. Run once: sudo $ROOT/scripts/orin_timesync.sh install" >&2
  return 1
}

cmd_status() {
  local now epoch ntp synced
  now="$(date -u '+%Y-%m-%dT%H:%M:%SZ' 2>/dev/null || date -u)"
  epoch="$(date -u +%s 2>/dev/null || echo 0)"
  ntp="unknown"
  synced="unknown"
  if command -v timedatectl >/dev/null 2>&1; then
    ntp="$(timedatectl show -p NTP --value 2>/dev/null || echo unknown)"
    synced="$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unknown)"
  fi
  printf 'utc=%s epoch=%s ntp=%s synchronized=%s\n' "$now" "$epoch" "$ntp" "$synced"
  timedatectl 2>/dev/null | head -12 || true
}

cmd_sync_ntp() {
  if [[ "$(id -u)" -ne 0 ]]; then
    run_root sync-ntp
    return
  fi
  if command -v timedatectl >/dev/null 2>&1; then
    timedatectl set-ntp true || true
    # systemd-timesyncd may need a kick after long skew.
    systemctl enable systemd-timesyncd.service >/dev/null 2>&1 || true
    systemctl restart systemd-timesyncd.service >/dev/null 2>&1 || true
  fi
  # Wait up to ~20s for NTP; do not fail boot if offline.
  local i
  for i in $(seq 1 20); do
    if timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx yes; then
      echo "NTP synchronized."
      cmd_status
      return 0
    fi
    sleep 1
  done
  echo "NTP not synchronized yet (offline or blocked). Host can push epoch via agent."
  cmd_status
  return 0
}

cmd_set_epoch() {
  local epoch="${1:-}"
  if [[ ! "$epoch" =~ ^[0-9]+$ ]]; then
    echo "set-epoch requires a Unix epoch integer" >&2
    return 2
  fi
  if [[ "$(id -u)" -ne 0 ]]; then
    run_root set-epoch "$epoch"
    return
  fi
  # Wall clock in the same zone as the operator laptop, so `date` matches the console.
  if command -v timedatectl >/dev/null 2>&1; then
    timedatectl set-timezone Asia/Shanghai || true
  fi
  # Avoid tiny adjustments that fight NTP; only correct large skew.
  local now skew
  now="$(date -u +%s)"
  skew=$((epoch - now))
  if [[ ${skew#-} -lt 2 ]]; then
    echo "Clock already within 2s of target (skew=${skew}s); timezone aligned."
    return 0
  fi
  # Disable NTP briefly so set-time sticks, then re-enable for future boots.
  if command -v timedatectl >/dev/null 2>&1; then
    timedatectl set-ntp false || true
    timedatectl set-time "@${epoch}" || date -u -s "@${epoch}"
    timedatectl set-ntp true || true
    systemctl restart systemd-timesyncd.service >/dev/null 2>&1 || true
  else
    date -u -s "@${epoch}"
  fi
  hwclock -w 2>/dev/null || true
  echo "Set UTC from epoch ${epoch} (previous skew ${skew}s)."
  cmd_status
}

cmd_boot() {
  if [[ "$(id -u)" -ne 0 ]]; then
    run_root boot
    return
  fi
  if command -v timedatectl >/dev/null 2>&1; then
    timedatectl set-timezone Asia/Shanghai || true
  fi
  cmd_sync_ntp || true
}

cmd_install() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "Re-running install with sudo..."
    exec sudo env DIMOS_TIME_INSTALL_PHASE=1 "$ROOT/scripts/orin_timesync.sh" install
  fi
  install -d /usr/local/sbin
  sed "s|^DIMOS_ROOT=.*|DIMOS_ROOT=\"$ROOT\"|" "$ROOT/scripts/orin_timesync.sh" >"$INSTALLED_BIN"
  chmod 755 "$INSTALLED_BIN"
  if [[ "${DIMOS_TIME_INSTALL_PHASE:-1}" != "2" ]]; then
    exec env DIMOS_TIME_INSTALL_PHASE=2 DIMOS_ROOT="$ROOT" "$INSTALLED_BIN" install
  fi
  cat >"$SUDOERS_FILE" <<EOF
# DimOS Orin timesync helper — installed by orin_timesync.sh
unitree ALL=(root) NOPASSWD: $INSTALLED_BIN
EOF
  chmod 440 "$SUDOERS_FILE"
  visudo -cf "$SUDOERS_FILE"
  cat >"$UNIT_FILE" <<EOF
[Unit]
Description=DimOS Orin NTP / clock bootstrap
After=network-online.target NetworkManager.service dimos-wifi-preferred.service
Wants=network-online.target

[Service]
Type=oneshot
Environment=DIMOS_ROOT=$ROOT
ExecStart=$INSTALLED_BIN boot
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable systemd-timesyncd.service >/dev/null 2>&1 || true
  systemctl enable dimos-timesync.service
  timedatectl set-ntp true || true
  "$INSTALLED_BIN" boot || true
  echo "Installed: $INSTALLED_BIN , $SUDOERS_FILE , dimos-timesync.service"
  "$INSTALLED_BIN" status || true
}

main() {
  local cmd="${1:-}"
  shift || true
  case "$cmd" in
    status) cmd_status "$@" ;;
    sync-ntp) cmd_sync_ntp "$@" ;;
    set-epoch) cmd_set_epoch "$@" ;;
    boot) cmd_boot "$@" ;;
    install) cmd_install "$@" ;;
    -h|--help|help|"") usage ;;
    *) echo "Unknown command: $cmd" >&2; usage; exit 2 ;;
  esac
}

main "$@"
