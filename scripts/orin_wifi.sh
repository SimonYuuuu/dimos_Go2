#!/usr/bin/env bash
# Orin WiFi helper: preferred profile, boot auto-connect, and safe nmcli actions.
# Install once (needs sudo): ./scripts/orin_wifi.sh install
set -euo pipefail

# Installed copy lives in /usr/local/sbin; keep an explicit project root.
DIMOS_ROOT="${DIMOS_ROOT:-/home/unitree/dimos_Go2}"
if [[ -f "$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)/robot_agent/wifi_profiles.json" ]]; then
  ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
else
  ROOT="$DIMOS_ROOT"
fi
CONFIG="${DIMOS_WIFI_CONFIG:-$ROOT/robot_agent/wifi_profiles.json}"
INSTALLED_BIN="/usr/local/sbin/dimos-orin-wifi"
SUDOERS_FILE="/etc/sudoers.d/dimos-orin-wifi"
UNIT_FILE="/etc/systemd/system/dimos-wifi-preferred.service"
DOG_LAN_UNIT_FILE="/etc/systemd/system/dimos-dog-lan.service"
NMCLI="${NMCLI:-/usr/bin/nmcli}"
# Go2 EDU internal LAN (Orin ↔ MCU). Match NIC by MAC so eth0/eth1 rename is OK.
DOG_LAN_IP="${DIMOS_DOG_LAN_IP:-192.168.123.18/24}"
DOG_LAN_MAC="${DIMOS_DOG_LAN_MAC:-74:25:54:ae:9c:c0}"
DOG_LAN_CONN="${DIMOS_DOG_LAN_CONN:-Wired connection 1}"

usage() {
  cat <<'EOF'
Usage: orin_wifi.sh <command> [args]

  status                 Show preferred profile + active WiFi
  list                   List configured profiles / NM wifi connections
  prefer <profile_id>    Set preferred profile (boot + future connects)
  connect [profile_id]   Connect now (default: preferred); raises its NM priority
  boot                   Apply preferred priorities and connect if SSID visible
  dog-lan                Ensure dog internal LAN IP (192.168.123.18) is up
  add <id> <ssid> <psk>  Create/update an NM wifi connection, then prefer it
  install                Install root helper, sudoers, and boot systemd units

Profiles live in robot_agent/wifi_profiles.json (preferred + nm_name map).
Dog LAN uses NIC MAC 74:25:54:ae:9c:c0 → 192.168.123.18/24 (override with
DIMOS_DOG_LAN_MAC / DIMOS_DOG_LAN_IP).
EOF
}

need_python() {
  if [[ -x "$ROOT/simulation_legacy/.venv/bin/python" ]]; then
    echo "$ROOT/simulation_legacy/.venv/bin/python"
  else
    command -v python3
  fi
}

read_config() {
  local py
  py="$(need_python)"
  "$py" - "$CONFIG" "$@" <<'PY'
import json, sys
path = sys.argv[1]
cmd = sys.argv[2]
data = json.loads(open(path, encoding="utf-8").read())
profiles = data.get("profiles") or {}
preferred = str(data.get("preferred") or "")
if cmd == "preferred":
    print(preferred)
elif cmd == "ids":
    print("\n".join(profiles.keys()))
elif cmd == "nm":
    pid = sys.argv[3]
    p = profiles.get(pid) or {}
    print(p.get("nm_name") or "")
elif cmd == "priority":
    pid = sys.argv[3]
    p = profiles.get(pid) or {}
    print(int(p.get("priority") or 50))
elif cmd == "label":
    pid = sys.argv[3]
    p = profiles.get(pid) or {}
    print(p.get("label") or pid)
elif cmd == "dump":
    print(json.dumps(data, ensure_ascii=False))
else:
    raise SystemExit(f"unknown read_config cmd {cmd}")
PY
}

write_preferred() {
  local pid="$1"
  local py
  py="$(need_python)"
  "$py" - "$CONFIG" "$pid" <<'PY'
import json, sys
path, pid = sys.argv[1], sys.argv[2]
data = json.loads(open(path, encoding="utf-8").read())
profiles = data.get("profiles") or {}
if pid not in profiles:
    raise SystemExit(f"unknown profile: {pid}")
data["preferred"] = pid
open(path, "w", encoding="utf-8").write(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
print(pid)
PY
}

upsert_profile() {
  local pid="$1" nm="$2" label="$3" priority="$4"
  local py
  py="$(need_python)"
  "$py" - "$CONFIG" "$pid" "$nm" "$label" "$priority" <<'PY'
import json, sys
path, pid, nm, label, priority = sys.argv[1:6]
data = json.loads(open(path, encoding="utf-8").read())
profiles = data.setdefault("profiles", {})
profiles[pid] = {
    "label": label,
    "nm_name": nm,
    "priority": int(priority),
}
data["preferred"] = pid
open(path, "w", encoding="utf-8").write(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
print(pid)
PY
}

run_nmcli() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$NMCLI" "$@"
  elif sudo -n true 2>/dev/null; then
    sudo -n "$NMCLI" "$@"
  elif [[ -x "$INSTALLED_BIN" && "$0" != "$INSTALLED_BIN" ]]; then
    # Agent/user path: delegated installed helper (NOPASSWD).
    sudo -n "$INSTALLED_BIN" _nmcli "$@"
  else
    echo "Need root nmcli. Run once: sudo $ROOT/scripts/orin_wifi.sh install" >&2
    return 1
  fi
}

cmd_status() {
  local preferred nm active
  preferred="$(read_config preferred)"
  nm="$(read_config nm "$preferred" 2>/dev/null || true)"
  active="$("$NMCLI" -t -f NAME,TYPE,DEVICE connection show --active 2>/dev/null | awk -F: '$2=="802-11-wireless"{print $1" ("$3")"; exit}')"
  echo "preferred=${preferred:-none} nm=${nm:-none}"
  echo "active_wifi=${active:-none}"
  echo "config=$CONFIG"
  echo "---saved wifi---"
  while IFS=: read -r name type _; do
    [[ "$type" == "802-11-wireless" ]] || continue
    ac="$("$NMCLI" -g connection.autoconnect connection show "$name" 2>/dev/null || true)"
    pr="$("$NMCLI" -g connection.autoconnect-priority connection show "$name" 2>/dev/null || true)"
    ssid="$("$NMCLI" -g 802-11-wireless.ssid connection show "$name" 2>/dev/null || true)"
    echo "  $name  ssid=$ssid  autoconnect=$ac  priority=$pr"
  done < <("$NMCLI" -t -f NAME,TYPE connection show 2>/dev/null)
}

cmd_list() {
  local py
  py="$(need_python)"
  "$py" - "$CONFIG" <<'PY'
import json, sys, subprocess
data = json.loads(open(sys.argv[1], encoding="utf-8").read())
preferred = data.get("preferred")
print(f"preferred={preferred}")
for pid, meta in (data.get("profiles") or {}).items():
    mark = "*" if pid == preferred else " "
    print(f"{mark} {pid}: {meta.get('label') or pid} -> nm={meta.get('nm_name')} priority={meta.get('priority')}")
try:
    out = subprocess.check_output(["nmcli", "-t", "-f", "NAME,TYPE", "connection", "show"], text=True)
except Exception as e:
    print(f"nmcli error: {e}")
    raise SystemExit(0)
print("---nm wifi connections---")
for line in out.splitlines():
    name, typ, *rest = line.split(":")
    if typ == "802-11-wireless":
        print(f"  {name}")
PY
}

apply_priorities() {
  local preferred="$1"
  local pid nm pri
  for pid in $(read_config ids); do
    nm="$(read_config nm "$pid")"
    [[ -n "$nm" ]] || continue
    if ! "$NMCLI" -t -f NAME connection show 2>/dev/null | grep -Fxq "$nm"; then
      echo "skip missing NM connection: $nm ($pid)"
      continue
    fi
    if [[ "$pid" == "$preferred" ]]; then
      pri="$(read_config priority "$pid")"
    else
      pri=10
    fi
    run_nmcli connection modify "$nm" \
      connection.autoconnect yes \
      connection.autoconnect-priority "$pri" >/dev/null
    echo "priority $nm -> $pri"
  done
}

ssid_visible() {
  local nm="$1"
  local ssid active
  ssid="$("$NMCLI" -g 802-11-wireless.ssid connection show "$nm" 2>/dev/null || true)"
  [[ -n "$ssid" ]] || return 1
  # Already associated with this connection — treat as visible.
  active="$("$NMCLI" -t -f NAME connection show --active 2>/dev/null || true)"
  if printf '%s\n' "$active" | grep -Fxq "$nm"; then
    return 0
  fi
  run_nmcli device wifi rescan >/dev/null 2>&1 || true
  sleep 2
  "$NMCLI" -t -f SSID device wifi list 2>/dev/null | grep -Fxq "$ssid"
}

cmd_connect() {
  local preferred pid nm
  preferred="$(read_config preferred)"
  pid="${1:-$preferred}"
  [[ -n "$pid" ]] || { echo "No preferred profile set." >&2; return 2; }
  nm="$(read_config nm "$pid")"
  [[ -n "$nm" ]] || { echo "Profile $pid missing nm_name" >&2; return 2; }
  if ! "$NMCLI" -t -f NAME connection show 2>/dev/null | grep -Fxq "$nm"; then
    echo "NM connection '$nm' not found. Create it first (orin_wifi.sh add ...)." >&2
    return 2
  fi
  write_preferred "$pid" >/dev/null
  apply_priorities "$pid"
  echo "Connecting $pid -> $nm ..."
  run_nmcli connection up "$nm"
}

cmd_prefer() {
  local pid="${1:?profile id required}"
  write_preferred "$pid" >/dev/null
  apply_priorities "$pid"
  echo "Preferred WiFi profile set to $pid (boot will prefer this SSID when visible)."
}

cmd_boot() {
  local preferred nm tries i
  preferred="$(read_config preferred)"
  [[ -n "$preferred" ]] || { echo "No preferred profile; nothing to do."; return 0; }
  nm="$(read_config nm "$preferred")"
  apply_priorities "$preferred"
  # Phone hotspots often appear after Orin finishes booting — retry for ~90s.
  sleep 3
  if [[ -z "$nm" ]]; then
    echo "preferred $preferred has no nm_name"
    return 0
  fi
  if ! "$NMCLI" -t -f NAME connection show 2>/dev/null | grep -Fxq "$nm"; then
    echo "NM connection $nm missing; keep waiting for other autoconnect."
    return 0
  fi
  # Already on preferred — nothing to do.
  if printf '%s\n' "$("$NMCLI" -t -f NAME connection show --active 2>/dev/null || true)" | grep -Fxq "$nm"; then
    echo "Preferred WiFi already active: $preferred ($nm)"
    return 0
  fi
  tries="${DIMOS_WIFI_BOOT_TRIES:-18}"
  for i in $(seq 1 "$tries"); do
    if ssid_visible "$nm"; then
      echo "Preferred SSID visible (try $i/$tries); bringing up $nm"
      if run_nmcli connection up "$nm"; then
        echo "Connected preferred WiFi: $preferred ($nm)"
        return 0
      fi
      echo "connection up failed; retrying..."
    else
      echo "waiting for preferred SSID '$preferred' ($i/$tries)..."
    fi
    sleep 5
  done
  echo "Preferred SSID not seen after ${tries} tries; NM may use lower-priority saved WiFi."
}

iface_by_mac() {
  local want="$1" path addr
  want="$(echo "$want" | tr 'A-F' 'a-f')"
  for path in /sys/class/net/*/address; do
    [[ -f "$path" ]] || continue
    addr="$(tr 'A-F' 'a-f' <"$path" | tr -d '\n')"
    if [[ "$addr" == "$want" ]]; then
      basename "$(dirname "$path")"
      return 0
    fi
  done
  return 1
}

cmd_dog_lan() {
  local iface tries i conn other
  tries="${DIMOS_DOG_LAN_TRIES:-24}"
  conn="$DOG_LAN_CONN"
  echo "Ensuring dog LAN ${DOG_LAN_IP} on MAC ${DOG_LAN_MAC}..."
  for i in $(seq 1 "$tries"); do
    iface="$(iface_by_mac "$DOG_LAN_MAC" || true)"
    if [[ -z "$iface" ]]; then
      echo "waiting for dog-LAN NIC ($i/$tries)..."
      sleep 2
      continue
    fi
    # Wait for carrier when possible.
    if [[ -f "/sys/class/net/$iface/carrier" ]] && [[ "$(cat "/sys/class/net/$iface/carrier" 2>/dev/null || echo 0)" != "1" ]]; then
      echo "waiting for carrier on $iface ($i/$tries)..."
      ip link set "$iface" up 2>/dev/null || true
      sleep 2
      continue
    fi
    # Disable the competing wired profile so DHCP "getting IP" cannot steal the NIC.
    for other in "Wired connection 1" "Wired connection 2"; do
      [[ "$other" == "$conn" ]] && continue
      if "$NMCLI" -t -f NAME connection show 2>/dev/null | grep -Fxq "$other"; then
        run_nmcli connection modify "$other" connection.autoconnect no >/dev/null 2>&1 || true
        run_nmcli connection down "$other" >/dev/null 2>&1 || true
      fi
    done
    if ! "$NMCLI" -t -f NAME connection show 2>/dev/null | grep -Fxq "$conn"; then
      run_nmcli connection add type ethernet ifname "$iface" con-name "$conn" \
        ipv4.method manual ipv4.addresses "$DOG_LAN_IP" \
        ipv4.never-default yes connection.autoconnect yes
    else
      run_nmcli connection modify "$conn" \
        connection.interface-name "$iface" \
        ipv4.method manual \
        ipv4.addresses "$DOG_LAN_IP" \
        ipv4.gateway "" \
        ipv4.never-default yes \
        connection.autoconnect yes
    fi
    if run_nmcli connection up "$conn"; then
      sleep 1
      if ip -4 addr show dev "$iface" 2>/dev/null | grep -q "${DOG_LAN_IP%/*}"; then
        echo "Dog LAN ready: $iface → $DOG_LAN_IP"
        return 0
      fi
      # Fallback if NM up "succeeds" without address.
      ip addr add "$DOG_LAN_IP" dev "$iface" 2>/dev/null || true
      ip link set "$iface" up 2>/dev/null || true
      if ip -4 addr show dev "$iface" 2>/dev/null | grep -q "${DOG_LAN_IP%/*}"; then
        echo "Dog LAN ready (ip addr fallback): $iface → $DOG_LAN_IP"
        return 0
      fi
    fi
    echo "dog-lan bring-up retry ($i/$tries) on $iface..."
    sleep 2
  done
  echo "FAILED to bring up dog LAN ${DOG_LAN_IP}" >&2
  return 1
}

cmd_add() {
  local pid="${1:?id}" ssid="${2:?ssid}" psk="${3:?psk}"
  local nm="$pid"
  if "$NMCLI" -t -f NAME connection show 2>/dev/null | grep -Fxq "$nm"; then
    run_nmcli connection modify "$nm" \
      802-11-wireless.ssid "$ssid" \
      802-11-wireless-security.key-mgmt wpa-psk \
      802-11-wireless-security.psk "$psk" \
      connection.autoconnect yes
  else
    run_nmcli connection add type wifi ifname "*" con-name "$nm" ssid "$ssid" \
      wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$psk" \
      connection.autoconnect yes
  fi
  upsert_profile "$pid" "$nm" "$ssid" 120 >/dev/null
  apply_priorities "$pid"
  echo "Added/updated profile $pid (ssid=$ssid). Connecting..."
  run_nmcli connection up "$nm"
}

cmd_install() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "Re-running install with sudo..."
    exec sudo env DIMOS_WIFI_INSTALL_PHASE=1 "$ROOT/scripts/orin_wifi.sh" install
  fi
  install -d /usr/local/sbin
  # Bake project root into the installed helper so /usr/local/sbin path still works.
  sed "s|^DIMOS_ROOT=.*|DIMOS_ROOT=\"$ROOT\"|" "$ROOT/scripts/orin_wifi.sh" >"$INSTALLED_BIN"
  chmod 755 "$INSTALLED_BIN"
  # Re-exec the freshly installed copy so unit files match the new script.
  if [[ "${DIMOS_WIFI_INSTALL_PHASE:-1}" != "2" ]]; then
    export DIMOS_WIFI_INSTALL_PHASE=2
    exec env DIMOS_WIFI_INSTALL_PHASE=2 DIMOS_ROOT="$ROOT" "$INSTALLED_BIN" install
  fi
  cat >"$SUDOERS_FILE" <<EOF
# DimOS Orin WiFi helper — installed by orin_wifi.sh
unitree ALL=(root) NOPASSWD: $INSTALLED_BIN
EOF
  chmod 440 "$SUDOERS_FILE"
  visudo -cf "$SUDOERS_FILE"
  cat >"$UNIT_FILE" <<EOF
[Unit]
Description=DimOS preferred WiFi auto-connect
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
Environment=DIMOS_ROOT=$ROOT
ExecStart=$INSTALLED_BIN boot
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
  cat >"$DOG_LAN_UNIT_FILE" <<EOF
[Unit]
Description=DimOS Go2 dog-LAN static IP (192.168.123.18)
After=NetworkManager.service
Wants=NetworkManager.service
Before=dimos-wifi-preferred.service

[Service]
Type=oneshot
Environment=DIMOS_ROOT=$ROOT
Environment=DIMOS_DOG_LAN_IP=$DOG_LAN_IP
Environment=DIMOS_DOG_LAN_MAC=$DOG_LAN_MAC
ExecStart=$INSTALLED_BIN dog-lan
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable dimos-wifi-preferred.service
  systemctl enable dimos-dog-lan.service
  # Apply priorities only on install (full boot retry can wait for hotspot ~90s).
  preferred="$(read_config preferred || true)"
  if [[ -n "$preferred" ]]; then
    apply_priorities "$preferred" || true
  fi
  "$INSTALLED_BIN" dog-lan || true
  echo "Installed: $INSTALLED_BIN , $SUDOERS_FILE , dimos-wifi-preferred.service , dimos-dog-lan.service"
  "$INSTALLED_BIN" status || true
}

cmd_nmcli_passthrough() {
  # Used by installed binary under sudo for nested calls — not for users.
  shift || true
  "$NMCLI" "$@"
}

main() {
  local cmd="${1:-}"
  shift || true
  case "$cmd" in
    status) cmd_status "$@" ;;
    list) cmd_list "$@" ;;
    prefer) cmd_prefer "$@" ;;
    connect) cmd_connect "$@" ;;
    boot) cmd_boot "$@" ;;
    dog-lan) cmd_dog_lan "$@" ;;
    add) cmd_add "$@" ;;
    install) cmd_install "$@" ;;
    _nmcli) cmd_nmcli_passthrough _nmcli "$@" ;;
    -h|--help|help|"") usage ;;
    *) echo "Unknown command: $cmd" >&2; usage; exit 2 ;;
  esac
}

main "$@"
