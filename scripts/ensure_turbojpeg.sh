#!/usr/bin/env bash
# Ensure libturbojpeg is available for DimOS camera JPEG encoding on Orin.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/simulation_legacy/.local-libs"

have_lib() {
  local p
  for p in \
    /usr/lib/aarch64-linux-gnu/libturbojpeg.so.0 \
    /usr/lib/aarch64-linux-gnu/libturbojpeg.so \
    "$DEST/libturbojpeg.so.0" \
    "$DEST/libturbojpeg.so"; do
    [[ -f "$p" || -L "$p" ]] && return 0
  done
  return 1
}

if have_lib; then
  echo "libturbojpeg already available."
  exit 0
fi

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "Skipping turbojpeg vendoring on $(uname -m) (Orin/aarch64 only)."
  exit 0
fi

mkdir -p "$DEST"

if command -v apt-get >/dev/null 2>&1; then
  if sudo -n true 2>/dev/null; then
    sudo apt-get update -y >/dev/null || true
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y libturbojpeg0 >/dev/null 2>&1 || \
      sudo DEBIAN_FRONTEND=noninteractive apt-get install -y libjpeg-turbo8 >/dev/null 2>&1 || true
  fi
  if have_lib; then
    echo "libturbojpeg installed via apt."
    exit 0
  fi
fi

TMP="$(mktemp -d)"
cleanup() { rm -rf "$TMP"; }
trap cleanup EXIT

DEB_URL="${TURBOJPEG_DEB_URL:-http://ports.ubuntu.com/pool/universe/libj/libjpeg-turbo/libturbojpeg0_2.1.2-0ubuntu1_arm64.deb}"
echo "Downloading libturbojpeg from ${DEB_URL}"
if command -v curl >/dev/null 2>&1; then
  curl -fsSL "$DEB_URL" -o "$TMP/libturbojpeg.deb"
elif command -v wget >/dev/null 2>&1; then
  wget -qO "$TMP/libturbojpeg.deb" "$DEB_URL"
else
  echo "Need curl/wget or apt package libturbojpeg0." >&2
  exit 2
fi

dpkg-deb -x "$TMP/libturbojpeg.deb" "$TMP/root"
mapfile -t SOS < <(find "$TMP/root" -name 'libturbojpeg.so*' | sort)
if [[ "${#SOS[@]}" -eq 0 ]]; then
  echo "Failed to extract libturbojpeg.so from deb." >&2
  exit 2
fi
cp -a "${SOS[@]}" "$DEST/"
# Prefer versioned real file for the unversioned soname link.
REAL="$(find "$DEST" -maxdepth 1 -type f -name 'libturbojpeg.so.*' | sort | tail -1)"
if [[ -n "$REAL" ]]; then
  ln -sfn "$(basename "$REAL")" "$DEST/libturbojpeg.so.0"
  ln -sfn "$(basename "$REAL")" "$DEST/libturbojpeg.so"
fi
echo "Vendored libturbojpeg into $DEST"
