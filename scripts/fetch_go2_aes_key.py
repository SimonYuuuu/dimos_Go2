#!/usr/bin/env python3
"""Fetch the Go2 AES-128 key when Orin's system clock is years behind.

TLS fails with 'certificate is not yet valid' because this Jetson is stuck
in 2022. Pass CORRECT_UTC_EPOCH from a machine with the right time.
"""
from __future__ import annotations

import argparse
import getpass
import os
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ENV_PATH = ROOT / "robot_agent" / ".env"


def apply_clock(epoch: int) -> int:
    real_now = time.time()
    offset = float(epoch) - real_now

    def corrected() -> float:
        return real_now + offset + (time.process_time() * 0)

    # Keep a monotonic-ish wall clock starting at epoch.
    start = time.monotonic()

    def now() -> float:
        return float(epoch) + (time.monotonic() - start)

    time.time = now  # type: ignore[method-assign]
    return int(offset)


def patch_tls() -> None:
    from curl_cffi import requests as cffi_requests
    from unitree_webrtc_connect.unitree_cloud import UnitreeCloud

    original = UnitreeCloud.__init__

    def wrapped(self, *args, **kwargs):
        original(self, *args, **kwargs)
        self._session = cffi_requests.Session(impersonate="chrome120", verify=False)

    UnitreeCloud.__init__ = wrapped  # type: ignore[method-assign]


def write_env_key(key: str) -> None:
    ENV_PATH.parent.mkdir(parents=True, exist_ok=True)
    drop = ("UNITREE_AES_128_KEY=", "GO2CONNECTION__AES_128_KEY=")
    lines: list[str] = []
    if ENV_PATH.exists():
        lines = [
            line
            for line in ENV_PATH.read_text(encoding="utf-8").splitlines()
            if not line.startswith(drop)
        ]
    # DimOS 0.0.15 module env name; keep old alias out of the process environment.
    lines.append(f"GO2CONNECTION__AES_128_KEY={key}")
    ENV_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    os.chmod(ENV_PATH, 0o600)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--email", required=True, help="Real Unitree Go2 app email")
    parser.add_argument("--region", choices=["cn", "global"], default="cn")
    parser.add_argument("--epoch", type=int, default=int(os.environ.get("CORRECT_UTC_EPOCH", "0") or 0))
    parser.add_argument("--sn", default="")
    args = parser.parse_args()
    if "你的" in args.email or "example.com" in args.email:
        print("Use the real Unitree app email, not the placeholder.", file=sys.stderr)
        return 2

    skew = 0
    if args.epoch:
        skew = apply_clock(args.epoch)
        print(f"Using corrected UTC epoch {args.epoch} (offset {skew}s).", file=sys.stderr)
        patch_tls()
        print("TLS date check disabled because the board clock is wrong.", file=sys.stderr)
    elif abs(time.time() - 1762800000) > 86400 * 30:
        # 1762800000 ~ 2025-11; if we are far from 2025-2027, demand --epoch.
        print(
            "This machine's clock looks wrong. Run via scripts/fetch_go2_aes_key.sh "
            "from the operator PC so CORRECT_UTC_EPOCH is injected.",
            file=sys.stderr,
        )
        return 2

    password = getpass.getpass(f"Password for {args.email}: ")
    from unitree_webrtc_connect.unitree_cloud import UnitreeCloud, UnitreeCloudError

    cloud = UnitreeCloud(region=args.region, device_type="Go2")
    try:
        print(f"Logging in to {args.region} as {args.email}…", file=sys.stderr)
        cloud.login_email(args.email, password)
        devices = cloud.list_devices()
    except UnitreeCloudError as error:
        print(error, file=sys.stderr)
        return 1
    except Exception as error:
        print(f"Cloud request failed: {error}", file=sys.stderr)
        return 1

    if not devices:
        print(f"No devices in region={args.region}. Try --region global.", file=sys.stderr)
        return 1

    selected = None
    for device in devices:
        print(f"Found SN={device.sn} alias={device.alias or '-'} online={device.online} has_key={'yes' if device.key else 'no'}")
        if args.sn:
            if device.sn == args.sn:
                selected = device
        elif device.key and selected is None:
            selected = device
    if selected is None or not selected.key:
        print("No AES key in the device list. Bind the dog in the official app first.", file=sys.stderr)
        return 1
    if not re.fullmatch(r"[0-9a-fA-F]{32}", selected.key.strip()):
        print("Cloud key is not 32 hex characters.", file=sys.stderr)
        return 1
    write_env_key(selected.key.strip())
    print(f"Saved UNITREE_AES_128_KEY for SN {selected.sn} to {ENV_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
