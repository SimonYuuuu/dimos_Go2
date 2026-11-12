#!/usr/bin/env python3
"""Check that the Go2 MCU is reachable for DimOS (WebRTC or DDS sport)."""
from __future__ import annotations

import os
import socket
import sys
import time


def tcp_open(ip: str, port: int, timeout: float = 0.4) -> bool:
    sock = socket.socket()
    sock.settimeout(timeout)
    try:
        return sock.connect_ex((ip, port)) == 0
    finally:
        sock.close()


def dds_sport_visible(iface: str = "eth0") -> bool:
    try:
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
        from cyclonedds.domain import DomainParticipant
    except Exception:
        return False
    try:
        participant = DomainParticipant(0)
        reader = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
        time.sleep(3.0)
        topics = {sample.topic_name for sample in reader.take(N=400)}
        # A live sport server publishes responses; sbus_handle only publishes requests.
        return "rt/api/sport/response" in topics or "rt/sportmodestate" in topics
    except Exception:
        return False


def main() -> int:
    ip = os.environ.get("ROBOT_IP", "192.168.123.161")
    webrtc = tcp_open(ip, 9991) or tcp_open(ip, 8081)
    dds = dds_sport_visible()
    if webrtc:
        print(f"Go2 {ip}: WebRTC signaling is open.")
        return 0
    if dds:
        print(f"Go2 {ip}: DDS sport service is visible (WebRTC 8081/9991 still closed).")
        return 0
    print(
        f"Go2 {ip} answers ping but is not ready for DimOS.\n"
        "  - WebRTC signaling ports 9991 and 8081 are closed\n"
        "  - DDS sport server is not publishing (only the remote sbus_handle client is visible)\n"
        "Use the official Unitree remote to fully boot the dog into a controllable state\n"
        "(battery on, MCU services up). Then StandUp with the remote once. Retry mapping after\n"
        "8081 or 9991 is open, or after rt/sportmodestate appears on DDS.",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
