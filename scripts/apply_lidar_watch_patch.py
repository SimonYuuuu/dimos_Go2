#!/usr/bin/env python3
"""Keep asking the dog for lidar until a cloud actually arrives.

disable_traffic_saving and the utlidar subscribe each run once at connect.
If the dog answers its public-network query with 4G first, the firmware
holds the cloud back and that one request is not repeated. A quiet stream
now re-disables traffic saving and sends the subscribe again.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_LIDAR_WATCH_V1"


def edit(path: Path, pairs: list[tuple[str, str]]) -> None:
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {path.name} {MARKER}")
        return
    for old, new in pairs:
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"{path.name}: anchor found {count} times: {old[:70]!r}")
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} -> {MARKER}")


def main() -> None:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    edit(
        site / "unitree_webrtc_connect/msgs/pub_sub.py",
        [
            (
                "def subscribe(self, topic, callback=None):\n"
                "        channel = self.channel\n"
                "\n"
                '        if not channel or channel.readyState != "open":\n'
                '            print("Error: Data channel is not open")\n'
                "            return\n"
                "        \n"
                "        # Register the callback for the topic\n"
                "        if callback:\n"
                "            self.subscriptions[topic] = callback\n"
                "\n"
                '        self.publish_without_callback(topic=topic, msg_type=DATA_CHANNEL_TYPE["SUBSCRIBE"])\n',
                "def subscribe(self, topic, callback=None):\n"
                f"        # {MARKER} — remember the callback even if the channel is not open yet.\n"
                "        if callback:\n"
                "            self.subscriptions[topic] = callback\n"
                "        channel = self.channel\n"
                '        if not channel or channel.readyState != "open":\n'
                '            print("Error: Data channel is not open")\n'
                "            return\n"
                '        self.publish_without_callback(topic=topic, msg_type=DATA_CHANNEL_TYPE["SUBSCRIBE"])\n',
            )
        ],
    )
    edit(
        site / "dimos/robot/unitree/connection.py",
        [
            (
                "    def start(self) -> None:\n"
                "        pass\n",
                "    def start(self) -> None:\n"
                "        pass\n"
                "\n"
                "    def revive_lidar(self) -> None:\n"
                f"        # {MARKER} — the dog drops the cloud while it reports a 4G uplink.\n"
                "        import asyncio\n"
                "\n"
                "        async def _run() -> None:\n"
                "            await self.conn.datachannel.disableTrafficSaving(True)\n"
                '            topic = RTC_TOPIC["ULIDAR_ARRAY"]\n'
                "            self.conn.datachannel.pub_sub.subscribe(topic)\n"
                "\n"
                "        if getattr(self, \"loop\", None) is not None and self.loop.is_running():\n"
                "            asyncio.run_coroutine_threadsafe(_run(), self.loop)\n",
            )
        ],
    )
    edit(
        site / "dimos/robot/unitree/go2/connection.py",
        [
            (
                "        if self.config.lidar:\n"
                "            self.register_disposable(self.connection.lidar_stream().subscribe(self.lidar.publish))\n",
                "        if self.config.lidar:\n"
                f"            # {MARKER}\n"
                "            self._lidar_rx = 0.0\n"
                "            self._lidar_n = 0\n"
                "\n"
                "            def _on_lidar(msg: PointCloud2) -> None:\n"
                "                self._lidar_rx = time.monotonic()\n"
                "                self._lidar_n += 1\n"
                "                if self._lidar_n == 1 or self._lidar_n % 40 == 0:\n"
                '                    logger.info("lidar stream frames=%s points=%s", self._lidar_n, len(msg))\n'
                "                self.lidar.publish(msg)\n"
                "\n"
                "            self.register_disposable(self.connection.lidar_stream().subscribe(_on_lidar))\n"
                "            Thread(target=self._watch_lidar, name=\"lidar-watch\", daemon=True).start()\n",
            ),
            (
                "    @rpc\n"
                "    def stop(self) -> None:\n",
                "    def _watch_lidar(self) -> None:\n"
                f"        # {MARKER} — repeat the request until a cloud shows up, and again if it stops.\n"
                "        while not self._camera_info_stop.is_set():\n"
                "            self._camera_info_stop.wait(2.0)\n"
                "            if self._camera_info_stop.is_set():\n"
                "                return\n"
                "            last = getattr(self, \"_lidar_rx\", 0.0)\n"
                "            if last and time.monotonic() - last < 2.0:\n"
                "                continue\n"
                "            revive = getattr(self.connection, \"revive_lidar\", None)\n"
                "            if not callable(revive):\n"
                "                return\n"
                "            try:\n"
                "                revive()\n"
                "            except Exception:\n"
                '                logger.warning("lidar revive failed", exc_info=True)\n'
                "                continue\n"
                "            quiet = time.monotonic() - last if last else 0.0\n"
                "            logger.warning(\n"
                '                "lidar stream quiet%s; asked the dog to send the cloud again",\n'
                '                f" for {quiet:.0f}s" if last else " since connect",\n'
                "            )\n"
                "\n"
                "    @rpc\n"
                "    def stop(self) -> None:\n",
            ),
        ],
    )


if __name__ == "__main__":
    main()
