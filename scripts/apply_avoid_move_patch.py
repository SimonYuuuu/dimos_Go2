#!/usr/bin/env python3
"""Keep Sport Move, and ask that controller to avoid obstacles.

Handing velocity to the avoidance Move API (and UseRemoteCommandFromApi)
took the body away from Sport Move. A yaw-only turn on that API was ignored,
so the dog stood still. Sport Move is what actually walks. SwitchAvoidMode
makes that same command steer around obstacles.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_AVOID_MOVE_V2"

OLD_IMPORT = """from unitree_webrtc_connect.constants import (
    DATA_CHANNEL_TYPE,
    RTC_TOPIC,
    SPORT_CMD,
    VUI_COLOR,
)
"""

V1_IMPORT = """from unitree_webrtc_connect.constants import (
    DATA_CHANNEL_TYPE,
    OBSTACLES_AVOID_API,
    RTC_TOPIC,
    SPORT_CMD,
    VUI_COLOR,
)
"""

NEW_IMPORT = """from unitree_webrtc_connect.constants import (
    DATA_CHANNEL_TYPE,
    OBSTACLES_AVOID_API,
    RTC_TOPIC,
    SPORT_CMD,
    SPORT_CMD_MCF,
    VUI_COLOR,
)
"""

OLD_INIT = """        self._velocity_api = velocity_api
        self._move_ids = SequentialIds()
"""

NEW_INIT = """        self._velocity_api = velocity_api
        self._obstacle_avoidance = False  # DIMOS_AVOID_MOVE_V1
        self._move_ids = SequentialIds()
"""

OLD_PUBLISH = '''    def _publish_movement(self, x: float, y: float, yaw: float) -> None:
        if self._velocity_api:
            self.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["SPORT_MOD"],
                data={
                    "header": {
                        "identity": {
                            "id": self._move_ids.next() + 1,
                            "api_id": SPORT_CMD["Move"],
                        }
                    },
                    "parameter": json.dumps({"x": x, "y": y, "z": yaw}),
                },
                msg_type=DATA_CHANNEL_TYPE["REQUEST"],
            )
            return
'''

V1_PUBLISH = '''    def _publish_movement(self, x: float, y: float, yaw: float) -> None:
        # DIMOS_AVOID_MOVE_V1 — Sport Move ignores the avoidance service.
        if self._velocity_api and self._obstacle_avoidance:
            self.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["OBSTACLES_AVOID"],
                data={
                    "header": {
                        "identity": {
                            "id": self._move_ids.next() + 1,
                            "api_id": OBSTACLES_AVOID_API["MOVE"],
                        }
                    },
                    "parameter": json.dumps({"x": x, "y": y, "yaw": yaw, "mode": 0}),
                },
                msg_type=DATA_CHANNEL_TYPE["REQUEST"],
            )
            return
        if self._velocity_api:'''

SPORT_PUBLISH_HEAD = '''    def _publish_movement(self, x: float, y: float, yaw: float) -> None:
        # DIMOS_AVOID_MOVE_V2 — Sport Move drives; SwitchAvoidMode steers it.
        if self._velocity_api:'''

V1_SET = '''    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        # DIMOS_AVOID_MOVE_V1 — arm the service, then feed it API velocity.
        switched = bool(
            self.publish_request(
                RTC_TOPIC["OBSTACLES_AVOID"],
                {
                    "api_id": OBSTACLES_AVOID_API["SWITCH_SET"],
                    "parameter": {"enable": int(enabled)},
                },
            )
        )
        remote = bool(
            self.publish_request(
                RTC_TOPIC["OBSTACLES_AVOID"],
                {
                    "api_id": OBSTACLES_AVOID_API["USE_REMOTE_COMMAND_FROM_API"],
                    "parameter": {"is_remote_commands_from_api": bool(enabled)},
                },
            )
        )
        self._obstacle_avoidance = bool(enabled and switched)
        logger.info(
            "obstacle avoidance enabled=%s switch=%s api_move=%s",
            enabled,
            switched,
            remote,
        )
        return switched
'''

OLD_SET = '''    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        return bool(
            self.publish_request(
                RTC_TOPIC["OBSTACLES_AVOID"],
                {"api_id": 1001, "parameter": {"enable": int(enabled)}},
            )
        )
'''

NEW_SET = '''    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        # DIMOS_AVOID_MOVE_V2 — release the avoidance Move API, keep Sport Move.
        switched = self.publish_request(
            RTC_TOPIC["OBSTACLES_AVOID"],
            {
                "api_id": OBSTACLES_AVOID_API["SWITCH_SET"],
                "parameter": {"enable": int(enabled)},
            },
        )
        released = self.publish_request(
            RTC_TOPIC["OBSTACLES_AVOID"],
            {
                "api_id": OBSTACLES_AVOID_API["USE_REMOTE_COMMAND_FROM_API"],
                "parameter": {"is_remote_commands_from_api": False},
            },
        )
        try:
            sport_avoid = self.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {
                    "api_id": SPORT_CMD_MCF["SwitchAvoidMode"],
                    "parameter": {"enable": bool(enabled)},
                },
            )
        except Exception:
            logger.exception("sport SwitchAvoidMode failed")
            sport_avoid = None
        self._obstacle_avoidance = False
        logger.info(
            "obstacle avoidance enabled=%s switch=%s release_api=%s sport_avoid=%s",
            enabled,
            switched,
            released,
            sport_avoid,
        )
        return bool(switched)
'''


def main() -> int:
    target = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
        / "dimos/robot/unitree/connection.py"
    )
    if not target.is_file():
        print(f"not found: {target}", file=sys.stderr)
        return 1
    text = target.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"already patched: {target}")
        return 0
    if "DIMOS_AVOID_MOVE_V1" in text:
        for label, block, repl in (
            ("import", V1_IMPORT, NEW_IMPORT),
            ("publish", V1_PUBLISH, SPORT_PUBLISH_HEAD),
            ("set", V1_SET, NEW_SET),
        ):
            if block not in text:
                print(f"v1 {label} block not found", file=sys.stderr)
                return 1
            text = text.replace(block, repl, 1)
    else:
        for label, block, repl in (
            ("import", OLD_IMPORT, NEW_IMPORT),
            ("init", OLD_INIT, NEW_INIT),
            ("set", OLD_SET, NEW_SET),
        ):
            if block not in text:
                print(f"{label} block not found", file=sys.stderr)
                return 1
            text = text.replace(block, repl, 1)
        if SPORT_PUBLISH_HEAD not in text and OLD_PUBLISH in text:
            text = text.replace(
                "    def _publish_movement(self, x: float, y: float, yaw: float) -> None:\n        if self._velocity_api:",
                SPORT_PUBLISH_HEAD,
                1,
            )
    target.write_text(text, encoding="utf-8")
    print(f"patched {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
