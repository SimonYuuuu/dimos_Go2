#!/usr/bin/env python3
"""Steady teleop over WiFi, and a stand-up that waits for the dog.

1. Keyboard Twists arrive over an SSH tunnel and can be late. The stock
   0.2 s deadman then sends a zero Move in the middle of a walk, which
   stutters the gait. With DIMOS_MOVE_HOLD_S > 0 the last command is re-sent
   at 25 Hz until it is that old, then the dog is stopped. A release sends a
   few zeros and then stays quiet instead of streaming Move(0) forever.
2. StandUp -> fixed 3 s -> BalanceStand can land mid-rise, which leaves the
   rear legs low. Now BalanceStand waits until sportmodestate reports a
   standing body height.
3. While moving, the commanded and the actual velocity are logged every 5 s.
"""

from __future__ import annotations

import sys
from pathlib import Path

MARKER = "DIMOS_TELEOP_HOLD_V1"

HOLD_FN = '''def _move_hold_s() -> float:
    # DIMOS_TELEOP_HOLD_V1 — 0 keeps the stock 0.2 s deadman.
    import os

    try:
        return max(0.0, min(2.0, float(os.environ.get("DIMOS_MOVE_HOLD_S", "0") or 0)))
    except ValueError:
        return 0.0


'''

INIT_OLD = """        self._obstacle_avoidance = False  # DIMOS_AVOID_MOVE_V1
        self._move_ids = SequentialIds()
"""
INIT_NEW = """        self._obstacle_avoidance = False  # DIMOS_AVOID_MOVE_V1
        self._move_ids = SequentialIds()
        # DIMOS_TELEOP_HOLD_V1
        self._hold_lock = threading.Lock()
        self._hold_stop = threading.Event()
        self._hold_thread: threading.Thread | None = None
        self._hold_cmd = (0.0, 0.0, 0.0)
        self._hold_ts = 0.0
        self._hold_moving = False
        self._hold_zero_left = 0
        self._hold_log_ts = 0.0
        self._sport_state: dict[str, Any] = {}
"""

STOP_OLD = """    def stop(self) -> None:
        # Cancel timer
"""
STOP_NEW = """    def stop(self) -> None:
        self._hold_stop.set()  # DIMOS_TELEOP_HOLD_V1
        # Cancel timer
"""

METHODS = '''    def start_state_monitor(self) -> None:
        # DIMOS_TELEOP_HOLD_V1 — latest sportmodestate: body height, gait, velocity.
        def on_state(msg: Any) -> None:
            try:
                d = msg.get("data") or {}
                self._sport_state = {
                    "t": time.monotonic(),
                    "height": float(d.get("body_height", 0.0)),
                    "gait": d.get("gait_type"),
                    "mode": d.get("mode"),
                    "foot": d.get("foot_raise_height"),
                    "err": d.get("error_code"),
                    "vel": list(d.get("velocity") or [0.0, 0.0, 0.0]),
                    "yaw": d.get("yaw_speed"),
                }
            except (AttributeError, TypeError, ValueError):
                pass

        def subscribe() -> None:
            self.conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], on_state)

        self.loop.call_soon_threadsafe(subscribe)

    def wait_standing(self, timeout: float = 10.0, min_height: float = 0.22) -> bool:
        # DIMOS_TELEOP_HOLD_V1
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self._sport_state
            if state and time.monotonic() - state["t"] < 1.0 and state["height"] >= min_height:
                return True
            time.sleep(0.2)
        return False

    def _hold_move(self, x: float, y: float, yaw: float) -> bool:
        # DIMOS_TELEOP_HOLD_V1 — remember the command; _hold_loop keeps sending it.
        moving = bool(x or y or yaw)
        with self._hold_lock:
            if self._hold_moving and not moving:
                self._hold_zero_left = 6
            self._hold_moving = moving
            self._hold_cmd = (x, y, yaw)
            self._hold_ts = time.monotonic()
            if self._hold_thread is None or not self._hold_thread.is_alive():
                self._hold_stop.clear()
                self._hold_thread = threading.Thread(
                    target=self._hold_loop, name="move-hold", daemon=True
                )
                self._hold_thread.start()
        return True

    def _hold_loop(self) -> None:
        # DIMOS_TELEOP_HOLD_V1
        while not self._hold_stop.is_set():
            send = False
            with self._hold_lock:
                cmd = self._hold_cmd
                age = time.monotonic() - self._hold_ts
                if self._hold_moving and age > _move_hold_s():
                    logger.warning("teleop command %.2fs old; stopping the dog", age)
                    self._hold_moving = False
                    self._hold_zero_left = 6
                if self._hold_moving:
                    send = True
                elif self._hold_zero_left > 0:
                    self._hold_zero_left -= 1
                    cmd = (0.0, 0.0, 0.0)
                    send = True
            if send and self.loop.is_running():
                self.loop.call_soon_threadsafe(self._publish_movement, *cmd)
            if send and any(cmd) and time.monotonic() - self._hold_log_ts > 5.0:
                self._hold_log_ts = time.monotonic()
                state = self._sport_state
                if state:
                    logger.info(
                        "teleop cmd=(%.2f %.2f %.2f) dog vel=(%.2f %.2f) yaw=%.2f gait=%s "
                        "height=%.2f foot_raise=%s mode=%s err=%s",
                        *cmd,
                        *state["vel"][:2],
                        state["yaw"] or 0.0,
                        state["gait"],
                        state["height"],
                        state["foot"],
                        state["mode"],
                        state["err"],
                    )
            self._hold_stop.wait(0.04)

'''

MOVE_OLD = """        x, y, yaw = twist.linear.x, twist.linear.y, twist.angular.z

        async def async_move() -> None:
"""
MOVE_NEW = """        x, y, yaw = twist.linear.x, twist.linear.y, twist.angular.z
        if duration <= 0 and _move_hold_s() > 0:  # DIMOS_TELEOP_HOLD_V1
            return self._hold_move(x, y, yaw)

        async def async_move() -> None:
"""

STOPMOVE_OLD = '''        """Halt the base: publish a zero twist and cancel the auto-stop timer."""
'''
STOPMOVE_NEW = '''        """Halt the base: publish a zero twist and cancel the auto-stop timer."""
        with self._hold_lock:  # DIMOS_TELEOP_HOLD_V1
            self._hold_moving = False
            self._hold_zero_left = 0
'''

STAND_OLD = """        self.standup()
        if isinstance(self.connection, UnitreeWebRTCConnection):
            # The physical robot takes about 3 s to stand before it accepts BalanceStand.
            time.sleep(3)
        self.connection.balance_stand()
"""
STAND_NEW = """        if isinstance(self.connection, UnitreeWebRTCConnection):
            self.connection.start_state_monitor()  # DIMOS_TELEOP_HOLD_V1
        stand_t0 = time.monotonic()
        self.standup()
        if isinstance(self.connection, UnitreeWebRTCConnection):
            # DIMOS_TELEOP_HOLD_V1 — BalanceStand mid-rise leaves the rear legs low.
            if not self.connection.wait_standing():
                logger.warning("dog not reported standing after 10 s; sending BalanceStand anyway")
            time.sleep(max(1.5, 3.0 - (time.monotonic() - stand_t0)))
        self.connection.balance_stand()
"""


FREEWALK_MARKER = "DIMOS_FREEWALK_V1"

# In MCF, Move is served by the FreeWalk FSM. After BalanceStand the dog stays at gait 0,
# creeps, and the SpeedLevel left by the app scales the speed down.
FW_METHODS = '''    def free_walk_mcf(self) -> Any:
        # DIMOS_FREEWALK_V1
        import os

        gait = os.environ.get("DIMOS_MCF_WALK", "FreeWalk")
        return self.publish_request(
            RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD_MCF[gait]}
        )

    def set_speed_level(self, level: int) -> Any:
        # DIMOS_FREEWALK_V1
        return self.publish_request(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD_MCF["SpeedLevel"], "parameter": {"data": level}},
        )

'''

FW_OLD = """            time.sleep(max(1.5, 3.0 - (time.monotonic() - stand_t0)))
        self.connection.balance_stand()
"""
FW_NEW = """            time.sleep(max(1.5, 3.0 - (time.monotonic() - stand_t0)))
        if not self._enter_free_walk():  # DIMOS_FREEWALK_V1
            self.connection.balance_stand()
"""

FW_HELPER = '''    def _enter_free_walk(self) -> bool:
        # DIMOS_FREEWALK_V1 — FreeWalk, then SpeedLevel; False means fall back to BalanceStand.
        import os

        if os.environ.get("DIMOS_FREEWALK") != "1":
            return False
        if not isinstance(self.connection, UnitreeWebRTCConnection):
            return False

        def code(resp: Any) -> Any:
            try:
                return resp["data"]["header"]["status"]["code"]
            except (KeyError, TypeError):
                return None

        resp = self.connection.free_walk_mcf()
        logger.info("FreeWalk status=%s", code(resp))
        if code(resp) != 0:
            return False
        time.sleep(2.0)
        level = int(os.environ.get("DIMOS_SPEED_LEVEL", "1"))
        logger.info("SpeedLevel(%s) status=%s", level, code(self.connection.set_speed_level(level)))
        return True

'''


def edit(path: Path, pairs: list[tuple[str, str]], marker: str = MARKER) -> int:
    if not path.is_file():
        print(f"not found: {path}", file=sys.stderr)
        return 1
    text = path.read_text(encoding="utf-8")
    if marker in text:
        print(f"already patched: {path.name} {marker}")
        return 0
    for old, new in pairs:
        if text.count(old) != 1:
            print(f"{path}: anchor found {text.count(old)} times: {old[:60]!r}", file=sys.stderr)
            return 1
        text = text.replace(old, new, 1)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path} -> {marker}")
    return 0


def main() -> int:
    default = (
        Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    rc = edit(
        site / "dimos/robot/unitree/connection.py",
        [
            ("def _walk_mps() -> float:\n", HOLD_FN + "def _walk_mps() -> float:\n"),
            (INIT_OLD, INIT_NEW),
            (STOP_OLD, STOP_NEW),
            ("    def move(self, twist: Twist, duration: float = 0.0) -> bool:\n", METHODS
             + "    def move(self, twist: Twist, duration: float = 0.0) -> bool:\n"),
            (MOVE_OLD, MOVE_NEW),
            (STOPMOVE_OLD, STOPMOVE_NEW),
        ],
    )
    if rc:
        return rc
    rc = edit(site / "dimos/robot/unitree/go2/connection.py", [(STAND_OLD, STAND_NEW)])
    if rc:
        return rc
    rc = edit(
        site / "dimos/robot/unitree/connection.py",
        [("    def start_state_monitor(self) -> None:\n", FW_METHODS + "    def start_state_monitor(self) -> None:\n")],
        FREEWALK_MARKER,
    )
    if rc:
        return rc
    return edit(
        site / "dimos/robot/unitree/go2/connection.py",
        [(FW_OLD, FW_NEW), ("    @rpc\n    def stop(self) -> None:\n", FW_HELPER + "    @rpc\n    def stop(self) -> None:\n")],
        FREEWALK_MARKER,
    )


if __name__ == "__main__":
    raise SystemExit(main())
