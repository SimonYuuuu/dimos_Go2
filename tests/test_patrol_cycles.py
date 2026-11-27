import contextlib
import io
import json
import os
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "simulation_legacy"))
import waypoint_patrol as patrol  # noqa: E402
from dimos_lcm.std_msgs import Bool  # noqa: E402


class FakeTransport:
    instances = {}
    goals = []
    costmap_available = True
    n = 0
    delay_s = 0.0
    delay_on = 0

    def __init__(self, topic, _message_type):
        self.topic = topic
        self.callbacks = []
        FakeTransport.instances[topic] = self

    def subscribe(self, callback):
        self.callbacks.append(callback)
        if self.topic == "global_costmap" and FakeTransport.costmap_available:
            callback(SimpleNamespace(width=10, height=10, grid=SimpleNamespace(size=100)))
        return callback

    def broadcast(self, _channel, message):
        if self.topic == "goal_request":
            FakeTransport.n += 1
            position = message.position
            FakeTransport.goals.append((round(float(position.x), 2), round(float(position.y), 2)))
            if FakeTransport.delay_s and FakeTransport.n == FakeTransport.delay_on:
                time.sleep(FakeTransport.delay_s)
            for callback in FakeTransport.instances["goal_reached"].callbacks:
                callback(Bool(data=True))


class PatrolCycleTests(unittest.TestCase):
    def run_patrol(self, repeat_enabled, costmap_available=True):
        FakeTransport.instances = {}
        FakeTransport.goals = []
        FakeTransport.costmap_available = costmap_available
        FakeTransport.n = 0
        FakeTransport.delay_s = 0.0
        FakeTransport.delay_on = 0
        with tempfile.TemporaryDirectory() as temp:
            control = Path(temp) / "control.json"
            status = Path(temp) / "status.json"
            control.write_text(json.dumps({"paused": False, "stop_requested": False}))
            with patch.object(patrol, "make_transport", FakeTransport), contextlib.redirect_stdout(io.StringIO()):
                patrol.run_patrol(
                    [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
                    (0.0, 0.0),
                    1.5 if repeat_enabled else 5.0,
                    0.02,
                    status,
                    None,
                    control,
                    repeat_enabled,
                )
            return list(FakeTransport.goals), json.loads(status.read_text())

    def test_continuous_mode_runs_multiple_cycles_without_inter_cycle_home(self):
        goals, status = self.run_patrol(repeat_enabled=True)
        self.assertEqual(goals[0], (0.0, 0.0))
        self.assertEqual(goals[-1], (0.0, 0.0))
        self.assertEqual(goals.count((0.0, 0.0)), 2)
        self.assertGreaterEqual(len(goals), 8)
        self.assertEqual(status["phase"], "completed")

    def test_single_mode_runs_one_cycle_then_returns_home(self):
        goals, status = self.run_patrol(repeat_enabled=False)
        self.assertEqual(goals, [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (0.0, 0.0)])
        self.assertEqual(status["phase"], "completed")

    def test_missing_costmap_sends_no_navigation_goal(self):
        with patch.object(patrol, "COSTMAP_TIMEOUT_S", 0.02):
            goals, status = self.run_patrol(repeat_enabled=False, costmap_available=False)
        self.assertEqual(goals, [])
        self.assertEqual(status["phase"], "error")
        self.assertIn("No valid /global_costmap", status["message"])

    def test_duration_finishes_the_open_round_before_going_home(self):
        FakeTransport.instances = {}
        FakeTransport.goals = []
        FakeTransport.costmap_available = True
        FakeTransport.n = 0
        FakeTransport.delay_s = 0.25
        FakeTransport.delay_on = 2
        with tempfile.TemporaryDirectory() as temp:
            control = Path(temp) / "control.json"
            status_path = Path(temp) / "status.json"
            control.write_text(json.dumps({"paused": False, "stop_requested": False}))
            with patch.object(patrol, "make_transport", FakeTransport), contextlib.redirect_stdout(io.StringIO()):
                patrol.run_patrol(
                    [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
                    (0.0, 0.0),
                    0.05,
                    0.0,
                    status_path,
                    None,
                    control,
                    True,
                )
            goals = list(FakeTransport.goals)
            status = json.loads(status_path.read_text())
        self.assertEqual(goals, [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (0.0, 0.0)])
        self.assertEqual(status["phase"], "completed")
        self.assertIn("本圈", status["message"])

    def test_low_battery_returns_home_without_more_waypoints(self):
        with tempfile.TemporaryDirectory() as temp:
            battery = Path(temp) / "battery.json"
            battery.write_text(json.dumps({"soc": 4, "t": time.time()}))
            control = Path(temp) / "control.json"
            status_path = Path(temp) / "status.json"
            control.write_text(json.dumps({"paused": False, "stop_requested": False}))
            FakeTransport.instances = {}
            FakeTransport.goals = []
            FakeTransport.costmap_available = True
            FakeTransport.n = 0
            FakeTransport.delay_s = 0.0
            with patch.dict(os.environ, {"PATROL_BATTERY_FILE": str(battery)}), patch.object(
                patrol, "make_transport", FakeTransport
            ), contextlib.redirect_stdout(io.StringIO()):
                patrol.run_patrol(
                    [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
                    (0.0, 0.0),
                    30.0,
                    0.0,
                    status_path,
                    None,
                    control,
                    False,
                )
            goals = list(FakeTransport.goals)
            status = json.loads(status_path.read_text())
        self.assertEqual(goals, [(0.0, 0.0), (0.0, 0.0)])
        self.assertIn("5%", status["message"])


if __name__ == "__main__":
    unittest.main()
