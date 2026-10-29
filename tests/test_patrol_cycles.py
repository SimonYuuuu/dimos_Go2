import contextlib
import io
import json
import sys
import tempfile
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

    def __init__(self, topic, _message_type):
        self.topic = topic
        self.callbacks = []
        FakeTransport.instances[topic] = self

    def subscribe(self, callback):
        self.callbacks.append(callback)
        if self.topic == "/global_costmap" and FakeTransport.costmap_available:
            callback(SimpleNamespace(width=10, height=10, grid=SimpleNamespace(size=100)))
        return callback

    def broadcast(self, _channel, message):
        if self.topic == "/goal_request":
            position = message.position
            FakeTransport.goals.append((round(float(position.x), 2), round(float(position.y), 2)))
            for callback in FakeTransport.instances["/goal_reached"].callbacks:
                callback(Bool(data=True))


class PatrolCycleTests(unittest.TestCase):
    def run_patrol(self, repeat_enabled, costmap_available=True):
        FakeTransport.instances = {}
        FakeTransport.goals = []
        FakeTransport.costmap_available = costmap_available
        with tempfile.TemporaryDirectory() as temp:
            control = Path(temp) / "control.json"
            status = Path(temp) / "status.json"
            control.write_text(json.dumps({"paused": False, "stop_requested": False}))
            with patch.object(patrol, "LCMTransport", FakeTransport), contextlib.redirect_stdout(io.StringIO()):
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


if __name__ == "__main__":
    unittest.main()
