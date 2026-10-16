import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import sys
PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "robot_agent"))
import agent  # noqa: E402


class RemoteAgentFireSignalTests(unittest.TestCase):
    def test_fire_alarm_and_safe_are_timestamped_and_latched(self):
        with tempfile.TemporaryDirectory() as temp:
            control = Path(temp) / "control.json"
            events = Path(temp) / "events.jsonl"
            with (
                patch.object(agent, "TOKEN", "unit-test-token"),
                patch.object(agent, "CONTROL_FILE", control),
                patch.object(agent, "EVENTS_FILE", events),
            ):
                client = agent.app.test_client()
                headers = {"Authorization": "Bearer unit-test-token"}
                denied = client.post("/api/v1/fire-signal", json={"state": "Fire_alarm"})
                self.assertEqual(denied.status_code, 401)
                alarm = client.post(
                    "/api/v1/fire-signal",
                    headers=headers,
                    json={"state": "Fire_alarm", "source": "thermal-camera"},
                )
                self.assertEqual(alarm.status_code, 200)
                self.assertTrue(json.loads(control.read_text())["paused"])
                safe = client.post(
                    "/api/v1/fire-signal",
                    headers=headers,
                    json={"state": "Safe", "source": "operator-confirmation"},
                )
                self.assertEqual(safe.status_code, 200)
                state = json.loads(control.read_text())
                self.assertFalse(state["paused"])
                self.assertEqual(state["fire_state"], "Safe")
                records = [json.loads(line) for line in events.read_text().splitlines()]
                self.assertEqual([record["state"] for record in records], ["Fire_alarm", "Safe"])
                self.assertTrue(all(record["timestamp"] for record in records))

    def test_invalid_alarm_state_is_rejected(self):
        with patch.object(agent, "TOKEN", "unit-test-token"):
            response = agent.app.test_client().post(
                "/api/v1/fire-signal",
                headers={"Authorization": "Bearer unit-test-token"},
                json={"state": "unknown"},
            )
        self.assertEqual(response.status_code, 400)


if __name__ == "__main__":
    unittest.main()
