import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "remote_client"))
import console  # noqa: E402


class RemoteConsoleTests(unittest.TestCase):
    def test_saved_route_is_forwarded_to_robot_agent_as_one_shot_payload(self):
        route = {
            "return_to_start": {"x": 0.0, "y": 0.0},
            "waypoints": [{"x": 1.0, "y": 2.0, "dwell_s": 1}],
        }
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(console, "ROUTES", Path(temp)):
                client = console.app.test_client()
                saved = client.post("/api/route", json={"name": "test-route", "route": route})
                self.assertEqual(saved.status_code, 200)
                calls = []

                def fake_agent(path, method="GET", payload=None, timeout=15.0):
                    calls.append((path, method, payload))
                    return {"ok": True}, 200

                with patch.object(console, "_agent", side_effect=fake_agent):
                    response = client.post(
                        "/api/patrol/start",
                        json={
                            "map": "warehouse.pc2.lcm",
                            "route": "test-route.json",
                            "execution_mode": "immediate",
                            "max_duration_s": 120,
                            "repeat_enabled": True,
                            "repeat_interval_s": 0,
                        },
                    )
                self.assertEqual(response.status_code, 200)
                self.assertEqual(calls[0][0], "patrol/start")
                forwarded = calls[0][2]
                self.assertEqual(forwarded["route"], route)
                self.assertTrue(forwarded["repeat_enabled"])
                self.assertEqual(forwarded["repeat_interval_s"], 0)

    def test_remote_page_uses_robot_endpoints(self):
        response = console.app.test_client().get("/")
        self.assertEqual(response.status_code, 200)
        self.assertIn(b"/command-center", response.data)
        self.assertIn("持续循环至任务时长".encode(), response.data)
        self.assertIn("火情信号：NaN".encode(), response.data)
        response.close()


if __name__ == "__main__":
    unittest.main()
