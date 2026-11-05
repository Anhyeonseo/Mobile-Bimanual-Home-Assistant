import json
import unittest
import tempfile
from pathlib import Path

from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.gateway import Gateway, GatewayStore
from home_task_manager.core import TaskManager
from home_task_manager.demo import EXAMPLE
from home_task_manager.gateway_bridge import PickApplication, GatewayBridge


class GatewayTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot(stop_delay_s=0.3)
        self.manager = TaskManager(lambda: self.robot)
        self.store = GatewayStore(":memory:")
        self.gateway = Gateway(PickApplication(self.manager), "t" * 48, self.store)
        self.now = 0.0
        self.gateway.now = lambda: self.now
        self.auth = "Bearer " + "t" * 48
        self.body = {**EXAMPLE, "request_id": EXAMPLE["task_id"]}
        del self.body["task_id"]

    def tearDown(self):
        self.store.db.close()

    def permit(self):
        return self.gateway.dispatch("GET", "/v1/permit", self.auth, None)[1]["permit"]

    def submit(self, body=None):
        return self.gateway.dispatch("POST", "/v1/requests", self.auth,
                                     body or self.body, self.permit())

    def test_auth_and_fresh_permit(self):
        self.assertEqual(self.gateway.dispatch("POST", "/v1/requests", "bad", self.body)[0], 401)
        self.assertEqual(self.gateway.dispatch("POST", "/v1/requests", self.auth, self.body)[0], 409)
        permit = self.permit()
        self.now = 4
        self.assertEqual(self.gateway.dispatch("POST", "/v1/requests", self.auth, self.body, permit)[0], 409)
        self.assertFalse(self.manager.tasks)

    def test_single_executor_identity_and_busy(self):
        code, result = self.submit()
        self.assertEqual(code, 202)
        self.assertEqual(result["request_id"], "pick-001")
        self.assertIs(self.gateway.app.tasks, self.manager.tasks)
        self.assertIs(self.gateway.app.lease, self.manager.lease)
        self.assertEqual(self.gateway.dispatch("POST", "/v1/requests", self.auth, self.body)[0], 200)
        with self.assertRaisesRegex(InvalidTask, "robot_busy"):
            self.submit({**self.body, "request_id": "next"})
        self.assertEqual(len(self.manager.tasks), 1)

    def test_cancel_and_persistence(self):
        self.submit()
        code, result = self.gateway.dispatch("POST", "/v1/requests/pick-001/cancel", self.auth, {})
        self.assertEqual(code, 202)
        self.assertEqual(result["state"], "STOPPING")
        self.now = 0.4
        self.gateway.tick()
        self.assertEqual(self.store.get("pick-001")[1]["status"], "CANCELLED")
        self.assertIsNone(self.manager.lease.owner)

    def test_success_persisted_after_return_to_idle(self):
        self.submit()
        for i in range(1, 100):
            self.now = i / 10
            self.gateway.tick()
        self.assertEqual(self.manager.status()["state"], "IDLE")
        saved = self.store.get("pick-001")[1]
        self.assertEqual(saved["state"], "COMPLETED")
        self.assertEqual(saved["status"], "SUCCEEDED")
        self.assertEqual(saved["completed_steps"], 5)

    def test_invalid_request_and_unsupported_navigation(self):
        with self.assertRaises(InvalidTask):
            self.submit({**self.body, "operation": "fetch_object"})
        self.assertIsNone(self.store.get("pick-001"))
        with self.assertRaisesRegex(InvalidTask, "navigation_not_supported"):
            self.gateway.dispatch("GET", "/v1/map", self.auth, None)

    def test_fault_latch_cancels_and_blocks_admission(self):
        self.submit()
        self.gateway.latch_fault()
        self.assertEqual(self.manager.status()["state"], "STOPPING")
        self.assertEqual(self.gateway.dispatch("GET", "/v1/permit", self.auth, None)[0], 503)

    def test_restart_with_remote_action_in_flight_blocks_admission(self):
        with tempfile.TemporaryDirectory() as directory:
            database = str(Path(directory) / "tasks.db")
            token = Path(directory) / "token"
            token.write_text("t" * 48)
            store = GatewayStore(database)
            store.save(self.body, {"request_id": "pick-001", "status": "RUNNING", "state": "GRASPING"})
            store.db.close()
            bridge = GatewayBridge(TaskManager(), str(token), database, 0, 0)
            try:
                self.assertTrue(bridge.gateway.faulted)
                code, result = bridge.gateway.dispatch("GET", "/v1/requests/pick-001", self.auth, None)
                self.assertEqual(code, 200)
                self.assertEqual(result["state"], "INTERRUPTED")
                self.assertTrue(result["stop_confirmation_required"])
                self.assertEqual(bridge.gateway.dispatch("GET", "/v1/permit", self.auth, None)[0], 503)
            finally:
                bridge.close_http()
                bridge.close_store()


if __name__ == "__main__":
    unittest.main()
