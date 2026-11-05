"""Deterministic transport races that are difficult to provoke with DDS."""
from concurrent.futures import Future
import json
from types import SimpleNamespace
import unittest

try:
    from home_task_manager.ros_adapter import RosSkillAdapter
except ImportError:
    RosSkillAdapter = None


class Client:
    def __init__(self):
        self.sent = Future()
        self.stop = Future()
        self.stop_calls = 0

    def send_goal_async(self, goal, feedback_callback):
        self.goal, self.feedback = goal, feedback_callback
        return self.sent

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.stop_calls += 1
        return self.stop


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancelled = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancelled += 1
        future = Future()
        future.set_result(SimpleNamespace(goals_canceling=["ack"]))
        return future


@unittest.skipIf(RosSkillAdapter is None, "build and source ROS interfaces")
class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.client = Client()
        transport = SimpleNamespace(clients={"search": self.client}, stop_client=self.client,
                                    clock=lambda: 0.2)
        self.adapter = RosSkillAdapter(transport)
        self.step = {"skill": "search", "parameters": {}, "start_requires": []}
        self.adapter.start("run/SEARCHING", self.step, 0)
        self.handle = Handle()

    def result(self, simulated=True, goal_id="run/SEARCHING", status=5):
        self.handle.result.set_result(SimpleNamespace(status=status, result=SimpleNamespace(
            goal_id=goal_id, simulated=simulated, success=status == 4,
            conditions_json="{}", output_json="{}", reason="")))

    def confirmation(self, run_id="run", stopped=True, load=True):
        self.client.stop.set_result(SimpleNamespace(
            run_id=run_id, simulated=True, stopped=stopped,
            conditions_json=json.dumps(dict(base_stopped=True, lift_stopped=True,
                                            arms_holding=True, load_retained=load))))

    def test_cancel_before_goal_acceptance_waits_for_result_and_independent_stop(self):
        self.adapter.request_stop("run", True, 0.1)
        self.assertEqual(self.adapter.poll_stop("run", 0.1).status, "RUNNING")
        self.assertEqual(self.client.stop_calls, 0)
        self.client.sent.set_result(self.handle)
        self.assertEqual(self.handle.cancelled, 1)
        self.assertEqual(self.adapter.poll_stop("run", 0.2).status, "RUNNING")
        self.assertEqual(self.client.stop_calls, 0)
        self.result()
        self.assertEqual(self.adapter.poll_stop("run", 0.3).status, "RUNNING")
        self.assertEqual(self.client.stop_calls, 1)
        self.confirmation()
        self.assertEqual(self.adapter.poll_stop("run", 0.4).status, "STOPPED")

    def test_wrong_goal_result_cannot_release_ownership(self):
        self.client.sent.set_result(self.handle)
        self.result(goal_id="other/SEARCHING")
        self.adapter.request_stop("run", False, 0.3)
        self.assertEqual(self.adapter.poll_stop("run", 1).status, "RUNNING")
        self.assertEqual(self.client.stop_calls, 0)

    def test_physical_result_is_rejected(self):
        self.client.sent.set_result(self.handle)
        self.result(simulated=False)
        self.assertEqual(self.adapter.poll("run/SEARCHING", self.step, 0.3).status, "FAILED")

    def test_missing_load_confirmation_keeps_stopping(self):
        self.client.sent.set_result(self.handle)
        self.result()
        self.adapter.request_stop("run", True, 0.3)
        self.adapter.poll_stop("run", 0.4)
        self.confirmation(load=False)
        self.assertEqual(self.adapter.poll_stop("run", 0.5).status, "RUNNING")

    def test_wrong_run_stop_response_keeps_stopping(self):
        self.client.sent.set_result(self.handle)
        self.result()
        self.adapter.request_stop("run", False, 0.3)
        self.adapter.poll_stop("run", 0.4)
        self.confirmation(run_id="other")
        self.assertEqual(self.adapter.poll_stop("run", 0.5).status, "RUNNING")

    def test_first_feedback_timestamp_does_not_refresh_on_poll(self):
        first = self.adapter.poll("run/SEARCHING", self.step, 0.1)
        later = self.adapter.poll("run/SEARCHING", self.step, 0.8)
        self.assertEqual(first.observed_s, later.observed_s)


if __name__ == "__main__":
    unittest.main()
