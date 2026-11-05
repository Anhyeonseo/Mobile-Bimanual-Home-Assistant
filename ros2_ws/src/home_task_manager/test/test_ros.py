"""Integration tests launch the installed executable and use real DDS services."""
import json
import os
import signal
import subprocess
import tempfile
import time
import unittest
import uuid
import re
import urllib.request
import urllib.error

try:
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from home_task_interfaces.srv import SubmitTask, CancelTask, GetTaskStatus
except ImportError:
    rclpy = None

from home_task_manager.demo import EXAMPLE


@unittest.skipIf(rclpy is None, "source the built ROS2 workspace first")
class RosTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.client_node = rclpy.create_node("test_client_" + uuid.uuid4().hex)
        self.process = None
        self.skills = None
        self.skill_log = tempfile.TemporaryFile(mode="w+")
        self.files = tempfile.TemporaryDirectory()
        self.log = tempfile.TemporaryFile(mode="w+")
        self.namespace = "/tm_test_" + uuid.uuid4().hex

    def tearDown(self):
        if self.process:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        self.client_node.destroy_node()
        rclpy.shutdown()
        if self.skills:
            os.killpg(self.skills.pid, signal.SIGINT)
            try:
                self.skills.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.skills.pid, signal.SIGKILL)
                self.skills.wait(timeout=5)
        self.log.close()
        self.skill_log.close()
        self.files.cleanup()

    def launch_skills(self, *parameters):
        command = ["ros2", "run", "home_task_manager", "mock_skills", "--ros-args",
                   "-r", "__ns:=" + self.namespace]
        for parameter in parameters:
            command += ["-p", parameter]
        self.skills = subprocess.Popen(command, stdout=self.skill_log, stderr=self.skill_log,
                                       start_new_session=True)

    def launch(self, *parameters):
        command = ["ros2", "run", "home_task_manager", "task_manager", "--ros-args",
                   "-r", "__ns:=" + self.namespace]
        for parameter in parameters:
            command += ["-p", parameter]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=self.log,
                                        start_new_session=True)
        self.submit = self.client_node.create_client(SubmitTask, self.namespace + "/task/submit")
        self.cancel = self.client_node.create_client(CancelTask, self.namespace + "/task/cancel")
        self.status = self.client_node.create_client(GetTaskStatus, self.namespace + "/task/get_status")
        service = self.status if "gateway_enabled:=true" in parameters else self.submit
        if not service.wait_for_service(timeout_sec=12):
            self.log.seek(0)
            self.fail("service unavailable: " + self.log.read())

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.client_node, future, timeout_sec=5)
        self.assertTrue(future.done(), "service response timeout")
        return future.result()

    def send(self, task_id="pick-001"):
        return self.call(self.submit, SubmitTask.Request(
            json_request=json.dumps({**EXAMPLE, "task_id": task_id})))

    def wait_state(self, wanted):
        end = time.monotonic() + 10
        while time.monotonic() < end:
            result = self.call(self.status, GetTaskStatus.Request(task_id="pick-001"))
            self.assertTrue(result.found)
            state = json.loads(result.status_json)
            if state["state"] == wanted:
                return state
            rclpy.spin_once(self.client_node, timeout_sec=0.05)
        self.fail(f"never reached {wanted}: {state}")

    def test_submit_status_topic_and_success(self):
        self.launch("mock_step_duration_s:=0.15")
        states = []
        self.client_node.create_subscription(
            String, self.namespace + "/task/status",
            lambda msg: states.append(json.loads(msg.data)["state"]),
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        bad = self.call(self.submit, SubmitTask.Request(json_request="{}"))
        self.assertFalse(bad.accepted)
        self.assertTrue(self.send().accepted)
        self.assertFalse(self.send("busy").accepted)
        result = self.wait_state("COMPLETED")
        self.assertEqual(result["completed_steps"], 5)
        self.assertEqual(result["hardware_commands"], 0)
        for stage in ("VALIDATING", "SEARCHING", "LOCALIZING", "ALIGNING", "GRASPING", "VERIFYING"):
            self.assertIn(stage, states)
        self.assertTrue(self.send().accepted)
        unknown = self.call(self.status, GetTaskStatus.Request(task_id="unknown"))
        self.assertFalse(unknown.found)

    def test_cancel_waits_for_stop(self):
        self.launch("mock_step_duration_s:=3.0", "mock_stop_delay_s:=0.5")
        self.assertTrue(self.send().accepted)
        self.wait_state("SEARCHING")
        response = self.call(self.cancel, CancelTask.Request(task_id="pick-001"))
        self.assertTrue(response.accepted)
        self.assertEqual(json.loads(response.status_json)["state"], "STOPPING")
        self.assertFalse(self.send("next").accepted)
        result = self.wait_state("CANCELLED")
        self.assertTrue(result["simulated_stop_confirmed"])
        self.assertTrue(self.send("next").accepted)

    def test_failure(self):
        self.launch("mock_fail_state:=GRASPING", "mock_step_duration_s:=0.1")
        self.assertTrue(self.send().accepted)
        result = self.wait_state("FAILED")
        self.assertEqual(result["completed_steps"], 3)
        self.assertTrue(result["simulated_stop_confirmed"])

    def test_missing_stop_ack(self):
        self.launch("mock_never_stop:=true", "mock_step_duration_s:=3.0")
        self.assertTrue(self.send().accepted)
        self.call(self.cancel, CancelTask.Request(task_id="pick-001"))
        result = self.wait_state("STOP_UNCONFIRMED")
        self.assertTrue(result["control_owned"])
        self.assertFalse(self.send("blocked").accepted)

    def test_action_pipeline_with_pose_transfer(self):
        self.launch_skills("duration_s:=0.15")
        self.launch("backend:=ros_action")
        self.assertTrue(self.send().accepted)
        result = self.wait_state("COMPLETED")
        self.assertEqual(result["completed_steps"], 5)
        self.assertEqual(result["hardware_commands"], 0)

    def test_action_cancel_with_delayed_stop_confirmation(self):
        self.launch_skills("duration_s:=2.0", "stop_delay_s:=0.7")
        self.launch("backend:=ros_action")
        self.assertTrue(self.send().accepted)
        self.wait_state("SEARCHING")
        response = self.call(self.cancel, CancelTask.Request(task_id="pick-001"))
        self.assertEqual(json.loads(response.status_json)["state"], "STOPPING")
        self.assertFalse(self.send("next").accepted)
        result = self.wait_state("CANCELLED")
        self.assertTrue(result["simulated_stop_confirmed"])

    def test_action_failure_and_rejection(self):
        self.launch_skills("reject_skill:=pick", "duration_s:=0.1")
        self.launch("backend:=ros_action")
        self.assertTrue(self.send().accepted)
        result = self.wait_state("FAILED")
        self.assertEqual(result["completed_steps"], 3)
        self.assertEqual(result["reason"], "action_rejected")
        self.assertTrue(result["simulated_stop_confirmed"])

    def test_action_aborted_and_stop_confirmed(self):
        self.launch_skills("fail_skill:=pick", "duration_s:=0.1")
        self.launch("backend:=ros_action")
        self.assertTrue(self.send().accepted)
        result = self.wait_state("FAILED")
        self.assertEqual(result["completed_steps"], 3)
        self.assertEqual(result["reason"], "injected_failure")
        self.assertTrue(result["simulated_stop_confirmed"])

    def test_action_feedback_silence_fails_and_stops(self):
        self.launch_skills("silent_feedback:=true", "duration_s:=3.0")
        self.launch("backend:=ros_action")
        self.assertTrue(self.send().accepted)
        result = self.wait_state("FAILED")
        self.assertIn("stale_feedback", result["reason"])
        self.assertEqual(result["completed_steps"], 0)

    def test_missing_action_and_stop_servers_retain_ownership(self):
        self.launch("backend:=ros_action", "step_timeout_s:=0.4")
        self.assertTrue(self.send().accepted)
        result = self.wait_state("STOP_UNCONFIRMED")
        self.assertEqual(result["reason"], "step_timeout")
        self.assertTrue(result["control_owned"])
        self.assertFalse(self.send("blocked").accepted)

    def test_action_unconfirmed_stop(self):
        self.launch_skills("never_confirm_stop:=true", "duration_s:=2.0")
        self.launch("backend:=ros_action")
        self.assertTrue(self.send().accepted)
        self.wait_state("SEARCHING")
        self.call(self.cancel, CancelTask.Request(task_id="pick-001"))
        result = self.wait_state("STOP_UNCONFIRMED")
        self.assertTrue(result["control_owned"])
        self.assertFalse(self.send("blocked").accepted)

    def http(self, method, path, body=None, permit=None, authorized=True):
        headers = {"Content-Type": "application/json"}
        if authorized:
            headers["Authorization"] = "Bearer " + "t" * 48
        if permit:
            headers["X-Request-Permit"] = permit
        req = urllib.request.Request(self.url + path, method=method, headers=headers,
                                     data=None if body is None else json.dumps(body).encode())
        try:
            response = urllib.request.urlopen(req, timeout=3)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            return response.status, json.load(response)

    def test_gateway_to_actions_end_to_end(self):
        token_path = os.path.join(self.files.name, "token")
        with open(token_path, "w") as stream:
            stream.write("t" * 48)
        self.launch_skills("duration_s:=0.15")
        self.launch("backend:=ros_action", "gateway_enabled:=true",
                    "gateway_token_file:=" + token_path,
                    "gateway_database:=" + os.path.join(self.files.name, "tasks.sqlite3"),
                    "gateway_port:=0")
        self.log.seek(0)
        self.url = re.search(r"Authenticated Pick Gateway: (http://127.0.0.1:\d+)", self.log.read()).group(1)
        self.assertFalse(self.submit.service_is_ready(), "unauthenticated submit must be absent")
        self.assertFalse(self.cancel.service_is_ready(), "unauthenticated cancel must be absent")
        body = {**EXAMPLE, "request_id": EXAMPLE["task_id"]}
        del body["task_id"]
        self.assertEqual(self.http("POST", "/v1/requests", body, authorized=False)[0], 401)
        self.assertEqual(self.http("POST", "/v1/requests", body)[0], 409)
        permit = self.http("GET", "/v1/permit")[1]["permit"]
        self.assertEqual(self.http("POST", "/v1/requests", body, permit)[0], 202)
        self.wait_state("COMPLETED")
        code, result = self.http("GET", "/v1/requests/pick-001")
        self.assertEqual(code, 200)
        self.assertEqual(result["status"], "SUCCEEDED")
        self.assertEqual(result["completed_steps"], 5)
        self.assertEqual(self.http("POST", "/v1/requests", body)[0], 200)
        cli = subprocess.run([
            "ros2", "run", "home_task_manager", "gateway_client", "--url", self.url,
            "--token-file", token_path, "status", "pick-001"],
            capture_output=True, text=True, timeout=10)
        self.assertEqual(cli.returncode, 0, cli.stderr)
        self.assertEqual(json.loads(cli.stdout)["state"], "COMPLETED")
        body["request_id"] = "cancel-me"
        permit = self.http("GET", "/v1/permit")[1]["permit"]
        self.assertEqual(self.http("POST", "/v1/requests", body, permit)[0], 202)
        code, result = self.http("POST", "/v1/requests/cancel-me/cancel", {})
        self.assertEqual(code, 202)
        self.assertEqual(result["state"], "STOPPING")
        end = time.monotonic() + 5
        while time.monotonic() < end:
            result = self.http("GET", "/v1/requests/cancel-me")[1]
            if result["state"] == "CANCELLED":
                break
            time.sleep(0.05)
        self.assertEqual(result["state"], "CANCELLED")


if __name__ == "__main__":
    unittest.main()
