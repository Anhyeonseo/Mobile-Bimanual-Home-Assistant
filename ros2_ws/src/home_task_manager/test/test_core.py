import json
import unittest

from home_robot_tasks.execution import TaskLease
from home_robot_tasks.fake_robot import FakeRobot
from home_robot_tasks.fetch import InvalidTask
from home_task_manager.core import TaskManager, STAGES
from home_task_manager.demo import EXAMPLE


def request(task_id="pick-001", **changes):
    return json.dumps({**EXAMPLE, "task_id": task_id, **changes})


class CoreTests(unittest.TestCase):
    def finish(self, manager, start=0.1):
        for i in range(100):
            manager.tick(start + i / 10)
        return manager.status("pick-001")

    def test_success_and_idle(self):
        manager = TaskManager()
        self.assertEqual(manager.status()["state"], "IDLE")
        self.assertEqual(manager.submit(request(), 0)["state"], "VALIDATING")
        seen = []
        for i in range(1, 100):
            state = manager.tick(i / 10)["state"]
            if not seen or seen[-1] != state:
                seen.append(state)
        self.assertEqual(seen, [s for s, _, _ in STAGES] + ["COMPLETED", "IDLE"])
        result = manager.status("pick-001")
        self.assertEqual(result["completed_steps"], 5)
        self.assertFalse(result["control_owned"])
        self.assertEqual(result["hardware_commands"], 0)

    def test_busy_and_shared_lease(self):
        lease = TaskLease()
        manager, other = TaskManager(lease=lease), TaskManager(lease=lease)
        manager.submit(request(), 0)
        for candidate in (manager, other):
            with self.assertRaisesRegex(InvalidTask, "robot_busy"):
                candidate.submit(request("another"), 0)

    def test_idempotent_and_conflict(self):
        manager = TaskManager()
        first = manager.submit(request(), 0)
        self.assertEqual(manager.submit(request(), 0), first)
        with self.assertRaisesRegex(InvalidTask, "task_id_conflict"):
            manager.submit(request(target={"object": "cup", "location": "desk"}), 0)
        self.finish(manager)
        self.assertEqual(manager.submit(request(), 10)["state"], "COMPLETED")
        self.assertEqual(len(manager.tasks), 1)

    def test_cancel_at_every_stage(self):
        for stage, _, _ in STAGES:
            with self.subTest(stage=stage):
                manager = TaskManager()
                manager.submit(request(), 0)
                for i in range(1, 100):
                    now = i / 10
                    if manager.tick(now)["state"] == stage:
                        break
                stopped = manager.cancel("pick-001", now)
                self.assertEqual(stopped["state"], "STOPPING")
                self.assertTrue(stopped["control_owned"])
                with self.assertRaisesRegex(InvalidTask, "robot_busy"):
                    manager.submit(request("next"), now)
                self.assertEqual(manager.tick(now + 0.2)["state"], "CANCELLED")
                self.assertFalse(manager.status("pick-001")["control_owned"])
                manager.submit(request("next"), now + 0.3)

    def test_unconfirmed_stop_blocks_new_work_and_late_ack_recovers(self):
        robot = FakeRobot()
        robot.never_stop = True
        manager = TaskManager(lambda: robot)
        manager.submit(request(), 0)
        manager.tick(0.1)
        manager.cancel("pick-001", 0.2)
        self.assertEqual(manager.tick(3)["state"], "STOP_UNCONFIRMED")
        with self.assertRaisesRegex(InvalidTask, "robot_busy"):
            manager.submit(request("next"), 3)
        robot.never_stop = False
        self.assertEqual(manager.tick(3.1)["state"], "CANCELLED")

    def test_failure_at_every_stage(self):
        for stage, skill, _ in STAGES:
            with self.subTest(stage=stage):
                robot = FakeRobot()
                robot.fail_skill = skill
                manager = TaskManager(lambda: robot)
                manager.submit(request(), 0)
                result = self.finish(manager)
                self.assertEqual(result["state"], "FAILED")
                self.assertEqual(result["completed_steps"], [s for s, _, _ in STAGES].index(stage))
                self.assertTrue(result["simulated_stop_confirmed"])

    def test_stale_and_wrong_goal_feedback(self):
        for fault in ("stale", "wrong"):
            robot = FakeRobot()
            manager = TaskManager(lambda: robot)
            manager.submit(request(), 0)
            manager.tick(0.1)
            if fault == "stale":
                robot.stale_by_s = 1
            else:
                robot.wrong_goal = True
            result = self.finish(manager, 0.2)
            self.assertEqual(result["state"], "FAILED")
            self.assertEqual(result["completed_steps"], 0)

    def test_missing_completion_evidence(self):
        robot = FakeRobot()
        robot.false_conditions.add("object_found")
        manager = TaskManager(lambda: robot)
        manager.submit(request(), 0)
        result = self.finish(manager)
        self.assertEqual(result["reason"], "completion_not_verified")
        self.assertEqual(result["state"], "FAILED")

    def test_timeout(self):
        manager = TaskManager(lambda: FakeRobot(duration_s=100), step_timeout_s=0.5)
        manager.submit(request(), 0)
        result = self.finish(manager)
        self.assertEqual(result["state"], "FAILED")
        self.assertEqual(result["reason"], "step_timeout")

    def test_bad_json_never_acquires_lease(self):
        invalid = ["{", "[]", "null", "x" * 16385,
                   '{"schema_version":1,"schema_version":1}',
                   request(operation="place"), request(schema_version=True),
                   request(task_id="bad/id"), request(target={"object": "remote"}),
                   request(target={"object": "", "location": "desk"}),
                   request(target={"object": "remote", "location": float("nan")})]
        manager = TaskManager()
        for raw in invalid:
            with self.subTest(raw=raw[:80]), self.assertRaises(InvalidTask):
                manager.submit(raw, 0)
            self.assertIsNone(manager.lease.owner)
            self.assertEqual(manager.status()["state"], "IDLE")

    def test_unknown_task_and_clock(self):
        manager = TaskManager()
        with self.assertRaisesRegex(InvalidTask, "unknown_task"):
            manager.cancel("missing", 0)
        with self.assertRaisesRegex(InvalidTask, "unknown_task"):
            manager.status("missing")
        manager.tick(1)
        with self.assertRaisesRegex(InvalidTask, "backwards"):
            manager.tick(0)


if __name__ == "__main__":
    unittest.main()
