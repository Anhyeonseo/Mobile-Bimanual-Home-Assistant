"""Synthetic ROS Action servers; no hardware, camera or model imports."""
import json
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from home_task_interfaces.action import ExecuteSkill
from home_task_interfaces.srv import ConfirmStop
from home_robot_tasks.execution import invariants
from home_robot_tasks.replay import _seconds
from .core import STAGES
from .ros_adapter import ACTION_NAMES


class MockSkills(Node):
    def __init__(self):
        super().__init__("mock_task_skills")
        self.duration = _seconds(self.declare_parameter("duration_s", 0.4).value, "duration_s")
        self.stop_delay = _seconds(self.declare_parameter("stop_delay_s", 0.2).value, "stop_delay_s")
        self.never_stop = self.declare_parameter("never_confirm_stop", False).value
        self.fail_skill = self.declare_parameter("fail_skill", "").value
        self.reject_skill = self.declare_parameter("reject_skill", "").value
        self.silent = self.declare_parameter("silent_feedback", False).value
        self.lock = threading.RLock()
        self.active = None
        self.finished_at = {}
        self.group = ReentrantCallbackGroup()
        self.servers = [ActionServer(
            self, ExecuteSkill, name,
            execute_callback=self.execute,
            goal_callback=lambda goal, expected=skill: self.accept(goal, expected),
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        ) for skill, name in ACTION_NAMES.items()]
        self.create_service(ConfirmStop, "task_skills/confirm_stop", self.confirm_stop,
                            callback_group=self.group)
        self.get_logger().info("Synthetic skill actions ready; no hardware connected")

    def accept(self, goal, expected):
        with self.lock:
            try:
                params = json.loads(goal.parameters_json)
                if (not goal.run_id or goal.goal_id != goal.run_id + "/" +
                        next(s for s, k, _ in STAGES if k == expected)):
                    return GoalResponse.REJECT
                if goal.skill != expected or not isinstance(params.get("target"), dict):
                    return GoalResponse.REJECT
                if expected in {"align_for_pick", "pick", "verify_grasp"} and not params.get("context", {}).get("object_pose"):
                    return GoalResponse.REJECT
            except (ValueError, AttributeError, StopIteration):
                return GoalResponse.REJECT
            if self.active is not None or expected == self.reject_skill:
                return GoalResponse.REJECT
            self.active = goal.goal_id
            return GoalResponse.ACCEPT

    def execute(self, handle):
        goal = handle.request
        success_key = next(e for _, k, e in STAGES if k == goal.skill)
        evidence = {key: True for key in (*invariants(goal.skill), success_key)}
        began = time.monotonic()
        result = ExecuteSkill.Result(goal_id=goal.goal_id, simulated=True,
                                     success=False, conditions_json=json.dumps(evidence), output_json="{}")
        try:
            while rclpy.ok():
                if handle.is_cancel_requested:
                    result.reason = "cancelled"
                    handle.canceled()
                    return result
                if time.monotonic() - began >= self.duration:
                    if goal.skill == self.fail_skill:
                        result.reason = "injected_failure"
                        handle.abort()
                    else:
                        result.success = True
                        if goal.skill == "localize_object":
                            result.output_json = json.dumps({"object_pose": {
                                "frame_id": "mock_base_link", "position": [0.4, 0.0, 0.2],
                                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0], "simulated": True}})
                        handle.succeed()
                    return result
                if not self.silent:
                    handle.publish_feedback(ExecuteSkill.Feedback(
                        goal_id=goal.goal_id, simulated=True,
                        conditions_json=json.dumps(evidence)))
                time.sleep(0.03)
            result.reason = "server_shutdown"
            if handle.is_active:
                handle.abort()
            return result
        finally:
            with self.lock:
                self.active = None
                self.finished_at[goal.run_id] = time.monotonic()
                # Stop queries for old tasks need no timestamp after the delay;
                # bound diagnostic state independently of incoming run IDs.
                if len(self.finished_at) > 256:
                    self.finished_at.pop(next(iter(self.finished_at)))

    def confirm_stop(self, request, response):
        with self.lock:
            response.run_id, response.simulated = request.run_id, True
            # Idle includes rejection or a task that timed out before dispatch.
            stopped = (self.active is None and not self.never_stop and
                       time.monotonic() - self.finished_at.get(request.run_id, 0) >= self.stop_delay)
            response.stopped = stopped
            response.conditions_json = json.dumps({key: stopped for key in (
                "base_stopped", "lift_stopped", "arms_holding", "load_retained")})
            return response


def main(args=None):
    rclpy.init(args=args)
    node = MockSkills()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        executor.shutdown(timeout_sec=2)
        node.destroy_node()
