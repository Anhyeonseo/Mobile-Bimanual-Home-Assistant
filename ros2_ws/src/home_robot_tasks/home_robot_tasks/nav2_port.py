"""Nonblocking NavigateToPose action port; imports ROS only on construction.

Default namespace is offline_nav2 for PC integration tests. Spin the supplied
node in an executor; this port never calls spin/wait_for_server synchronously.
Successful action result must still pass the task's independent evidence checks.
"""

import math
from .skill_ports import PortResult


class Nav2Port:
    def __init__(self, node, action_name="/offline_nav2/navigate_to_pose"):
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose

        self.node, self.action_type = node, NavigateToPose
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.entries = {}

    def ready(self):
        return self.client.server_is_ready()

    def start(self, goal_id, parameters):
        if goal_id in self.entries:
            raise RuntimeError("duplicate navigation goal")
        if not self.ready():
            raise RuntimeError("navigation server unavailable")
        pose = parameters["goal"]
        goal = self.action_type.Goal()
        goal.pose.header.frame_id = parameters["frame_id"]
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose["x"])
        goal.pose.pose.position.y = float(pose["y"])
        # Planar Nav2 pose: display/floor height was validated before admission.
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(pose["yaw_rad"] / 2)
        goal.pose.pose.orientation.w = math.cos(pose["yaw_rad"] / 2)
        state = {"status": "RUNNING", "reason": "", "handle": None, "cancel": False}
        self.entries[goal_id] = state
        try:
            future = self.client.send_goal_async(goal)
        except Exception:
            state.update(status="FAILED", reason="nav2_send_failed")
            raise

        def accepted(done):
            try:
                handle = done.result()
                state["handle"] = handle
                if not handle.accepted:
                    state.update(status="FAILED", reason="nav2_rejected")
                    return
                result = handle.get_result_async()

                def finished(result_future):
                    try:
                        value = result_future.result()
                        if (
                            value.status == 4
                            and getattr(value.result, "error_code", 0) == 0
                        ):
                            state["status"] = "SUCCEEDED"
                        elif value.status == 5:
                            state["status"] = "CANCELLED"
                        else:
                            state.update(status="FAILED", reason="nav2_failed")
                    except Exception as error:
                        state.update(status="FAILED", reason=str(error))

                result.add_done_callback(finished)
                if state["cancel"]:
                    handle.cancel_goal_async()
            except Exception as error:
                state.update(status="FAILED", reason=str(error))

        future.add_done_callback(accepted)

    def poll(self, goal_id):
        state = self.entries[goal_id]
        return PortResult(state["status"], state["reason"])

    def cancel(self, goal_id):
        state = self.entries[goal_id]
        state["cancel"] = True
        if state["handle"] is not None and state["handle"].accepted:
            state["handle"].cancel_goal_async()
