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
        goal = self._goal(parameters)
        self._send(goal_id, goal)

    def _goal(self, parameters):
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
        return goal

    def _send(self, goal_id, goal):
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
                        state['error_code']=getattr(value.result,'error_code',0)
                        state['error_message']=getattr(value.result,'error_msg','')
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


class Nav2FollowPathPort(Nav2Port):
    """Send the checked SE(2) alignment path to the existing Nav2 controller."""
    def __init__(self,node,action_name="/offline_nav2/follow_path",*,controller_id="FollowPath"):
        from rclpy.action import ActionClient
        from nav2_msgs.action import FollowPath
        self.node,self.action_type=node,FollowPath
        self.client=ActionClient(node,FollowPath,action_name)
        self.entries={};self.controller_id=controller_id

    def _goal(self,parameters):
        from geometry_msgs.msg import PoseStamped
        poses=parameters['poses_xy_yaw']
        if not 2<=len(poses)<=20000:raise ValueError('bounded checked alignment path required')
        goal=self.action_type.Goal()
        goal.controller_id=self.controller_id
        goal.path.header.frame_id=parameters['frame_id']
        goal.path.header.stamp=self.node.get_clock().now().to_msg()
        for x,y,yaw in poses:
            if not all(math.isfinite(v) for v in (x,y,yaw)):raise ValueError('finite path required')
            p=PoseStamped();p.header=goal.path.header
            p.pose.position.x=float(x);p.pose.position.y=float(y)
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
            goal.path.poses.append(p)
        return goal


class Nav2MotionPort:
    """Route transit and checked alignment through one guarded base owner."""
    def __init__(self,pose,path):
        self.pose,self.path=pose,path
        self.entries={};self.owner=None
    def ready(self):return self.owner is None and self.pose.ready() and self.path.ready()
    def start(self,g,p):
        if not self.ready() or g in self.entries:raise RuntimeError('Nav2 motion port owned or duplicate')
        selected=self.path if 'poses_xy_yaw' in p else self.pose
        self.owner=g;self.entries[g]=selected
        selected.start(g,p)
    def poll(self,g):
        result=self.entries[g].poll(g)
        if result.state in {'SUCCEEDED','FAILED','CANCELLED'}:self.owner=None
        return result
    def cancel(self,g):self.entries[g].cancel(g)
