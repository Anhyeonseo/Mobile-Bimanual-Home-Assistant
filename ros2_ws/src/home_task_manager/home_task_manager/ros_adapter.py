"""Nonblocking SkillAdapter for ROS actions and explicit system-stop evidence.

All clients are constructed on the node thread. Each task gets private state.
Callbacks and Gateway HTTP workers serialize adapter state with an RLock.
This implementation deliberately accepts simulation evidence only.
"""
import json
import threading

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from home_task_interfaces.action import ExecuteSkill
from home_task_interfaces.srv import ConfirmStop
from home_robot_tasks.execution import Feedback, invariants


ACTION_NAMES = {
    "search": "perception/find_object",
    "localize_object": "perception/localize_object",
    "align_for_pick": "navigation/align",
    "pick": "manipulation/pick",
    "verify_grasp": "manipulation/verify_grasp",
}


def conditions(raw):
    value = json.loads(raw)
    if not isinstance(value, dict) or any(type(v) is not bool for v in value.values()):
        raise ValueError("conditions must be a boolean object")
    return value


class RosSkillTransport:
    def __init__(self, node, clock):
        self.clock = clock
        self.clients = {skill: ActionClient(node, ExecuteSkill, name)
                        for skill, name in ACTION_NAMES.items()}
        self.stop_client = node.create_client(ConfirmStop, "task_skills/confirm_stop")

    def adapter(self):
        return RosSkillAdapter(self)

    def destroy(self):
        for client in self.clients.values():
            client.destroy()


class RosSkillAdapter:
    mode = "simulation"

    def __init__(self, transport):
        self.transport = transport
        self.lock = threading.RLock()
        self.entry = None
        self.stopping = False
        self.run_id = None
        self.preserve_load = False
        self.stop_future = None
        self.context = {}

    def observe(self, goal_id, step, now_s):
        # Readiness is synthetic in this prototype; action-server evidence is
        # required after dispatch. This must not be reused for physical admission.
        ready = self.transport.clients[step["skill"]].server_is_ready()
        return Feedback(goal_id, now_s, "READY" if ready else "RUNNING",
                        {key: True for key in (*step["start_requires"], *invariants(step["skill"]))})

    def start(self, goal_id, step, now_s):
        with self.lock:
            if self.stopping or (self.entry and not self.entry["terminal"]):
                raise RuntimeError("action adapter busy")
            run_id = goal_id.rsplit("/", 1)[0]
            self.run_id = run_id
            entry = dict(goal_id=goal_id, handle=None, terminal=False, rejected=False,
                         error="", feedback=None, result=None, cancel_sent=False,
                         dispatched_at=now_s)
            self.entry = entry
            goal = ExecuteSkill.Goal(run_id=run_id, goal_id=goal_id, skill=step["skill"],
                                     parameters_json=json.dumps({"target": step["parameters"],
                                                                 "context": self.context}))
            # If send raises, ownership remains uncertain: do not fabricate a
            # terminal response or allow the stop service to release the lease.
            future = self.transport.clients[step["skill"]].send_goal_async(
                goal, feedback_callback=lambda msg: self._feedback(entry, msg.feedback))
            future.add_done_callback(lambda done: self._accepted(entry, done))

    def _feedback(self, entry, message):
        with self.lock:
            if entry["terminal"]:
                return
            try:
                if message.goal_id != entry["goal_id"] or not message.simulated:
                    raise ValueError("wrong goal or non-simulation feedback")
                entry["feedback"] = (self.transport.clock(), conditions(message.conditions_json))
            except Exception as error:
                entry["error"] = str(error)

    def _accepted(self, entry, future):
        with self.lock:
            try:
                handle = future.result()
                entry["handle"] = handle
                if not handle.accepted:
                    entry.update(terminal=True, rejected=True, error="action_rejected")
                    return
                handle.get_result_async().add_done_callback(lambda done: self._finished(entry, done))
                if self.stopping:
                    self._cancel(entry)
            except Exception as error:
                entry["error"] = "action_acceptance_unknown:" + str(error)

    def _finished(self, entry, future):
        with self.lock:
            try:
                response = future.result()
                result = response.result
                if result.goal_id != entry["goal_id"] or not result.simulated:
                    raise ValueError("wrong goal or non-simulation result")
                evidence = conditions(result.conditions_json)
                output = json.loads(result.output_json)
                if not isinstance(output, dict):
                    raise ValueError("action output must be an object")
                if response.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                           GoalStatus.STATUS_CANCELED):
                    raise ValueError("nonterminal action result")
                entry["terminal"] = True
                if response.status == GoalStatus.STATUS_SUCCEEDED and result.success:
                    self.context.update(output)
                entry["result"] = (
                    self.transport.clock(),
                    "SUCCEEDED" if response.status == GoalStatus.STATUS_SUCCEEDED and result.success else "FAILED",
                    evidence, result.reason,
                )
            except Exception as error:
                entry["error"] = "action_result_unknown:" + str(error)

    def _cancel(self, entry):
        if entry["handle"] is not None and entry["handle"].accepted and not entry["terminal"] and not entry["cancel_sent"]:
            # The response is intentionally NOT interpreted as stop evidence.
            entry["handle"].cancel_goal_async()
            entry["cancel_sent"] = True

    def poll(self, goal_id, step, now_s):
        with self.lock:
            entry = self.entry
            if entry is None or entry["goal_id"] != goal_id:
                raise RuntimeError("unknown action goal")
            if entry["error"]:
                return Feedback(goal_id, now_s, "FAILED", {}, entry["error"])
            if entry["result"]:
                stamp, status, evidence, reason = entry["result"]
                return Feedback(goal_id, stamp, status, evidence, reason)
            if entry["feedback"]:
                stamp, evidence = entry["feedback"]
                return Feedback(goal_id, stamp, "RUNNING", evidence)
            # Only allow the existing 0.5s freshness budget for first feedback.
            return Feedback(goal_id, entry["dispatched_at"], "RUNNING",
                            {key: True for key in invariants(step["skill"])})

    def request_stop(self, run_id, preserve_load, now_s):
        with self.lock:
            self.stopping, self.run_id, self.preserve_load = True, run_id, preserve_load
            if self.entry:
                self._cancel(self.entry)

    def poll_stop(self, run_id, now_s):
        with self.lock:
            if not self.stopping or run_id != self.run_id:
                raise RuntimeError("wrong stop owner")
            waiting = Feedback(run_id, now_s, "RUNNING", {})
            if self.entry and not self.entry["terminal"]:
                self._cancel(self.entry)
                return waiting
            # Even terminal actions need independent, freshly requested stop
            # confirmation. In particular SUCCEEDED does not mean motors stopped.
            client = self.transport.stop_client
            if self.stop_future is not None:
                if not self.stop_future.done():
                    return waiting
                future, self.stop_future = self.stop_future, None
                try:
                    result = future.result()
                    evidence = conditions(result.conditions_json)
                    required = {"base_stopped", "lift_stopped", "arms_holding"}
                    if self.preserve_load:
                        required.add("load_retained")
                    if (result.run_id == run_id and result.simulated and result.stopped
                            and all(evidence.get(key) is True for key in required)):
                        return Feedback(run_id, now_s, "STOPPED", evidence)
                except Exception:
                    pass  # Keep ownership and retry confirmation.
            if client.service_is_ready():
                self.stop_future = client.call_async(ConfirmStop.Request(
                    run_id=run_id, preserve_load=self.preserve_load))
            return waiting
