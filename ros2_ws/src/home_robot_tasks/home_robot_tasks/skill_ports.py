"""Connect task execution to asynchronous backends and independent evidence.

Backends supply execution status; monitors supply measured conditions. An action
server reporting success cannot manufacture grasp, localization or stop proof.
"""

from dataclasses import dataclass
from .execution import Feedback


@dataclass(frozen=True)
class PortResult:
    state: str
    reason: str = ""


class RoutedSkillAdapter:
    def __init__(self, ports, monitor, stop_backend, *, mode):
        if mode not in ("simulation", "physical"):
            raise ValueError("explicit adapter mode required")
        self.ports, self.monitor, self.stop_backend, self.mode = (
            dict(ports),
            monitor,
            stop_backend,
            mode,
        )
        self.active = None

    def observe(self, goal_id, step, now_s):
        evidence = self.monitor(goal_id, step, now_s)
        port = self.ports.get(step["skill"])
        status = "READY" if port is not None and port.ready() else "RUNNING"
        return Feedback(
            evidence.goal_id, evidence.observed_s, status, evidence.conditions
        )

    def start(self, goal_id, step, now_s):
        if self.active is not None:
            raise RuntimeError("adapter already active")
        port = self.ports[step["skill"]]
        self.active = (goal_id, port)
        port.start(goal_id, step["parameters"])

    def poll(self, goal_id, step, now_s):
        if self.active is None or self.active[0] != goal_id:
            raise RuntimeError("wrong adapter goal")
        result = self.active[1].poll(goal_id)
        evidence = self.monitor(goal_id, step, now_s)
        if result.state == "SUCCEEDED":
            # Result and sensor messages can arrive in either order. Keep the
            # resource owned until independent completion evidence arrives; the
            # executor still enforces freshness, invariants and its deadline.
            if not all(
                evidence.conditions.get(key) is True
                for key in step.get("success_requires", ())
            ):
                result = PortResult("RUNNING", "awaiting_completion_evidence")
            else:
                self.active = None
        return Feedback(
            evidence.goal_id,
            evidence.observed_s,
            result.state,
            evidence.conditions,
            result.reason,
        )

    def request_stop(self, run_id, preserve_load, now_s):
        if self.active is not None:
            self.active[1].cancel(self.active[0])
        self.stop_backend.request(run_id, preserve_load, now_s)

    def poll_stop(self, run_id, now_s):
        result = self.stop_backend.poll(run_id, now_s)
        # The underlying pending action must also be cancelled/terminal. A late
        # accepted goal cannot execute after the task has released ownership.
        port_done = self.active is None or self.active[1].poll(
            self.active[0]
        ).state in ("SUCCEEDED", "FAILED", "CANCELLED")
        if result.status == "STOPPED" and port_done:
            self.active = None
            return result
        return Feedback(run_id, result.observed_s, "RUNNING", result.conditions)
