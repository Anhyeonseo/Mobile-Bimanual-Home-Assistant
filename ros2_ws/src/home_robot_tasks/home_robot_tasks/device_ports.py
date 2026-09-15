"""Task ports for the resident arm bridge and AL/LS lift command channel.

Explicitly prepared clients are injected; these ports never discover hardware,
change limits, auto-home, or rearm after a stop. The task monitor independently
checks collision, stationary platform, payload and arrival evidence.
"""

from .fetch import InvalidTask
from .navigation import number
from .skill_ports import PortResult


class LiftPort:
    def __init__(self, client, clock, stop_backend):
        self.client, self.clock, self.stop = client, clock, stop_backend
        self.entries = {}
        self.owner = None

    def ready(self):
        return self.owner is None and self.client.mobile.boot_id is not None

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("lift owned or duplicate")
        if set(parameters) != {"height_um"} or type(parameters["height_um"]) is not int:
            raise InvalidTask("explicit integer height required")
        status = self.client.status()
        if status.session == 0xFFFFFFFF:
            raise InvalidTask("lift session exhausted")
        if not status.flags & 1:
            raise InvalidTask("explicit commissioning homing required")
        self.owner = goal_id
        self.entries[goal_id] = {
            "status": "RUNNING",
            "reason": "",
            "cancel": False,
            "start": self.clock(),
            "height": parameters["height_um"],
        }
        try:
            self.client.move(status.session + 1, parameters["height_um"])
        except Exception as error:
            self._stop(goal_id, str(error))

    def _stop(self, goal_id, reason, cancelled=False):
        e = self.entries[goal_id]
        if e["cancel"] or e["status"] != "RUNNING":
            return
        e.update(
            cancel=True,
            reason=reason,
            destination="CANCELLED" if cancelled else "FAILED",
        )
        # Whole stop remains available when a lift ACK is lost and its client
        # has dropped the local session. It must not depend on that ACK.
        self.stop.request(goal_id, True, self.clock())

    def cancel(self, goal_id):
        self._stop(goal_id, "cancelled", True)

    def poll(self, goal_id):
        e = self.entries[goal_id]
        if e["status"] != "RUNNING":
            return PortResult(e["status"], e["reason"])
        try:
            now = number(self.clock(), "lift clock")
            if now < e["start"]:
                raise InvalidTask("lift clock moved backwards")
            if e["cancel"]:
                proof = self.stop.poll(goal_id, now)
                if proof.status == "STOPPED":
                    e["status"] = e["destination"]
                    self.owner = None
            else:
                status = self.client.status()
                if status.state == 6:
                    raise InvalidTask("lift fault")
                if status.holding() and status.target_um == e["height"]:
                    e["status"] = "SUCCEEDED"
                    self.owner = None
                elif status.flags & 2:
                    self.client.keepalive()
                else:
                    raise InvalidTask("lift lost command ownership")
        except Exception as error:
            self._stop(goal_id, str(error))
        return PortResult(e["status"], e["reason"])


class ResidentArmPort:
    def __init__(
        self,
        adapter,
        route_builder,
        stop_backend,
        clock,
        *,
        transport_owner="home-executor"
    ):
        self.adapter, self.build, self.stop, self.clock = (
            adapter,
            route_builder,
            stop_backend,
            clock,
        )
        self.transport_owner = transport_owner
        self.entries = {}
        self.owner = None

    def ready(self):
        return self.owner is None and self.adapter.state.value == "ready"

    def maintain(self):
        # Register with RobotRuntime even while Nav2/lift/perception owns the task.
        if self.adapter.heartbeat_required:
            self.adapter.keepalive()

    def start(self, goal_id, parameters):
        from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint

        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("arm owned or not prepared")
        snapshot = self.adapter.feedback_snapshot()
        points = tuple(self.build(parameters, snapshot))
        if not 2 <= len(points) <= 20000 or any(
            not isinstance(p, TimedJointPoint) for p in points
        ):
            raise InvalidTask("finite checked arm route required")
        self.owner = goal_id
        self.entries[goal_id] = {"status": "RUNNING", "reason": "", "cancel": False}
        try:
            self.adapter.start(self.transport_owner, points, finite=True)
        except Exception as error:
            self._stop(goal_id, str(error))

    def _stop(self, goal_id, reason, cancelled=False):
        e = self.entries[goal_id]
        if e["cancel"] or e["status"] != "RUNNING":
            return
        e.update(
            cancel=True,
            reason=reason,
            destination="CANCELLED" if cancelled else "FAILED",
        )
        self.stop.request(goal_id, True, self.clock())

    def cancel(self, goal_id):
        self._stop(goal_id, "cancelled", True)

    def poll(self, goal_id):
        e = self.entries[goal_id]
        if e["status"] != "RUNNING":
            return PortResult(e["status"], e["reason"])
        try:
            if e["cancel"]:
                if self.stop.poll(goal_id, self.clock()).status == "STOPPED":
                    e["status"] = e["destination"]
                    self.owner = None
            else:
                self.adapter.poll(self.transport_owner)
                if self.adapter.state.value == "ready":
                    e["status"] = "SUCCEEDED"
                    self.owner = None
        except Exception as error:
            self._stop(goal_id, str(error))
        return PortResult(e["status"], e["reason"])
