"""Keep the task lease until fresh MCU whole-stop evidence is available."""
from .execution import Feedback


class SystemStopBackend:
    def __init__(self, client, lease=None):
        self.client = client
        self.lease = lease
        self.run_id = None
        self.requested = False

    def _owner(self, goal_id):
        if self.lease is None:
            return goal_id
        owner = self.lease.owner
        if owner is None or not isinstance(goal_id, str) or not (
            goal_id == owner or goal_id.startswith(owner + "/")
        ):
            raise RuntimeError("stop caller does not belong to the active task")
        return owner

    def request(self, run_id, preserve_load, now_s):
        owner = self._owner(run_id)
        if self.run_id is not None and self.run_id != owner:
            raise RuntimeError("stop already owned by another task")
        self.run_id = owner
        if not self.requested:
            self.client.request()
            self.requested = True
        # MCU always requires load retention, also when caller is more permissive.

    def poll(self, run_id, now_s):
        if self.run_id != self._owner(run_id):
            raise RuntimeError("stop ownership mismatch")
        reply, observed = self.client.status()
        return Feedback(run_id, observed, "STOPPED" if reply.confirmed else "RUNNING", {
            "base_stopped": reply.confirmed and bool(reply.flags & 16),
            "lift_stopped": reply.confirmed and bool(reply.flags & 32),
            "arms_holding": reply.confirmed and bool(reply.flags & 64),
            "load_retained": reply.confirmed and bool(reply.flags & 128),
        })
