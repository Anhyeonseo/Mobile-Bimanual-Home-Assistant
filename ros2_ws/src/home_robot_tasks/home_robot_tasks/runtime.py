"""Single-owner tick loop: maintain armed bridges even between task skills.

ROS owns callbacks; this scheduler runs at a configured rate on the same
serialized control worker. A deadline miss faults admission and requests whole
stop. It never automatically re-arms or restarts an interrupted task.
"""

from .navigation import number
from .fetch import InvalidTask


class RobotRuntime:
    def __init__(self, application, clock, maintenance=(), *, maximum_tick_gap_s=0.1):
        self.app, self.clock, self.maintenance = application, clock, tuple(maintenance)
        self.maximum_gap = number(maximum_tick_gap_s, "maximum control tick gap")
        if not 0 < self.maximum_gap <= 1:
            raise InvalidTask("invalid tick budget")
        self.last = None
        self.fault = None
        self.maximum_observed_gap_s = 0.0

    def tick(self):
        now = number(self.clock(), "runtime clock")
        if self.last is not None:
            gap = now - self.last
            self.maximum_observed_gap_s = max(self.maximum_observed_gap_s, gap)
            if gap < 0 or gap > self.maximum_gap:
                self.fault = "control_tick_deadline"
        self.last = now
        if self.fault is None:
            try:
                for maintain in self.maintenance:
                    maintain()
            except Exception as error:
                self.fault = f"bridge_maintenance:{error}"
        if self.fault is not None and self.app.lease.owner is not None:
            # Keep a monotonic application clock even when caller clock regresses.
            self.app.cancel(self.app.lease.owner, max(now, self.app.now))
        return self.app.tick(max(now, self.app.now))

    def submit(self, request):
        if self.fault is not None:
            raise InvalidTask("runtime fault requires explicit recovery")
        return self.app.submit(request, self.clock())
