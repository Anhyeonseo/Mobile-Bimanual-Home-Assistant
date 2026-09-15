"""Deterministic XY/yaw navigation plant with bounded speed/acceleration.

This is a kinematic test plant, not contact physics, wheel slip or Nav2. Fetch
skills retain the explicit synthetic FakeRobot contract. navigate_to follows the
validated map path and stop is confirmed only after velocity reaches zero.
"""

from __future__ import annotations
import math
from .fake_robot import FakeRobot
from .execution import Feedback
from .navigation import number


class KinematicRobot(FakeRobot):
    def __init__(
        self,
        initial_xy=(1.5, 1.5),
        *,
        speed_m_s=0.5,
        acceleration_m_s2=0.5,
        yaw_speed_rad_s=0.5,
    ):
        super().__init__()
        self.xy = tuple(number(v, "initial_xy") for v in initial_xy)
        self.speed = number(speed_m_s, "speed")
        self.acceleration = number(acceleration_m_s2, "acceleration")
        self.yaw_speed = number(yaw_speed_rad_s, "yaw_speed")
        if len(self.xy) != 2 or min(self.speed, self.acceleration, self.yaw_speed) <= 0:
            raise ValueError("invalid plant parameters")
        self.velocity = 0.0
        self.yaw = 0.0
        self.path = []
        self.last_update = 0.0
        self.goal_yaw = 0.0
        self.nav_active = False
        self.heading = (0.0, 0.0)

    def start(self, goal_id, step, now_s):
        super().start(goal_id, step, now_s)
        self.nav_active = step["skill"] == "navigate_to"
        self.last_update = now_s
        if self.nav_active:
            params = step["parameters"]
            self.path = [(p["x"], p["y"]) for p in params["path_xy"]]
            self.path.append((params["goal"]["x"], params["goal"]["y"]))
            self.goal_yaw = params["goal"]["yaw_rad"]

    def _advance(self, now_s, stopping=False):
        dt = now_s - self.last_update
        if dt < 0 or dt > 0.5:
            raise ValueError("plant clock gap")
        self.last_update = now_s
        # Small substeps make integration independent of client polling jitter.
        count = max(1, math.ceil(dt / 0.01))
        h = dt / count
        for _ in range(count):
            if stopping:
                previous = self.velocity
                self.velocity = max(0.0, self.velocity - self.acceleration * h)
                distance = (previous + self.velocity) * 0.5 * h
                self.xy = (
                    self.xy[0] + self.heading[0] * distance,
                    self.xy[1] + self.heading[1] * distance,
                )
                continue
            while self.path and math.dist(self.xy, self.path[0]) < 1e-8:
                self.path.pop(0)
            if self.path:
                target = self.path[0]
                distance = math.dist(self.xy, target)
                self.heading = (
                    (target[0] - self.xy[0]) / distance,
                    (target[1] - self.xy[1]) / distance,
                )
                desired = min(self.speed, math.sqrt(2 * self.acceleration * distance))
                self.velocity = max(
                    self.velocity - self.acceleration * h,
                    min(desired, self.velocity + self.acceleration * h),
                )
                moved = min(distance, self.velocity * h)
                self.xy = (
                    self.xy[0] + self.heading[0] * moved,
                    self.xy[1] + self.heading[1] * moved,
                )
                if moved == distance:
                    self.xy = target
                    self.path.pop(0)
                    self.velocity = 0.0
            else:
                self.velocity = 0.0
                error = (self.goal_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
                self.yaw += max(-self.yaw_speed * h, min(self.yaw_speed * h, error))

    def poll(self, goal_id, step, now_s):
        if not self.nav_active:
            return super().poll(goal_id, step, now_s)
        if self.active is None or self.active[0] != goal_id:
            raise ValueError("wrong navigation goal")
        if self.fail_skill == "navigate_to":
            return self._feedback(goal_id, step, now_s, "FAILED")
        self._advance(now_s)
        done = (
            not self.path
            and abs((self.goal_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi)
            < 1e-6
        )
        result = self._feedback(
            goal_id, step, now_s, "SUCCEEDED" if done else "RUNNING"
        )
        conditions = dict(result.conditions)
        conditions["base_stopped"] = (
            self.velocity == 0 and done and "base_stopped" not in self.false_conditions
        )
        conditions["localized_at_destination"] = (
            done and "localized_at_destination" not in self.false_conditions
        )
        if done:
            self.active = None
            self.nav_active = False
        return Feedback(result.goal_id, result.observed_s, result.status, conditions)

    def request_stop(self, run_id, preserve_load, now_s):
        # Do not integrate an unobserved gap as commanded movement.
        self.last_update = now_s
        super().request_stop(run_id, preserve_load, now_s)

    def poll_stop(self, run_id, now_s):
        if self.nav_active:
            self._advance(now_s, stopping=True)
            if self.velocity > 1e-8:
                return Feedback(run_id, now_s, "RUNNING", {"base_stopped": False})
            self.nav_active = False
            self.path = []
        return super().poll_stop(run_id, now_s)
