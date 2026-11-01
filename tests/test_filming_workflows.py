"""Filming-length sessions using the real host adapter and timed fake firmware.

The fake reproduces arm-only firmware's frozen feedback after finite completion;
it is not a motor/physics simulator and cannot establish physical grasp success.
"""
from contextlib import contextmanager
import sys
from types import SimpleNamespace

import pytest

from test_bimanual_stream_adapter import FakeTransport
from test_fixed_pick_demo import cli, recipe_for, settings
from so101_arm_bridge.bimanual_stream_adapter import AdapterState
from so101_arm_bridge.fixed_pick_demo import (
    FixedPickAdapter, OWNER, compose_targets, execute_leg, load_json,
)
from so101_arm_bridge.stream_protocol_v2 import StreamExecutorStateV2


class Clock:
    def __init__(self): self.now = 1.
    def __call__(self): return self.now
    def advance(self, seconds): self.now += max(seconds, .001)


class FilmingTransport(FakeTransport):
    def __init__(self, clock, home):
        super().__init__()
        self.clock = clock
        self.shadow_positions_urad = tuple(round(q * 1e6) for q in home)
        self.feedback_positions = self.shadow_positions_urad
        self.route = []
        self.horizon = None
        self.open_count = 0
        self.heartbeats = []

    def heartbeat(self):
        self.tick = round(self.clock() * 1000)
        self.heartbeats.append(self.clock())
        return super().heartbeat()

    def open_stream(self, policy):
        self.open_count += 1
        self.route = []
        self.horizon = policy.horizon_end_tick
        return super().open_stream(policy)

    def append(self, batch):
        self.route.extend(batch.samples)
        return super().append(batch)

    def get_executor_diagnostics(self):
        now = round(self.clock() * 1000)
        self.executor_state = (StreamExecutorStateV2.SUCCEEDED
            if self.horizon is not None and now >= self.horizon
            else StreamExecutorStateV2.RUNNING)
        return super().get_executor_diagnostics()

    def get_feedback_snapshot(self):
        now = round(self.clock() * 1000)
        for sample in self.route:
            if sample.apply_tick <= now:
                self.feedback_positions = sample.positions_urad
        # Arm-only resident firmware ends its reader when a finite leg ends.
        age = max(0, now - self.horizon) if self.horizon is not None else 0
        self.feedback_age_ms = (age,) * 12
        return super().get_feedback_snapshot()


def ready_at_lift(settings, clock):
    recipe = recipe_for()
    home = tuple(recipe["poses"]["home"]["positions_rad"])
    targets = compose_targets(recipe, home, *settings)
    transport = FilmingTransport(clock, home)
    adapter = FixedPickAdapter(transport, settings[0], home_reference=home, motion_authorized=True)
    adapter.prepare()
    previous = None
    for stage in ("home", "approach", "grasp", "closed", "lift"):
        execute_leg(adapter, targets[stage], settings[0], commanded_start=previous,
                    clock=clock, sleep=clock.advance)
        previous = targets[stage]
    return adapter, transport, targets


def test_filming_hold_survives_60_seconds_without_idle_feedback_reader(settings, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(cli.time, "monotonic", clock)
    adapter, transport, targets = ready_at_lift(settings, clock)
    began = clock()
    before = len(transport.appended)
    class Commands:
        def read(self, timeout=0):
            clock.advance(max(timeout, .01))
            return ["return"] if clock() - began >= 60 else []
    assert cli.wait_command(adapter, settings[0], targets["lift"], Commands(),
                            clock=clock, sleep=clock.advance) == "return"
    assert clock() - began >= 60
    assert adapter.state is AdapterState.READY and adapter.heartbeat_required
    assert transport.safe_stop_count == 0
    expected = tuple(round(q * 1e6) for q in targets["lift"])
    assert all(s.positions_urad == expected for b in transport.appended[before:] for s in b.samples)
    assert max(b-a for a, b in zip(transport.heartbeats, transport.heartbeats[1:])) < .15


@pytest.mark.parametrize("exit_kind", ["q", "eof", "interrupt", "feedback_loss", "stop_failure"])
def test_complete_shoot_return_repeat_then_shutdown(settings, tmp_path, monkeypatch, exit_kind):
    recipe = recipe_for()
    path = tmp_path / "poses.json"
    cli.save_json(path, recipe)
    clock = Clock()
    monkeypatch.setattr(cli.time, "monotonic", clock)
    transport = FilmingTransport(clock, recipe["poses"]["home"]["positions_rad"])
    @contextmanager
    def connection(_): yield transport
    monkeypatch.setattr(cli, "connection", connection)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))
    class Commands:
        started = None
        action = None
        def read(self, timeout=0):
            clock.advance(max(timeout, .01))
            if self.started is None or clock() - self.started < 2: return []
            action, self.started = self.action, None
            if action == "eof": raise EOFError("SSH disconnected")
            if action == "interrupt": raise KeyboardInterrupt()
            if action == "feedback_loss":
                transport.feedback_transport_errors_remaining = 100
                return []
            if action == "stop_failure":
                def failed_stop():
                    transport.safe_stop_count += 1
                    raise OSError("USB disconnected before stop acknowledgement")
                transport.safe_stop = failed_stop
                return ["q"]
            return [action]
        def during_motion(self):
            assert self.started is None
    commands = Commands()
    monkeypatch.setattr(cli, "TerminalCommands", lambda _: commands)
    real_leg, real_wait = cli.execute_leg, cli.wait_command
    def leg(*a, **k):
        k.update(clock=clock, sleep=clock.advance)
        return real_leg(*a, **k)
    waits = iter(["return", "run", exit_kind])
    def wait(*a, **k):
        commands.action, commands.started = next(waits), clock()
        return real_wait(*a, **k, clock=clock, sleep=clock.advance)
    monkeypatch.setattr(cli, "execute_leg", leg)
    monkeypatch.setattr(cli, "wait_command", wait)
    args = SimpleNamespace(file=path, execute=True, log=None)
    if exit_kind == "q": cli.play(args, *settings)
    else:
        with pytest.raises((EOFError, KeyboardInterrupt, RuntimeError)):
            cli.play(args, *settings)
    log = load_json(path.with_name("poses.last-run.json"))
    assert [e["stage"] for e in log["events"]] == [
        "home", "approach", "grasp", "closed", "lift", "closed", "grasp", "approach", "home",
        "approach", "grasp", "closed", "lift"]
    assert transport.safe_stop_count >= 1
    assert log["stop_latched"] is (exit_kind != "stop_failure")
    assert not log["physical_pick_verified"]
    assert transport.calls.count("enable") == 1
    # No torque-off recapture when repeating or holding a loaded object.
    assert "shadow" not in transport.calls[transport.calls.index("enable") + 1:]
