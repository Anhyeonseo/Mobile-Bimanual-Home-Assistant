"""Fixed-pose filming workflow, malformed captures and stop-on-failure paths."""
from contextlib import contextmanager
import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/so101_arm_bridge"))
from so101_arm_bridge.bimanual_stream_adapter import AdapterState, load_operational_limits
from so101_arm_bridge.fixed_pick_demo import (
    FixedPickAdapter, MAX_CAPTURE_SPREAD, OWNER, SPEED_RAD_S, STAGES, arm_indices, capture_pose,
    check_recipe, compose_targets, execute_leg, limits_digest, load_json,
    measured_feedback, new_recipe, put_capture, smooth_leg, validate_recipe,
)

spec = importlib.util.spec_from_file_location("fixed_pick_cli", ROOT / "tools/run/fixed_pick_demo.py")
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


@pytest.fixture
def settings():
    path = ROOT / "config/bimanual_operational_limits.json"
    return load_operational_limits(path), limits_digest(path)


def recipe_for(arm="right"):
    digest = limits_digest(ROOT / "config/bimanual_operational_limits.json")
    recipe = new_recipe(arm, digest)
    indices = arm_indices(arm)
    for stage in STAGES:
        q = [0.] * 12
        q[indices[2]] = {"home": 0., "approach": .2, "grasp": .3, "closed": .3, "lift": .15}[stage]
        q[indices[-1]] = -.5 if stage in ("closed", "lift") else -.1
        recipe["poses"][stage] = dict(source="resident_torque_off_samples", sample_count=3,
            max_spread_rad=0., captured_at="synthetic-test-only", positions_rad=q)
    return recipe


@pytest.mark.parametrize("arm", ["left", "right"])
def test_roundtrip_targets_hold_other_arm_and_preserve_gripper(settings, arm):
    limits, digest = settings
    recipe = recipe_for(arm)
    other = set(range(12)) - set(arm_indices(arm))
    for stage in STAGES[1:]:
        for i in other:
            recipe["poses"][stage]["positions_rad"][i] = .1
    home = recipe["poses"]["home"]["positions_rad"]
    targets = compose_targets(recipe, home, limits, digest)
    for target in targets.values():
        assert all(target[i] == home[i] for i in other)
    indices = arm_indices(arm)
    assert all(targets["closed"][i] == targets["grasp"][i] for i in indices[:-1])
    assert targets["lift"][indices[-1]] == targets["closed"][indices[-1]]
    report = check_recipe(recipe, limits, digest)
    assert [leg["stage"] for leg in report["legs"]] == ["approach", "grasp", "closed", "lift", "closed", "grasp", "approach", "home"]
    assert not report["collision_checked"] and not report["physical_pick_verified"]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "0", 100])
def test_bad_angles_rejected_before_play(settings, bad):
    recipe = recipe_for(); recipe["poses"]["lift"]["positions_rad"][8] = bad
    with pytest.raises(ValueError):
        check_recipe(recipe, *settings)


@pytest.mark.parametrize("field,bad", [("firmware_version", 1), ("calibration_hash", 0),
    ("limits_sha256", "wrong"), ("schema_version", True), ("joint_names", ["wrong"]),
    ("record_kind", "simulation"), ("arm", "both")])
def test_wrong_profile_or_identity_rejected(settings, field, bad):
    recipe = recipe_for(); recipe[field] = bad
    with pytest.raises(ValueError):
        check_recipe(recipe, *settings)


@pytest.mark.parametrize("change", ["missing", "arm_move_while_close", "jaw_opens_on_lift", "wrong_close_direction", "not_measured", "unstable"])
def test_invalid_pick_sequence(settings, change):
    recipe = recipe_for(); poses = recipe["poses"]
    if change == "missing": del poses["grasp"]
    elif change == "arm_move_while_close": poses["closed"]["positions_rad"][8] += .1
    elif change == "jaw_opens_on_lift": poses["lift"]["positions_rad"][11] = -.1
    elif change == "wrong_close_direction":
        poses["closed"]["positions_rad"][11] = .1
        poses["lift"]["positions_rad"][11] = .1
    elif change == "not_measured": poses["home"]["source"] = "typed_angles"
    else: poses["grasp"]["max_spread_rad"] = MAX_CAPTURE_SPREAD + .001
    with pytest.raises(ValueError): check_recipe(recipe, *settings)


def test_current_home_mismatch_blocks_play_even_on_unused_arm(settings):
    recipe = recipe_for(); home = list(recipe["poses"]["home"]["positions_rad"])
    home[0] += .1
    with pytest.raises(ValueError, match="home"):
        compose_targets(recipe, home, *settings)


def test_trajectory_is_bounded_and_exact_at_end(settings):
    limits, _ = settings
    start = (0.,) * 12; target = tuple(.6 if i == 8 else 0. for i in range(12))
    points = smooth_leg(start, target, limits)
    assert points[0].positions_rad == start and points[-1].positions_rad == target
    velocities = []
    for a, b in zip(points, points[1:]):
        assert b.offset_ms > a.offset_ms and b.offset_ms % 5 == 0
        velocity = (b.positions_rad[8] - a.positions_rad[8]) / ((b.offset_ms - a.offset_ms) / 1000)
        velocities.append(velocity)
        assert 0 <= velocity <= SPEED_RAD_S
        assert all(b.positions_rad[i] == 0 for i in range(12) if i != 8)
    assert velocities[0] < .001 and velocities[-1] < .001
    assert max(abs(b - a) / .05 for a, b in zip(velocities, velocities[1:])) <= .4


def test_capture_requires_three_new_stable_samples_and_uses_measured_sample(settings):
    limits, digest = settings
    samples = iter([(0,) * 12, (1000,) * 12, (2000,) * 12])
    adapter = SimpleNamespace(heartbeat_required=False,
        refresh_unarmed_anchor=lambda: SimpleNamespace(positions_urad=next(samples)))
    record = capture_pose(adapter, limits, sleep=lambda _: None)
    assert record["sample_count"] == 3
    assert record["positions_rad"] == [.002] * 12
    recipe = put_capture(new_recipe("right", digest), "home", record, limits, digest)
    validate_recipe(recipe, limits, digest, complete=False)
    with pytest.raises(ValueError): validate_recipe(recipe, limits, digest)


@pytest.mark.parametrize("armed,moving", [(True, False), (False, True)])
def test_capture_rejects_enabled_or_unstable_arms(settings, armed, moving):
    samples = iter([(0,) * 12, (100000 if moving else 0,) * 12, (0,) * 12])
    adapter = SimpleNamespace(heartbeat_required=armed,
        refresh_unarmed_anchor=lambda: SimpleNamespace(positions_urad=next(samples)))
    with pytest.raises(ValueError): capture_pose(adapter, settings[0], sleep=lambda _: None)


@pytest.mark.parametrize("ages", [(0,) * 11, (200,) * 12, (-1,) * 12, (True,) * 12])
def test_stale_or_missing_feedback_blocks_next_leg(settings, ages):
    adapter = SimpleNamespace(feedback_snapshot=lambda: SimpleNamespace(sample_age_ms=ages, positions_urad=(0,) * 12))
    with pytest.raises(ValueError): measured_feedback(adapter, settings[0])


def test_duplicate_json_keys_rejected(tmp_path):
    path = tmp_path / "poses.json"; path.write_text('{"arm":"right","arm":"left"}')
    with pytest.raises(ValueError): load_json(path)


def test_default_play_checks_without_serial_or_motor_access(settings, tmp_path, monkeypatch, capsys):
    path = tmp_path / "poses.json"; cli.save_json(path, recipe_for())
    monkeypatch.setattr(cli, "connection", lambda *_: pytest.fail("must not access serial"))
    assert cli.main(["play", "--file", str(path)]) == 0
    assert "장치는 열지 않았습니다" in capsys.readouterr().out


@pytest.mark.parametrize("failure", [None, "motion", "disconnect"])
def test_play_waits_for_return_and_stops_on_all_exit_paths(settings, tmp_path, monkeypatch, failure):
    recipe = recipe_for()
    path = tmp_path / "poses.json"; cli.save_json(path, recipe)
    home = tuple(round(q * 1e6) for q in recipe["poses"]["home"]["positions_rad"])
    calls, user_events = [], []
    class Adapter:
        heartbeat_required = False
        def __init__(self, *_a, **_k): pass
        def prepare(self): calls.append("prepare")
        def refresh_unarmed_anchor(self): return SimpleNamespace(positions_urad=home)
        def stop(self, owner): assert owner == OWNER; calls.append("STOP")
    @contextmanager
    def connected(_args): yield object()
    monkeypatch.setattr(cli, "connection", connected)
    monkeypatch.setattr(cli, "FixedPickAdapter", Adapter)
    monkeypatch.setattr(cli, "TerminalCommands", lambda _stream: SimpleNamespace(during_motion=lambda: None))
    targets = compose_targets(recipe, tuple(q/1e6 for q in home), *settings)
    def leg(adapter, target, limits, **kwargs):
        name = next(name for name, value in targets.items() if value == target)
        calls.append(name); adapter.heartbeat_required = True
        if failure == "motion" and name == "grasp": raise RuntimeError("injected motion fault")
    commands = iter(["return", "q"])
    def wait(*_args):
        user_events.append(calls[-1])
        if failure == "disconnect": raise EOFError("SSH gone")
        return next(commands)
    monkeypatch.setattr(cli, "execute_leg", leg)
    monkeypatch.setattr(cli, "wait_command", wait)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))
    args = SimpleNamespace(file=path, execute=True, log=None)
    if failure:
        with pytest.raises((RuntimeError, EOFError)): cli.play(args, *settings)
    else:
        cli.play(args, *settings)
        assert user_events == ["lift", "home"]
        assert calls[1:-1] == ["home", "approach", "grasp", "closed", "lift", "closed", "grasp", "approach", "home"]
    assert calls[-1] == "STOP"
    log = load_json(path.with_name("poses.last-run.json"))
    assert log["stop_latched"] and not log["physical_pick_verified"]
    if failure == "motion": assert "lift" not in calls


def test_leg_needs_terminal_confirmation_and_times_out(settings):
    class Adapter:
        heartbeat_required = False
        state = AdapterState.READY
        def refresh_unarmed_anchor(self): return SimpleNamespace(positions_urad=(0,) * 12)
        def start(self, owner, points, finite): self.state = AdapterState.ACTIVE
        def poll(self, owner): pass
    ticks = iter([0, 100])
    with pytest.raises(TimeoutError):
        execute_leg(Adapter(), (0.,)*12, settings[0], clock=lambda: next(ticks), sleep=lambda _: None)


def test_new_route_uses_existing_resident_admission_and_small_step_limit(settings):
    from test_bimanual_stream_adapter import FakeTransport
    from so101_arm_bridge.bimanual_stream_adapter import ResidentBimanualStreamAdapter, ResidentStreamConfig
    limits, digest = settings
    recipe = recipe_for(); home = tuple(recipe["poses"]["home"]["positions_rad"])
    transport = FakeTransport()
    transport.shadow_positions_urad = tuple(round(q * 1e6) for q in home)
    adapter = ResidentBimanualStreamAdapter(transport, limits, motion_authorized=True,
        config=ResidentStreamConfig(maximum_step_urad_per_tick=1000))
    adapter.prepare()
    targets = compose_targets(recipe, home, limits, digest)
    adapter.start(OWNER, smooth_leg(home, targets["approach"], limits), finite=True)
    assert adapter.state is AdapterState.ACTIVE
    assert transport.appended and transport.opened.maximum_step_urad_per_tick == (1000,) * 12
    assert all(sample.positions_urad[:6] == (0,) * 6 for sample in transport.appended[0].samples)


@pytest.mark.parametrize("bad", ["drift", "stale"])
def test_hold_monitors_feedback_while_waiting_for_operator(settings, monkeypatch, bad):
    feedback = SimpleNamespace(sample_age_ms=(151 if bad == "stale" else 0,)*12,
                               positions_urad=(100000 if bad == "drift" else 0,)*12)
    calls = []
    adapter = SimpleNamespace(keepalive=lambda: calls.append("heartbeat"),
                              feedback_snapshot=lambda: feedback)
    with pytest.raises((ValueError, RuntimeError)):
        cli.wait_command(adapter, settings[0], (0.,)*12, None, clock=lambda: 0)
    assert calls == ["heartbeat"]


@pytest.mark.parametrize("arm", ["left", "right"])
def test_lift_never_relaxes_loaded_jaw_or_moves_unused_arm_on_tracking_error(settings, arm):
    recipe = recipe_for(arm)
    home = tuple(recipe["poses"]["home"]["positions_rad"])
    targets = compose_targets(recipe, home, *settings)
    before, after = targets["closed"], targets["lift"]
    indices = arm_indices(arm)
    idle = next(i for i in range(12) if i not in indices)
    measured = list(before)
    measured[indices[-1]] += .10  # jaw is in contact with the object
    measured[idle] += .02        # ordinary position tracking residual
    measured[indices[2]] -= .02
    class Adapter:
        heartbeat_required = True
        state = AdapterState.READY
        def feedback_snapshot(self):
            return SimpleNamespace(sample_age_ms=(0,)*12,
                positions_urad=tuple(round(q*1e6) for q in measured))
        def start(self, owner, points, finite): self.points = points
    adapter = Adapter()
    execute_leg(adapter, after, settings[0], commanded_start=before)
    assert all(p.positions_rad[indices[-1]] == before[indices[-1]] for p in adapter.points)
    assert all(p.positions_rad[idle] == before[idle] for p in adapter.points)
    assert adapter.points[0].positions_rad == before


def test_held_route_without_commanded_reference_is_refused(settings):
    adapter = SimpleNamespace(heartbeat_required=True,
        feedback_snapshot=lambda: SimpleNamespace(sample_age_ms=(0,)*12, positions_urad=(0,)*12),
        start=lambda *a, **k: pytest.fail("must not send a new route"))
    with pytest.raises(ValueError, match="이전 목표"):
        execute_leg(adapter, (0.,)*12, settings[0])


@pytest.mark.parametrize("late_value", [.2, .03])
def test_final_pre_arm_refresh_rejects_late_pose_change(settings, late_value):
    from test_bimanual_stream_adapter import FakeTransport
    transport = FakeTransport()
    adapter = FixedPickAdapter(transport, settings[0], home_reference=(0.,)*12, motion_authorized=True)
    adapter.prepare()
    adapter.refresh_unarmed_anchor()
    planned = smooth_leg((0.,)*12, (.1,)+(0.,)*11, settings[0])
    # Change only after all early checks. The .03 case still passes the home
    # tolerance, but is too far from the route's measured first point.
    transport.shadow_positions_urad = (round(late_value*1e6),)+(0,)*11
    with pytest.raises(ValueError): adapter.start(OWNER, planned, finite=True)
    assert "enable" not in transport.calls
    assert not any(isinstance(c, tuple) and c[0] == "arm" for c in transport.calls)


def test_existing_resident_accepts_continuous_held_commands_despite_gripper_contact(settings):
    from test_bimanual_stream_adapter import FakeTransport
    from so101_arm_bridge.bimanual_stream_adapter import PreparedBimanualState
    transport = FakeTransport(); recipe = recipe_for()
    home = tuple(recipe["poses"]["home"]["positions_rad"])
    transport.shadow_positions_urad = tuple(round(q*1e6) for q in home)
    adapter = FixedPickAdapter(transport, settings[0], home_reference=home, motion_authorized=True)
    adapter.prepare()
    targets = compose_targets(recipe, home, *settings)
    adapter.start(OWNER, smooth_leg(home, targets["closed"], settings[0]), finite=True)
    # Emulate a completed close with admissible physical contact residual.
    measured = list(targets["closed"]); measured[11] += .10
    transport.feedback_positions = tuple(round(q*1e6) for q in measured)
    adapter._state = AdapterState.READY
    adapter._prepared = PreparedBimanualState((2048,)*12, (2048,)*12, transport.feedback_positions)
    adapter.poll = lambda _owner: setattr(adapter, "_state", AdapterState.READY)
    execute_leg(adapter, targets["lift"], settings[0], commanded_start=targets["closed"])
    assert transport.appended[-1].samples[0].positions_urad[11] == -500000
    assert all(sample.positions_urad[11] == -500000 for sample in transport.appended[-1].samples)


def test_motion_cancel_is_polled_before_arming_and_during_motion(settings):
    class Adapter:
        heartbeat_required = False
        state = AdapterState.READY
        def refresh_unarmed_anchor(self): return SimpleNamespace(positions_urad=(0,)*12)
        def start(self, *a, **k): self.state = AdapterState.ACTIVE
        def poll(self, _owner): pytest.fail("q should cancel before the next controller poll")
    adapter = Adapter(); calls = []
    def interrupt():
        calls.append(1)
        if len(calls) == 3: raise cli.OperatorStop()
    with pytest.raises(cli.OperatorStop):
        execute_leg(adapter, (0.,)*12, settings[0], interrupt=interrupt)
    assert adapter.state is AdapterState.ACTIVE


def test_terminal_input_does_not_hide_a_buffered_stop_or_replay_motion_commands():
    read_fd, write_fd = os.pipe()
    try:
        commands = cli.TerminalCommands(SimpleNamespace(fileno=lambda: read_fd))
        os.write(write_fd, b"return\nrun\n")
        commands.during_motion()
        assert commands.read() == []  # no delayed action after the move
        os.write(write_fd, b"run\nq\n")
        with pytest.raises(cli.OperatorStop): commands.during_motion()
        os.write(write_fd, b"q")
        commands.during_motion()
        os.write(write_fd, b"\n")
        with pytest.raises(cli.OperatorStop): commands.during_motion()
        os.close(write_fd); write_fd = None
        with pytest.raises(EOFError): commands.read()
    finally:
        os.close(read_fd)
        if write_fd is not None: os.close(write_fd)


@pytest.mark.parametrize("exit_kind", ["q", "eof"])
def test_cancel_during_motion_stops_before_later_pick_stages(settings, tmp_path, monkeypatch, exit_kind):
    recipe = recipe_for(); path = tmp_path / "poses.json"; cli.save_json(path, recipe)
    home = tuple(round(q*1e6) for q in recipe["poses"]["home"]["positions_rad"])
    events = []
    class Adapter:
        heartbeat_required = False
        def __init__(self, *a, **k): pass
        def prepare(self): pass
        def refresh_unarmed_anchor(self): return SimpleNamespace(positions_urad=home)
        def stop(self, _owner): events.append("STOP")
    @contextmanager
    def connected(_): yield object()
    def cancel():
        raise cli.OperatorStop() if exit_kind == "q" else EOFError()
    def leg(adapter, target, limits, **kwargs):
        events.append("motion_started")
        kwargs["interrupt"]()
    monkeypatch.setattr(cli, "connection", connected)
    monkeypatch.setattr(cli, "FixedPickAdapter", Adapter)
    monkeypatch.setattr(cli, "TerminalCommands", lambda _: SimpleNamespace(during_motion=cancel))
    monkeypatch.setattr(cli, "execute_leg", leg)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))
    args = SimpleNamespace(file=path, execute=True, log=None)
    if exit_kind == "q": cli.play(args, *settings)
    else:
        with pytest.raises(EOFError): cli.play(args, *settings)
    assert events == ["motion_started", "STOP"]
