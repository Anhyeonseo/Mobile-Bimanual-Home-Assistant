"""Host commissioning protocol and independent nominal omni geometry check."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import pytest
from home_robot_tasks.base_motion import Wheel, OmniKinematics

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/run"))
spec = importlib.util.spec_from_file_location("base_pulse", ROOT / "tools/run/pulse_mobile_base.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
INFO = dict(firmware="mobile-wheel-pulse-v4", pulse_supported=True, base_pulse_supported=True,
    pulse_consumed=False, motion_enabled=False, ids=[7, 8, 9, 10], host_baud=921600,
    servo_baud=1000000, speed_raw=200, duration_ms=3000)


class Port:
    def __init__(self, replies): self.replies = copy.deepcopy(replies); self.writes = []
    def reset_input_buffer(self): pass
    def write(self, data): self.writes.append(data); return len(data)
    def readline(self): return json.dumps(self.replies.pop(0)).encode() + b"\n"


def replies(motion):
    letter, speeds = module.PATTERNS[motion]
    return [INFO, dict(base_result="begin", motion=letter, pulse_ok=True, stop_confirmed=True,
        cleanup_attempted=True, stage="BASE_COMPLETE_TORQUE_OFF"),
        *[dict(id=i + 7, command_raw=speed, position_before=1000, position_after=1050,
            goal_after=0, torque_after=0, speed_after=0, stop_confirmed=True) for i, speed in enumerate(speeds)],
        dict(base_result="end")]


def test_info_does_not_move():
    port = Port([INFO]); assert not module.run(port)["pulse_requested"]
    assert port.writes == [b"INFO\n"]


@pytest.mark.parametrize("change", [dict(firmware="mobile-wheel-pulse-v3"),
    dict(base_pulse_supported=False), dict(pulse_consumed=True)])
def test_wrong_or_consumed_image_never_moves(change):
    port = Port([INFO | change])
    with pytest.raises(RuntimeError): module.run(port, "forward")
    assert port.writes == [b"INFO\n"]


@pytest.mark.parametrize("motion", module.PATTERNS)
def test_group_command_and_complete_stop(motion):
    port = Port(replies(motion)); report = module.run(port, motion)
    assert report["result"]["pulse_ok"]
    assert port.writes == [b"INFO\n", f"BASE {module.PATTERNS[motion][0]}\n".encode()]


@pytest.mark.parametrize("change", [dict(id=10), dict(command_raw=200),
    dict(goal_after=200), dict(torque_after=1), dict(speed_after=1), dict(position_after=-1)])
def test_inconsistent_wheel_result_rejects_without_retry(change):
    data = replies("forward"); data[2].update(change); port = Port(data)
    with pytest.raises(RuntimeError): module.run(port, "forward")
    assert len(port.writes) == 2


def test_uncertain_group_stop_is_reported():
    data = replies("forward"); data[1].update(pulse_ok=False, stop_confirmed=False)
    data[2].update(stop_confirmed=False, goal_after=200)
    report = module.run(Port(data), "forward")
    assert "Cut motor" in report["next_action"]


@pytest.mark.parametrize("motion,twist", [("forward", (1, 0, 0)), ("backward", (-1, 0, 0)),
    ("left", (0, 1, 0)), ("right", (0, -1, 0)),
    ("rotate-left", (0, 0, 1)), ("rotate-right", (0, 0, -1))])
def test_patterns_against_existing_omni_math(motion, twist):
    # Nominal geometry; not a commissioned physical navigation profile.
    geometry = OmniKinematics(tuple(Wheel(.125 * math.cos(a), .125 * math.sin(a),
        a + math.pi / 2, .05) for a in (2 * math.pi / 3, 4 * math.pi / 3, 0)))
    rates = geometry.wheel_rates(twist)
    expected = [round(v / max(map(abs, rates)) * 200) for v in rates]
    assert module.PATTERNS[motion][1] == expected
