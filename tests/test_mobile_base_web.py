"""Browser ownership, held-key heartbeat, stale-frame rejection and release behavior."""
from pathlib import Path
import sys
import pytest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/run"))
from base_teleop_web import Control, handler_for, PAGE


def controller():
    state = Control(); state.set_status(True, "ready"); assert state.claim("a" * 16, 0)
    return state


def test_hold_continues_without_repeated_keydown_until_release():
    state = controller()
    for seq in range(1, 101):
        now = seq * .1
        assert state.update("a" * 16, seq, "F", now)
        assert state.desired(now + .09) == "F"
    assert state.update("a" * 16, 101, "Z", 10.01)
    assert state.desired(10.02) == "Z"


def test_browser_loss_and_stale_frame_cannot_extend_motion():
    state = controller(); assert state.update("a" * 16, 1, "F", .1)
    assert state.desired(.36) == "Z"
    assert not state.update("a" * 16, 1, "F", .37)
    assert state.desired(.38) == "Z"
    assert not state.update("a" * 16, 2, "Z", .39)
    assert not state.update("a" * 16, 1, "F", .4)
    assert state.desired(.41) == "Z"


def test_only_one_browser_owner_and_reload_after_expiry():
    state = controller(); assert state.update("a" * 16, 1, "F", .1)
    assert not state.claim("b" * 16, .2)
    assert not state.update("b" * 16, 2, "F", .2)
    assert state.claim("b" * 16, 2.1)
    assert state.desired(2.1) == "Z"
    assert not state.update("a" * 16, 2, "F", 2.2)
    assert state.update("b" * 16, 1, "F", 2.2)


@pytest.mark.parametrize("seq,motion", [(True, "F"), (-1, "F"), (1, "X"), (1, None), (1, {}), (2**32, "F")])
def test_bad_browser_input_does_not_refresh_lease(seq, motion):
    state = controller(); assert not state.update("a" * 16, seq, motion, 10)
    assert state.desired(10) == "Z"


def test_firmware_fault_clears_browser_motion():
    state = controller(); state.update("a" * 16, 1, "F", .1)
    state.set_status(False, "fault"); assert state.desired(1.1) == "Z"
    assert not state.update("a" * 16, 2, "F", .2)


@pytest.mark.parametrize("failure", ["loss", "timeout"])
def test_browser_keydown_keyup_blur_touch_and_loss_events(failure):
    import shutil
    import subprocess
    node = shutil.which("node")
    if not node: pytest.skip("Node.js not available for browser event harness")
    script = PAGE.split("<script>", 1)[1].split("</script>", 1)[0]
    subprocess.run([node, str(ROOT / "tests/fixtures/base_web_events.js"), failure], input=script, text=True, check=True)


@pytest.mark.parametrize("auth,origin,expected", [
    ("", "http://pi:8765", 403), ("Bearer wrong", "http://pi:8765", 403),
    ("Bearer secret", "http://elsewhere", 403), ("Bearer secret", "http://pi:8765", 200)])
def test_http_control_requires_secret_and_same_origin(auth, origin, expected, monkeypatch):
    import base_teleop_web
    monkeypatch.setattr(base_teleop_web.time, "monotonic", lambda: .1)
    import io
    import json
    state = controller()
    handler = handler_for(state, "secret").__new__(handler_for(state, "secret"))
    body = json.dumps(dict(client="a" * 16, sequence=1, motion="F")).encode()
    handler.path = "/control"
    handler.headers = {"Authorization": auth, "Origin": origin, "Host": "pi:8765", "Content-Length": str(len(body))}
    handler.rfile = io.BytesIO(body)
    handler.connection = type("Connection", (), {"settimeout": lambda *args: None})()
    replies = []
    handler.respond = lambda code, value: replies.append(code)
    handler.do_POST()
    assert replies == [expected]
    assert state.motion == ("F" if expected == 200 else "Z")


@pytest.mark.parametrize("poll_first", [False, True])
def test_delayed_new_sequence_cannot_restart_after_expiry(poll_first):
    state = controller()
    assert state.update("a" * 16, 1, "F", .1)
    if poll_first: assert state.desired(.4) == "Z"
    assert not state.update("a" * 16, 2, "F", .41)
    assert not state.update("a" * 16, 3, "Z", .42)
    assert not state.update("a" * 16, 4, "F", .43)
    assert state.desired(.44) == "Z"
    assert not state.claim("a" * 16, 2)
    assert state.claim("b" * 16, 2)
    assert state.desired(2) == "Z"
    assert state.update("b" * 16, 1, "L", 2.1)
    assert not state.claim("a" * 16, 4)


def test_same_client_claim_cannot_reset_sequence_or_disrupt_current_motion():
    state = controller()
    assert state.update("a" * 16, 20, "F", .1)
    assert not state.claim("a" * 16, .12)
    assert state.sequence == 20 and state.desired(.13) == "F"
    assert not state.update("a" * 16, 1, "F", .14)


def test_client_history_is_bounded_and_never_reuses_retired_identity():
    state = controller()
    for i in range(256): assert state.claim(f"browser-{i:016d}", i+2)
    assert len(state.retired) == 256
    assert not state.claim("fresh-browser-id-1", 300)
    assert not state.claim("a" * 16, 300)
    assert state.desired(300) == "Z"
