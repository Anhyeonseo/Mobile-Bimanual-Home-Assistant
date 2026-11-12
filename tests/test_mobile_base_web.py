"""Browser ownership, held-key heartbeat, stale-frame rejection and release behavior."""
from pathlib import Path
import sys
import pytest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/run"))
from base_teleop_web import Control, handler_for, PAGE


def controller():
    state = Control(); assert state.claim("a" * 16, 0); state.set_status(True, "ready")
    return state


def test_hold_continues_without_repeated_keydown_until_release():
    state = controller()
    for seq in range(1, 101):
        now = seq * .1
        assert state.update("a" * 16, seq, "F", now)
        assert state.desired(now + .09) == "F"
    assert state.update("a" * 16, 101, "Z", 10.01)
    assert state.desired(10.02) == "Z"


def test_ten_minutes_of_browser_heartbeats_have_no_session_time_limit():
    state=controller()
    now=0.0
    for seq in range(1,6001):
        now+=.15 if seq%100==0 else .1
        motion='FZLZAZBZRZDZ'[(seq//40)%12]
        assert state.update('a'*16,seq,motion,now)
        assert state.desired(now+.05)==motion
    assert now>=600 and not state.expired
    assert state.desired(now+.251)=='Z'
    assert not state.update('a'*16,6001,'F',now+.26)


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
    assert not state.claim("b" * 16, 2.1)  # active sessions cannot be stolen
    state.set_status(False, "stopped")
    assert state.claim("b" * 16, 2.1)
    assert state.desired(2.1) == "Z"
    assert not state.update("a" * 16, 2, "F", 2.2)
    assert not state.update("b" * 16, 1, "F", 2.2)
    assert state.update("b" * 16, 1, "Z", 2.2)


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
    state.set_status(False, "stopped")
    assert state.claim("b" * 16, 2)
    assert state.desired(2) == "Z"
    assert not state.update("b" * 16, 1, "L", 2.1)
    assert state.update("b" * 16, 1, "Z", 2.1)
    assert not state.claim("a" * 16, 4)


def test_same_client_claim_cannot_reset_sequence_or_disrupt_current_motion():
    state = controller()
    assert state.update("a" * 16, 20, "F", .1)
    assert not state.claim("a" * 16, .12)
    assert state.sequence == 20 and state.desired(.13) == "F"
    assert not state.update("a" * 16, 1, "F", .14)


def test_client_history_is_bounded_and_never_reuses_retired_identity():
    state = controller(); state.set_status(False, "idle")
    for i in range(256): assert state.claim(f"browser-{i:016d}", i+2)
    assert len(state.retired) == 256
    assert not state.claim("fresh-browser-id-1", 300)
    assert not state.claim("a" * 16, 300)
    assert state.desired(300) == "Z"


def test_explicit_actions_share_sequence_and_cannot_survive_expiry():
    state = Control(); assert state.claim('a'*16, 0)
    assert not state.request('a'*16, 1, 'start', .01)
    assert state.update('a'*16, 1, 'Z', .05)
    assert state.request('a'*16, 2, 'hold', .1)
    assert not state.request('a'*16, 3, 'hold', .11)
    assert not state.claim('b'*16, 2)
    assert state.take_action(.4) is None
    assert not state.request('a'*16, 4, 'start', .4)
    assert state.claim('b'*16, 2)
    state.configure(ready=True)
    assert not state.request('a'*16, 5, 'start', 2.01)
    assert state.request('b'*16, 1, 'start', 2.01)
    assert not state.update('b'*16, 1, 'F', 2.02)
    assert state.take_action(2.02) == 'start'


@pytest.mark.parametrize('scenario', ['idle_finish', 'drive_finish', 'loss', 'bad_ack', 'disk_error', 'hold'])
def test_web_lifecycle_uses_explicit_actions_and_always_cleans_up(monkeypatch, tmp_path, scenario):
    import base_teleop_web as web
    import json
    now = [0.]
    monkeypatch.setattr(web.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(web.time, 'sleep', lambda delay: now.__setitem__(0, now[0]+delay))
    calls = []
    class State(Control):
        def __init__(self):
            super().__init__(); self.claim('a'*16, 0); self.round = 0
        def take_action(self, stamp):
            self.round += 1
            if self.round == 1:
                assert not calls  # no ARM on opening the server
                if scenario == 'idle_finish': return 'finish'
                return 'hold' if scenario == 'hold' else 'start'
            if scenario == 'hold': return 'finish'
            if scenario == 'loss':
                self.configure(expired=True)
                return None
            return 'finish'
    class Server:
        server_port = 12345
        def __init__(self, *args): pass
        def serve_forever(self): pass
        def shutdown(self): pass
        def server_close(self): pass
    monkeypatch.setattr(web, 'Control', State)
    monkeypatch.setattr(web, 'ThreadingHTTPServer', Server)
    monkeypatch.setattr(web, 'inspect', lambda port: {'hold_setup': scenario=='hold'})
    monkeypatch.setattr(web, 'prepare_hold', lambda port, execute: calls.append('hold') or {'ready':True})
    monkeypatch.setattr(web, 'send', lambda port, data: calls.append(data.decode().split()))
    def receive(port, timeout):
        command = calls[-1]
        return dict(type='armed' if command[0]=='ARM' else 'ack', armed=True,
                    fault=scenario=='bad_ack' and command[0]=='D',
                    session=int(command[1],16), sequence=0 if command[0]=='ARM' else int(command[2],16))
    monkeypatch.setattr(web, 'receive', receive)
    monkeypatch.setattr(web, 'stop', lambda port: calls.append('stop') or {'stop_confirmed':True})
    if scenario == 'disk_error':
        original = web.SessionRecord.write
        def write(self, event, **data):
            if event=='command': raise OSError('disk full')
            return original(self,event,**data)
        monkeypatch.setattr(web.SessionRecord,'write',write)
    if scenario in ('bad_ack','disk_error'):
        with pytest.raises((RuntimeError,OSError)): web.run_web(None, record_dir=tmp_path)
    else:
        web.run_web(None, record_dir=tmp_path)
    lines = [json.loads(x) for x in next(tmp_path.glob('*.jsonl')).read_text().splitlines()]
    assert lines[0]['calibrated_odometry'] is False
    assert lines[-1]['event'] == 'finished' and lines[-1]['map_saved'] is False
    if scenario=='idle_finish': assert calls==[]
    elif scenario=='hold': assert calls==['hold']
    else:
        assert calls[0][0]=='ARM' and calls[-1]=='stop'
        assert lines[-1]['stop']['stop_confirmed']
    if scenario=='disk_error': assert not any(isinstance(c,list) and c[0]=='D' for c in calls)


def test_live_feedback_requires_all_ids_and_bounded_fresh_read_window():
    from base_teleop_web import validate_feedback
    samples=[dict(id=i,position_raw=12,speed_raw=32818,tick_begin_ms=0xffffffff,tick_end_ms=1) for i in (7,8,9)]
    validate_feedback({'wheel_feedback':samples})  # MCU tick wrap, raw +/-50 preserved
    for bad in (None, samples[:2], [samples[0]]*3, [dict(samples[0],tick_end_ms=999),*samples[1:]],
                [dict(samples[0],position_raw=True),*samples[1:]]):
        with pytest.raises(RuntimeError): validate_feedback({'wheel_feedback':bad})
