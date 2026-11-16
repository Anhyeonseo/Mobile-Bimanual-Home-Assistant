import json
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/run'))
from live_map_localization import LiveEncoder,LogFollower
from base_teleop_web import localization_status
from home_robot_tasks.s4_recorded_mapping import encoder_path
from test_s4_base_preparation import teleop_capture


def geometry():return json.loads((ROOT/'config/home_assistant/s4_base.nominal.json').read_text())


def test_incremental_positions_match_completed_record_and_command_alone_is_not_motion():
    rows=teleop_capture();expected,_=encoder_path(rows,geometry());live=LiveEncoder(geometry())
    for row in rows[:-1]:live.consume(row)
    np.testing.assert_allclose(live.rows,expected,atol=1e-12)
    last=live.last_feedback
    assert live.valid(last+.1)
    assert not live.valid(last+.36)
    before=live.matrix.copy()
    live.consume(dict(event='command',host_monotonic_ns=int((last+.1)*1e9),session=123,sequence=41,motion='F'))
    np.testing.assert_array_equal(live.matrix,before)
    assert live.last_feedback==last


def test_stopped_prior_requires_zero_off_and_current_hold_and_rejects_bad_sequences():
    live=LiveEncoder(geometry())
    stop=dict(armed=False,fault=False,stop_confirmed=True,hold_confirmed=True,
              wheels=[dict(id=i,goal_raw=0,torque_raw=0,position_raw=2048) for i in (7,8,9)])
    live.consume(dict(event='stationary_baseline',host_monotonic_ns=1_000_000_000,stop=stop))
    assert live.valid(10) and live.stationary
    live.append(10)
    assert live.rows[-1]==[10,0,0,0]
    stop['wheels'][0]['torque_raw']=1
    with pytest.raises(ValueError,match='baseline'):
        live.consume(dict(event='stationary_baseline',host_monotonic_ns=11_000_000_000,stop=stop))
    other=LiveEncoder(geometry());rows=teleop_capture()
    for row in rows[:3]:other.consume(row)
    bad=json.loads(json.dumps(rows[3]));bad['reply']['session']=999
    with pytest.raises(ValueError,match='mismatch'):other.consume(bad)


def test_log_follower_ignores_partial_lines_and_detects_truncation(tmp_path):
    path=tmp_path/'log';rows=teleop_capture()
    path.write_text(json.dumps(rows[0])+'\n')
    live=LiveEncoder(geometry());f=LogFollower(path,live)
    with path.open('a') as stream:stream.write(json.dumps(rows[1]))
    f.poll();assert not live.active
    with path.open('a') as stream:stream.write('\n')
    f.poll();assert live.active
    path.write_text('')
    with pytest.raises(ValueError,match='truncated'):f.poll()


@pytest.mark.parametrize('change',[{'teleop_record':'old'},{'map_sha256':'other'},
    {'host_monotonic_s':8.},{'host_monotonic_s':11.},{'host_monotonic_s':float('nan')},
    {'ready':'true'}])
def test_web_never_uses_old_or_cross_session_localization(tmp_path,change):
    path=tmp_path/'status.json'
    value=dict(schema_version=1,teleop_record='current',map_sha256='map',host_monotonic_s=9.9,ready=True)
    path.write_text(json.dumps(value));assert localization_status(path,'current','map',10)['ready']
    value.update(change);path.write_text(json.dumps(value))
    assert not localization_status(path,'current','map',10)['ready']


def test_missing_localization_cannot_arm_even_with_forged_start_action(tmp_path,monkeypatch):
    import base_teleop_web as web
    class Control(web.Control):
        calls=0
        def take_action(self,now):
            self.calls+=1
            return 'start' if self.calls==1 else 'finish'
    class Server:
        server_port=1234
        def __init__(self,*args):pass
        def serve_forever(self):pass
        def shutdown(self):pass
        def server_close(self):pass
    monkeypatch.setattr(web,'Control',Control);monkeypatch.setattr(web,'ThreadingHTTPServer',Server)
    monkeypatch.setattr(web,'inspect',lambda port:dict(hold_setup=True,wheel_feedback=True))
    monkeypatch.setattr(web,'stop',lambda port:dict(stop_confirmed=True,hold_confirmed=True))
    monkeypatch.setattr(web,'send',lambda *args:pytest.fail('must not ARM while localization unavailable'))
    monkeypatch.setattr(web.time,'sleep',lambda _:None)
    map_path=tmp_path/'map.json';map_path.write_text('{}')
    web.run_web(None,record_dir=tmp_path/'records',localization_path=tmp_path/'missing',map_path=map_path)
    records=[json.loads(row) for row in next((tmp_path/'records').glob('*.jsonl')).read_text().splitlines()]
    assert not any(r['event']=='arm_requested' for r in records)
    assert records[-1]['wheel_session_started'] is False


@pytest.mark.parametrize('ack_failure',[False,True])
def test_manual_localization_warning_does_not_pause_but_ack_failure_still_stops(tmp_path,monkeypatch,ack_failure):
    import base_teleop_web as web
    sent=[]
    class Control(web.Control):
        calls=0
        def take_action(self,now):
            self.calls+=1
            return {1:'start',4:'finish'}.get(self.calls)
    class Server:
        server_port=1234
        def __init__(self,*args):pass
        def serve_forever(self):pass
        def shutdown(self):pass
        def server_close(self):pass
    calls=[0]
    def status(*args):
        calls[0]+=1
        return dict(ready=calls[0]<=3)
    def stop(port):
        sent.append('STOP')
        return dict(armed=False,fault=False,stop_confirmed=True,hold_confirmed=True)
    monkeypatch.setattr(web,'Control',Control);monkeypatch.setattr(web,'ThreadingHTTPServer',Server)
    monkeypatch.setattr(web,'localization_status',status)
    monkeypatch.setattr(web,'inspect',lambda port:dict(hold_setup=False,wheel_feedback=False))
    monkeypatch.setattr(web,'stop',stop)
    monkeypatch.setattr(web,'send',lambda port,data:sent.append(data.decode().split()[0]))
    monkeypatch.setattr(web,'receive',lambda *args:dict(type='ack',fault=False))
    def validate(*args):
        if ack_failure and sent.count('D')==2:raise RuntimeError('ack missing')
    monkeypatch.setattr(web,'validate',validate)
    monkeypatch.setattr(web.time,'sleep',lambda _:None)
    map_path=tmp_path/'map.json';map_path.write_text('{}')
    def run():web.run_web(None,record_dir=tmp_path/'records',localization_path=tmp_path/'status',map_path=map_path)
    if ack_failure:
        with pytest.raises(RuntimeError,match='ack missing'):run()
        assert sent==['STOP','ARM','D','D','STOP']
    else:
        run()
        assert sent==['STOP','ARM','D','D','D','STOP']
    records=[json.loads(row) for row in next((tmp_path/'records').glob('*.jsonl')).read_text().splitlines()]
    assert not any(r['event']=='paused' for r in records)
    changes=[r for r in records if r['event']=='localization_changed']
    assert [r['localization']['ready'] for r in changes]==[True,False]
    assert changes[-1]['policy']=='manual_advisory_after_start'
    assert records[-1]['stop']['stop_confirmed']


def test_fresh_arm_positions_bridge_idle_to_first_feedback_without_grace():
    live=LiveEncoder(geometry())
    wheels=[dict(id=i,goal_raw=0,torque_raw=0,position_raw=2048) for i in (7,8,9)]
    live.consume(dict(event='stationary_baseline',host_monotonic_ns=1_000_000_000,
        stop=dict(armed=False,fault=False,stop_confirmed=True,hold_confirmed=True,wheels=wheels)))
    live.append(20.)
    live.consume(dict(event='armed',host_monotonic_ns=21_000_000_000,
        reply=dict(session=42,wheels=wheels)))
    assert live.valid(21.05)
    assert not live.stationary
    assert not live.valid(21.36)
    np.testing.assert_array_equal(live.matrix,np.eye(3))
