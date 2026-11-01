import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/run'))
spec=importlib.util.spec_from_file_location('measure_lift_range',ROOT/'tools/run/measure_lift_range.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
INFO=dict(firmware='mobile-bus-inspection-v2',motion_enabled=False,lift_read_supported=True,
          ids=[7,8,9,10],host_baud=921600,servo_baud=1000000)


def motor(raw, seq, **changes):
    return dict(id=10,reported_id=10,identity_ok=True,model_raw=777,baud_code=0,mode_raw=0,
                torque_raw=0,position_raw=raw,read_sequence=seq)|changes


def add(s, value, **changes):
    return s.add(motor(value%4096,s.count+1,**changes),(s.count+1)*.05)


def hold(s, value):
    for _ in range(16):add(s,value)


@pytest.mark.parametrize('direction',[1,-1])
def test_multiturn_height_scale_and_return(direction):
    s=m.LiftSweep();hold(s,4000);s.mark('low')
    for i in range(1,101):add(s,4000+direction*i*100)
    hold(s,4000+direction*10000);s.mark('high')
    for i in range(99,-1,-1):add(s,4000+direction*i*100)
    hold(s,4000);s.mark('return')
    out=s.finish()
    assert out['encoder_delta_raw']==direction*10000
    assert out['encoder_up_direction']==direction and out['return_error_raw']==0


@pytest.mark.parametrize('changes',[dict(torque_raw=1),dict(mode_raw=1),dict(identity_ok=False),
                                  dict(position_raw=4096),dict(id=9),dict(model_raw=0)])
def test_invalid_feedback_rejected(changes):
    s=m.LiftSweep()
    with pytest.raises(ValueError):s.add(motor(2000,1,**changes),1)


def test_discontinuities_and_motion_during_marks_rejected():
    for raw,seq,t in ((2048,2,1.05),(100,2,2),(100,1,1.05)):
        s=m.LiftSweep();s.add(motor(100,1),1)
        with pytest.raises(ValueError):s.add(motor(raw,seq),t)
    s=m.LiftSweep()
    for i in range(20):add(s,i*100)
    with pytest.raises(ValueError,match='still'):s.mark('low')
    with pytest.raises(ValueError):s.finish()
    with pytest.raises(ValueError):s.mark('high')


class Port:
    def __init__(self, rows):self.rows=list(rows);self.writes=[]
    def write(self,p):self.writes.append(p);return len(p)
    def readline(self):return (json.dumps(self.rows.pop(0))+'\n').encode()


def recorded_capture(tmp_path):
    values=[4000]*16+[4000+i*100 for i in range(1,101)]+[14000]*16
    rows=[motor(v%4096,i+1) for i,v in enumerate(values)]
    port=Port([INFO,*rows]);step=iter(range(1,len(rows)+1))
    actions=iter([('low' if i==16 else 'high' if i==131 else 'finish' if i==132 else None)
                  for i in range(1,len(rows)+1)])
    times=iter([0]+[i*.05 for i in range(1,len(rows)+1) for _ in range(2)])
    # Capture calls the clock once per read and once per loop timeout check.
    path=tmp_path/'capture.json'
    result=m.capture(port,path,control=lambda:next(actions),clock=lambda:next(times),sleep=lambda _:None)
    assert set(port.writes)=={b'INFO\n',b'LIFT\n'}
    assert result['motor_write_commands']==0
    return path


def test_capture_finalize_and_tampering(tmp_path):
    path=recorded_capture(tmp_path)
    output=tmp_path/'geometry.json'
    result=m.finalize(path,output,500,900,'floor to top of lift platform')
    assert result['maximum_height_um']==400000 and result['um_per_turn']==163840
    assert not result['origin_persists_after_restart'] and not result['hardware_enabled']
    assert not result['physical_motion_test_verified']
    with pytest.raises(FileExistsError):m.finalize(path,output,500,900,'floor')
    path.with_suffix('.jsonl').write_text('{}\n')
    with pytest.raises(ValueError,match='matching trace'):m.finalize(path,tmp_path/'bad.json',500,900,'floor')


@pytest.mark.parametrize('low,high',[(float('nan'),100),(0,float('inf')),(10,0),(0,1)])
def test_invalid_heights(tmp_path,low,high):
    with pytest.raises(ValueError):m.finalize(tmp_path/'missing.json',tmp_path/'out.json',low,high,'floor')


def test_wrong_firmware_and_cancel_never_write_motors(tmp_path):
    port=Port([INFO|dict(firmware='arm-calibration-v1')])
    with pytest.raises(ValueError):m.capture(port,tmp_path/'wrong.json')
    assert port.writes==[b'INFO\n']
    port=Port([INFO,motor(123,1)])
    path=tmp_path/'cancel.json'
    with pytest.raises(RuntimeError):m.capture(port,path,control=lambda:'abort')
    assert json.loads(path.read_text())['status']=='incomplete_do_not_use'
    assert port.writes==[b'INFO\n',b'LIFT\n']


def test_high_to_low_capture_has_same_up_direction():
    s=m.LiftSweep();hold(s,2690);s.mark('high')
    for i in range(1,101):add(s,2690+i*100)
    hold(s,12690);s.mark('low')
    assert s.finish()['encoder_delta_raw']==-10000
    assert s.finish()['encoder_up_direction']==-1


def test_single_missing_register_read_recovery_keeps_original_continuity():
    s=m.LiftSweep();s.add(motor(4090,1),1)
    s.note_read_failure(motor(-1,2),1.05)
    assert not s.window
    s.add(motor(10,3),1.1)
    assert s.value==16 and s.read_failures==1 and s.maximum_gap==pytest.approx(.1)
    with pytest.raises(ValueError):s.mark('low')
    s.note_read_failure(motor(20,4,torque_raw=-1),1.15)
    with pytest.raises(ValueError):s.note_read_failure(motor(20,5,torque_raw=-1),1.2)
    with pytest.raises(ValueError):s.add(motor(300,5),1.2)
    with pytest.raises(ValueError):s.add(motor(20,5),1.7)


@pytest.mark.parametrize('changes',[dict(torque_raw=1),dict(mode_raw=1),dict(id=9),
                                  dict(position_raw=4096),dict(model_raw=0),dict(read_sequence=1)])
def test_retry_does_not_hide_real_state_or_sequence_errors(changes):
    s=m.LiftSweep();s.add(motor(100,1),1)
    row=motor(-1,2);row.update(changes)
    with pytest.raises(ValueError):s.note_read_failure(row,1.05)


def test_failed_capture_retains_exact_bad_response(tmp_path):
    bad=motor(123,1,torque_raw=1)
    port=Port([INFO,bad]);path=tmp_path/'failed.json'
    with pytest.raises(ValueError):m.capture(port,path)
    record=json.loads(path.read_text())
    assert record['last_rejected_sample']['motor']==bad
    assert record['status']=='incomplete_do_not_use'


@pytest.mark.parametrize('failure_kind', ['register', 'actual_timeout'])
def test_recovered_read_is_replayed_and_verified_when_finalizing(tmp_path,failure_kind):
    values=[4000]*16+[4000+i*100 for i in range(1,101)]+[14000]*16
    rows=[motor(v%4096,1) for v in values]
    failure = motor(-1,1) if failure_kind == 'register' else json.loads(
        (ROOT/'tests/fixtures/lift_read_timeout.json').read_text())['motor']
    rows.insert(70,failure)
    for i,row in enumerate(rows):row['read_sequence']=i+1
    port=Port([INFO,*rows])
    actions=iter([('low' if i==16 else 'high' if i==131 else 'finish' if i==132 else None)
                  for i in range(1,len(values)+1)])
    tick=[0]
    def clock():
        tick[0]+=.025
        return tick[0]
    path=tmp_path/'retry.json'
    record=m.capture(port,path,control=lambda:next(actions),clock=clock,sleep=lambda _:None)
    assert record['recovered_read_failures']==1
    assert record['last_rejected_sample']['motor']['position_raw']==-1
    result=m.finalize(path,tmp_path/'result.json',270,890,'floor to platform')
    assert result['maximum_height_um']==620000
    assert result['um_per_turn']==253952
    assert set(port.writes)=={b'INFO\n',b'LIFT\n'}


def test_actual_timeout_response_allows_one_read_but_never_erases_gap_limit():
    event=json.loads((ROOT/'tests/fixtures/lift_read_timeout.json').read_text())
    def prepared():
        sweep=m.LiftSweep()
        sweep.add(motor(2563,10105),8056.319237324)
        sweep.note_read_failure(event['motor'],event['time_monotonic'])
        return sweep
    s=prepared()
    s.add(motor(2700,10107),8056.48)
    assert s.value==137 and s.read_failures==1
    with pytest.raises(ValueError):prepared().add(motor(2700,10107),8056.9)
    with pytest.raises(ValueError):prepared().add(motor(2900,10107),8056.48)
    with pytest.raises(ValueError):prepared().note_read_failure(event['motor']|dict(read_sequence=10107),8056.48)
    for changes in ({'identity_hal_status':2},{'identity_servo_status':32},{'identity_failure':'SERVO_STATUS'}):
        s=m.LiftSweep();s.add(motor(2563,10105),8056.319237324)
        with pytest.raises(ValueError):s.note_read_failure(event['motor']|changes,event['time_monotonic'])
