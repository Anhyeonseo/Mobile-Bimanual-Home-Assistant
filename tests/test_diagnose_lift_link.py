import importlib.util
import json
from pathlib import Path
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/run'))
spec=importlib.util.spec_from_file_location('diagnose_lift_link',ROOT/'tools/run/diagnose_lift_link.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
INFO=dict(firmware='mobile-bus-inspection-v2',motion_enabled=False,lift_read_supported=True,
          ids=[7,8,9,10],reference_id=1,host_baud=921600,servo_baud=1000000)


def good(seq):
    return dict(id=10,reported_id=10,identity_ok=True,model_raw=777,baud_code=0,
                mode_raw=0,torque_raw=0,position_raw=2000,read_sequence=seq)


def bad(seq):
    return good(seq)|dict(identity_ok=False,model_raw=-1,reported_id=-1,baud_code=-1,
                          mode_raw=-1,torque_raw=-1,position_raw=-1,
                          identity_failure='UART',identity_uart_error=4)


class Port:
    def __init__(self,replies):self.replies=list(replies);self.writes=[]
    def reset_input_buffer(self):pass
    def write(self,data):self.writes.append(data);return len(data)
    def readline(self):return (json.dumps(self.replies.pop(0))+'\n').encode()


def comparison():
    return [INFO,{'scan':'begin'},dict(id=1,reported_id=1,identity_ok=True),
            *[good(0)|dict(id=i,reported_id=i) for i in range(7,11)],
            dict(scan='end',motor_write_commands=0)]


def test_fault_and_recovery_retained_without_fabricating_calibration(tmp_path):
    port=Port([INFO,good(1),bad(2),*comparison(),bad(20),good(21)])
    actions=iter([None,None,None,'finish'])
    result=m.run(port,tmp_path/'check.json',control=lambda:next(actions),sleep=lambda _:None)
    assert result['bad_reply_count']==2 and result['recovered_episodes']==1
    assert result['comparisons']==1 and not result['calibration_valid']
    events=[json.loads(x) for x in (tmp_path/'check.jsonl').read_text().splitlines()]
    assert len([e for e in events if e['event']=='lift_read'])==4
    assert any(e.get('motor',{}).get('identity_uart_error')==4 for e in events)
    assert set(port.writes)=={b'INFO\n',b'LIFT\n',b'SCAN\n'}


@pytest.mark.parametrize('change',[dict(torque_raw=1),dict(mode_raw=1),dict(model_raw=1),dict(id=1),dict(read_sequence=1)])
def test_real_state_or_restart_change_stops_and_preserves_reply(tmp_path,change):
    row=good(2)|change;port=Port([INFO,good(1),row])
    path=tmp_path/'check.json'
    with pytest.raises(ValueError):m.run(port,path,control=lambda:None,sleep=lambda _:None)
    assert json.loads(path.read_text())['status']=='diagnostic_interrupted'
    assert json.loads(path.with_suffix('.jsonl').read_text().splitlines()[-1])['motor']==row
    assert b'SCAN\n' not in port.writes


def test_wrong_firmware_sends_only_info(tmp_path):
    port=Port([INFO|dict(firmware='other')])
    with pytest.raises(ValueError):m.run(port,tmp_path/'check.json')
    assert port.writes==[b'INFO\n']
