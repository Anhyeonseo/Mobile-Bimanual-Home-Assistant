import json
import sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/run'))
import move_lift_height as auto
import teleop_lift as base


@pytest.mark.parametrize('value',[0,-1,61.001,62,float('nan'),float('inf')])
def test_target_bounds_before_connection(value):
    with pytest.raises(ValueError):auto.target_counts(value)


def test_upper_target_retains_firmware_guard_margin(monkeypatch):
    assert auto.target_counts(61) == -29745
    assert auto.INFO['top_guard_counts'] - abs(auto.target_counts(61)) >= 250
    monkeypatch.setitem(auto.INFO, 'top_guard_counts', 29900)
    with pytest.raises(ValueError):auto.target_counts(61)


@pytest.mark.parametrize('mode',['target','upper_target','pulse','timeout','reference','fault','stop','down','visible','visible_stall','visible_upper'])
def test_automatic_move_and_failures_always_stop(monkeypatch,tmp_path,mode):
    class Port:
        def __init__(self):self.data=bytearray();self.writes=[];self.count=-29900 if mode=='visible_upper' else -2000 if mode=='down' else 0;self.seq=0;self.moves=0
        def write(self,data):
            self.writes.append(data)
            if data==b'STATUS\n':
                row=dict(type='status',armed=False,fault=False,reference_id=77,motor_write_count=0)
            elif data.startswith((b'ARM ',b'TEST ')):
                row=dict(type='armed',armed=True,fault=False,session=123,sequence=0,reference_id=77)
            elif data.startswith(b'D '):
                self.seq+=1;motion=data.decode().split()[3]
                if motion!='Z':
                    if mode=='timeout':raise RuntimeError('timeout')
                    self.moves+=1
                    if mode!='visible_stall':self.count+=-100 if motion=='U' else 100
                row=dict(type='ack',session=123,sequence=self.seq,armed=True,fault=False,reference_id=77,
                         relative_counts=self.count,sample_tick=self.seq*50,sample_count=self.seq,
                         speed_raw=0,goal_raw={'U':32868,'D':100,'Z':0}[motion],torque_raw=1,
                         telemetry_valid=True,motor_status_raw=0)
                if self.moves:
                    if mode=='reference':row['reference_id']=0
                    if mode=='fault':row['motor_status_raw']=8
                if mode=='pulse' and self.moves==3:
                    row.update(type='stopped',armed=False,reason='TEST_PULSE_COMPLETE',stop_confirmed=True)
            else:
                assert data==b'STOP\n'
                row=dict(type='stopped',armed=False,fault=False,stop_confirmed=mode!='stop',reference_id=77)
            self.data.extend(json.dumps(row).encode()+b'\n');return len(data)
        def read(self,n):r=bytes(self.data[:n]);del self.data[:n];return r
    monkeypatch.setattr(base,'inspect',lambda *a:auto.INFO)
    monkeypatch.setattr(auto.secrets,'randbelow',lambda n:122)
    if mode=='visible_stall':
        clock=[0.0]
        monkeypatch.setattr(auto.time,'monotonic',lambda:clock[0])
        monkeypatch.setattr(auto.time,'sleep',lambda t:clock.__setitem__(0,clock[0]+max(.05,t)))
    else:monkeypatch.setattr(auto.time,'sleep',lambda t:None)
    p=Port();path=tmp_path/'report.json'
    args=dict(visible_check=True) if mode.startswith('visible') else dict(check_up=True) if mode=='pulse' else dict(height_cm=61 if mode=='upper_target' else 2.05)
    if mode in ('timeout','reference','fault','stop','visible_stall','visible_upper'):
        with pytest.raises(RuntimeError):auto.run(p,path,**args)
    else:auto.run(p,path,**args)
    assert p.writes[-1]==b'STOP\n'
    report=json.loads(path.read_text())
    assert not report['scale_validated'] and not report['s2_complete']
    if mode=='target':
        assert report['status']=='target_reached_and_stopped'
        assert report['relative_counts']==-1000
    elif mode=='upper_target':
        assert report['status']=='target_reached_and_stopped'
        assert -auto.INFO['top_guard_counts'] < report['relative_counts'] <= auto.target_counts(61)
    elif mode=='down':
        assert report['relative_counts']==-1000
    elif mode=='visible':
        assert report['status']=='visible_up_check_stopped'
        assert report['observed_upward_counts']==500
        assert not any(w.startswith(b'TEST ') for w in p.writes)
    elif mode=='visible_stall':
        assert report['status']=='failed' and '15초' in report['error']
    elif mode=='visible_upper':
        assert report['status']=='failed'
        assert not any(b' U ' in w for w in p.writes)
    elif mode=='pulse':
        assert report['status']=='up_check_stopped' and report['observed_upward_counts']==300
    else:assert report['status']=='failed'
