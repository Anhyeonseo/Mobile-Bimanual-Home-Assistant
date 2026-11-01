import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/run'))
import pose_locked_profile as m

def calibration():
    rows=[]
    for id in range(1,7):
        e=bytearray(40);e[5]=id;e[31]=100+id
        rows.append(dict(id=id,ok=True,position=2048,torque=0,eeprom_hex=e.hex()))
    return dict(status='centered_verified',after=rows)

def record(jaw_sign=1):
    r=m.new_record(calibration())
    poses=[[2048,2048,2048,2048,2048,1900],
           [2100,2200,2300,2200,2048,1900],
           [2100,2250,2350,2220,2048,1900],
           [2105,2251,2355,2220,2048,1900+jaw_sign*300],
           [2100,2200,2300,2200,2048,1900+jaw_sign*300]]
    for name,q in zip(m.STAGES,poses):
        r['poses'][name]=dict(source='maintenance_read_only_three_scans',samples_raw=[q[:],q[:],q[:]],positions_raw=q[:],captured_at='2026-10-16T00:00:00Z')
    return r

def set_pose(r,name,q):
    r['poses'][name]['positions_raw']=q[:];r['poses'][name]['samples_raw']=[q[:],q[:],q[:]]

@pytest.mark.parametrize('jaw_sign',[1,-1])
def test_compile_only_taught_edges_and_exact_gripper_holds(jaw_sign):
    p=m.compile_profile(record(jaw_sign))
    assert p['allowed_edges']==[[0,1],[1,2],[2,3],[3,4]]
    q=p['targets_raw']
    assert q[2][:5]==q[3][:5]
    assert q[0][5]==q[1][5]==q[2][5]
    assert q[3][5]==q[4][5]
    assert p['targets_urad'][3][5]<p['targets_urad'][2][5]
    for j,b in enumerate(p['bounds']):
        assert b['minimum_raw']==max(0,min(v[j] for v in q)-m.MARGIN_RAW)
        assert b['maximum_raw']==min(4095,max(v[j] for v in q)+m.MARGIN_RAW)
    assert not p['collision_checked'] and not p['physical_replay_verified']

@pytest.mark.parametrize('defect',['missing','unstable','offset','old_recipe','wrap','wide','closed_arm','open_jaw','same_jaw','bad_sample','spoofed_sample'])
def test_reject_bad_measurements_and_incompatible_recipes(defect):
    r=record();expected=m.new_record(calibration())
    if defect=='missing':del r['poses']['lift']
    elif defect=='unstable':r['poses']['home']['samples_raw'][0][0]+=50
    elif defect=='offset':r['offsets_raw'][0]+=1
    elif defect=='old_recipe':r['record_kind']='measured_left_only_fixed_pick_demo'
    elif defect=='wrap':
        set_pose(r,'home',[0,*r['poses']['home']['positions_raw'][1:]])
        set_pose(r,'lift',[4095,*r['poses']['lift']['positions_raw'][1:]])
    elif defect=='wide':set_pose(r,'lift',[33,*r['poses']['lift']['positions_raw'][1:]])
    elif defect=='closed_arm':set_pose(r,'closed',[2500,*r['poses']['closed']['positions_raw'][1:]])
    elif defect=='open_jaw':q=r['poses']['lift']['positions_raw'][:];q[5]=1900;set_pose(r,'lift',q)
    elif defect=='same_jaw':q=r['poses']['closed']['positions_raw'][:];q[5]=1900;set_pose(r,'closed',q)
    elif defect=='bad_sample':r['poses']['home']['samples_raw'][0][0]=float('nan')
    elif defect=='spoofed_sample':r['poses']['home']['positions_raw'][0]+=1
    with pytest.raises(ValueError):
        m.validate_record(r,expected);m.compile_profile(r)

class Link:
    def __init__(self,defect=None):self.calls=[];self.defect=defect
    def exchange(self,command,scan=False):
        assert command=='SCAN' and scan
        self.calls.append(command)
        rows=calibration()['after']
        if self.defect=='torque':rows[0]['torque']=1
        elif self.defect=='offset':rows[0]['eeprom_hex']='00'*40
        elif self.defect=='unstable':rows[0]['position']+=50*len(self.calls)
        elif self.defect=='missing':rows.pop()
        return rows

def test_capture_uses_only_three_fresh_read_only_scans():
    link=Link();r=m.new_record(calibration());p=m.capture(link,r,sleep=lambda _:None)
    assert p['positions_raw']==[2048]*6 and link.calls==['SCAN']*3
@pytest.mark.parametrize('defect',['torque','offset','unstable','missing'])
def test_capture_rejects_without_any_motor_write(defect):
    link=Link(defect)
    with pytest.raises(ValueError):m.capture(link,m.new_record(calibration()),sleep=lambda _:None)
    assert set(link.calls)=={'SCAN'}

def test_profile_changes_when_pose_or_calibration_changes():
    r=record();a=m.compile_profile(r)
    r['poses']['lift']['captured_at']='new capture'
    b=m.compile_profile(r)
    assert a['profile_sha256']!=b['profile_sha256']

def test_cli_check_offline(tmp_path):
    path=tmp_path/'poses.json';path.write_text(json.dumps(record()))
    result=subprocess.run([sys.executable,str(ROOT/'tools/run/teach_left_demo.py'),'--check','--file',str(path)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['record_kind']=='left_pose_locked_profile'

@pytest.mark.parametrize('direction',[-1,1])
def test_gripper_preload_preserves_measurements_and_arm_targets(direction):
    r=record(direction);before=copy.deepcopy(r);base=m.compile_profile(r)
    r['gripper_close_extra_raw']=20;p=m.compile_profile(r)
    assert r['poses']==before['poses']
    for stage in range(5):
        assert p['targets_raw'][stage][:5]==base['targets_raw'][stage][:5]
        assert p['targets_raw'][stage][5]==base['targets_raw'][stage][5]+(direction*20 if stage in (3,4) else 0)
    assert p['offsets_raw']==base['offsets_raw']
    assert p['profile_sha256']!=base['profile_sha256']
    assert m.gripper_tolerance_urad(p)>=round(32*m.URAD_PER_RAW)

@pytest.mark.parametrize('extra',[-1,33,True,20.0,'20'])
def test_invalid_gripper_preload_is_rejected(extra):
    r=record();r['gripper_close_extra_raw']=extra
    with pytest.raises(ValueError):m.compile_profile(r)
