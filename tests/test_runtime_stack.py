from copy import deepcopy
from dataclasses import replace
import json
import subprocess
from pathlib import Path
import pytest
from home_robot_tasks.offline_stack import OfflineStack
from home_robot_tasks.recovery import RecoveryCoordinator, RecoveryEvidence
from home_robot_tasks.commissioning import validate_board_profile, export_profile
from home_robot_tasks.mission_catalog import MissionCatalog

ROOT=Path(__file__).resolve().parents[1]
def read(name):return json.loads((ROOT/'config'/name).read_text())
def stack(boot_id=42):return OfflineStack(read('fetch_stack.simulation.json'),read('home.example.json'),read('navigation_map.simulation.json'),boot_id=boot_id)
def request():return dict(schema_version=1,operation='fetch_object',request_id='stack',request=read('fetch_remote.example.json'))


def test_production_factory_resolves_and_executes_whole_offline_fetch():
    s=stack();r=s.run(request())
    assert r['status']=='SUCCEEDED' and r['completed_steps']==15
    assert r['hardware_commands']==0 and not r['physical_task_completed']
    assert s.ports['navigation'].starts[0][1]['goal']['x']==2.
    assert s.ports['navigation'].starts[-1][1]['goal']['x']==6.
    assert s.assembly.catalog.surface['object_at_destination']
    assert [h['phase'] for h in r['manipulation_stages']]==[
        'pick_open','pick_pregrasp','pick_approach','pick_close','pick_attach','pick_retreat',
        'place_preplace','place_approach','place_open','place_detach','place_retreat']
    assert len(s.ports['gripper'].starts)==3 and len(s.ports['scene'].starts)==2
    assert s.attached is None


@pytest.mark.parametrize('stage',[
    'pick_open','pick_pregrasp','pick_approach','pick_close','pick_attach','pick_retreat',
    'place_preplace','place_approach','place_open','place_detach','place_retreat'])
def test_whole_runtime_cancels_inside_every_manipulation_stage(stage):
    s=stack();s.runtime.submit(request());task=s.runtime.app.tasks['stack']
    cancelled=False;last_starts=None
    for _ in range(1000):
        seq=s.assembly.manipulation.sequence
        if seq.owner:
            e=seq.entries[seq.owner]
            if e['started'] and e['phases'][e['index']].name==stage and not cancelled:
                s.runtime.app.cancel(task.run_id,s.now);cancelled=True
                last_starts={k:len(p.starts) for k,p in s.ports.items()}
        s.tick()
        if task.state in task.TERMINAL or task.state=='STOP_UNCONFIRMED':break
    assert cancelled and task.state=='CANCELLED' and s.stop.requests==1
    assert last_starts=={k:len(p.starts) for k,p in s.ports.items()}


def test_false_gripper_success_cannot_attach_or_report_a_pick():
    from home_robot_tasks.skill_ports import PortResult
    s=stack();jaw=s.ports['gripper']
    def falsely_successful(g):
        jaw.entries[g]['state']='SUCCEEDED'  # Sensor plant remains empty.
        return PortResult('SUCCEEDED')
    jaw.poll=falsely_successful
    result=s.run(request())
    assert result['status']=='FAILED' and 'pick_close' in result['reason']
    assert not s.ports['scene'].starts and not s.held


@pytest.mark.parametrize('index',range(15))
def test_each_composed_phase_cancels_with_independent_empty_or_held_proof(index):
    s=stack();r=s.run(request(),cancel_step=index)
    assert r['status']=='CANCELLED' and r['stop_requests']==1 and not r['control_owned']


@pytest.mark.parametrize('fault',('sensor_loss','reboot','load_loss'))
def test_fault_does_not_erase_control_or_replay_job(fault):
    s=stack();r=s.run(request(),fault=fault)
    assert r['status']=='STOP_UNCONFIRMED' and r['control_owned']
    with pytest.raises(ValueError):s.runtime.submit({**request(),'request_id':'next'})


def test_pose_or_scene_change_invalidates_cached_target_and_bad_map_is_rejected():
    s=stack();s.run(request())
    c=s.assembly.catalog;c.checked_target(True)
    s.scene_revision='scene-2'
    with pytest.raises(ValueError):c.checked_target(True)
    s.scene_revision='scene-1';s.xy_yaw=(2.,2.,0.);s.tick()
    with pytest.raises(ValueError):c.checked_target(True)
    d=read('fetch_stack.simulation.json')['catalog'];d['map_revision']='wrong'
    with pytest.raises(ValueError):MissionCatalog(d,c.world,c.map,c.xy,c.motion,c.scene,c.clock)


def test_idle_deadline_fault_blocks_direct_app_admission_and_keeps_observation_running():
    s=stack();observed=[]
    s.runtime.observations=(lambda:observed.append(s.now),)
    s.runtime.tick();s.now+=1;s.runtime.tick()
    assert s.runtime.fault=='control_tick_deadline' and len(observed)==2 and s.stop.requests==1
    with pytest.raises(ValueError):s.runtime.app.submit(request(),s.now)
    s.now+=.05;s.runtime.tick();assert len(observed)==3


def test_robot_state_replay_and_reboot_latch_fault_and_preserve_last_sample_time():
    s=stack();old=s.state.latest
    with pytest.raises(ValueError):s.state.observe(old)
    assert not s.state.fresh() and s.state.latest.observed_s==old.observed_s
    s=stack();old=s.state.latest;s.now+=.05
    with pytest.raises(ValueError):s.state.observe(replace(old,sequence=old.sequence+1,observed_s=s.now,boot_id=43))
    assert s.state.boot==42 and s.state.fault=='device restarted'


def test_explicit_recovery_requires_new_disabled_boot_and_never_replays_old_job():
    s=stack();s.runtime.submit(request());s.tick()
    recovery=RecoveryCoordinator(s.runtime,42,'a'*64,s.clock)
    recovery.begin('operator secured mechanism')
    evidence=RecoveryEvidence(s.now,42,43,True,True,True,True,True,'a'*64)
    for bad in (replace(evidence,new_boot=42),replace(evidence,old_clients_closed=False),
                replace(evidence,outputs_disabled=False),replace(evidence,profile_sha256='b'*64)):
        with pytest.raises(ValueError):recovery.accept(bad)
    recovery.accept(evidence)
    new=recovery.bootstrap(lambda boot:stack(boot).runtime)
    assert recovery.state=='RETIRED' and not new.app.tasks
    assert s.runtime.fault.startswith('explicit_recovery')
    with pytest.raises(ValueError):s.runtime.submit({**request(),'request_id':'later'})
    with pytest.raises(ValueError):recovery.bootstrap(lambda boot:new)


def test_partial_bootstrap_is_never_repeated():
    s=stack();r=RecoveryCoordinator(s.runtime,42,'a'*64,s.clock);r.begin('test')
    r.accept(RecoveryEvidence(s.now,42,43,True,True,True,True,True,'a'*64))
    def broken(boot):raise RuntimeError('partial initialization')
    with pytest.raises(RuntimeError):r.bootstrap(broken)
    with pytest.raises(ValueError):r.bootstrap(broken)
    assert r.state=='BOOTSTRAP_FAILED'


def test_profile_export_hashes_and_simulation_compile_guard(tmp_path):
    p=read('mobile_board.simulation.json');m=export_profile(p,tmp_path/'profile')
    assert not m['hardware_authorized']
    assert '#error "Synthetic profile' in (tmp_path/'profile/mobile_profile.h').read_text()
    with pytest.raises(ValueError):export_profile(p,tmp_path/'profile')
    p['mode']='measured'
    with pytest.raises(ValueError):validate_board_profile(p)


@pytest.mark.parametrize('section,key,value',[
    ('lift','maximum_encoder_step_raw',2048),('mobile','lift_max_um',1),
    ('stop','feedback_max_age_ms',True),('stop','zero_budget_us',5),
    ('lift','encoder_up_direction',0),('lift','home_current_ma',999999)])
def test_invalid_profile_cannot_generate_deployment(section,key,value):
    p=read('mobile_board.simulation.json');p['board'][section][key]=value
    with pytest.raises(ValueError):validate_board_profile(p)


def test_scene_readiness_alone_never_proves_reachable_or_collision_free():
    s=stack();s.run(request());s.tick();v=s.state.latest
    assert s.state.conditions()['reachable']
    for changed in ({'checked_capture_id':'old'}, {'checked_scene_revision':'old'},
                    {'reachable':False,'collision_checked':False}):
        s.state.latest=replace(v,**changed)
        c=s.state.conditions()
        assert not c['reachable'] and not c['collision_checked']
    s.state.latest=v


def test_exported_profile_compiles_against_actual_board_type_and_rejects_default_build(tmp_path):
    export_profile(read('mobile_board.simulation.json'),tmp_path/'profile')
    src=tmp_path/'profile.c'
    src.write_text('#include "mobile_board.h"\n#include "profile/mobile_profile.h"\nint main(void){return commissioned_mobile_profile.devices.model[0]==777 ? 0:1;}\n')
    command=['gcc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'tests/fixtures/servo_hal'),
             '-I',str(ROOT/'firmware/stm32_g474_single_arm/Core/Inc'),'-I',str(ROOT/'firmware/stm32_actuator/include'),
             str(src),'-o',str(tmp_path/'profile-test')]
    blocked=subprocess.run(command,capture_output=True,text=True)
    assert blocked.returncode and 'Synthetic profile' in blocked.stderr
    subprocess.run([*command,'-DMOBILE_PROFILE_OFFLINE_TEST'],check=True)
    subprocess.run([str(tmp_path/'profile-test')],check=True)
