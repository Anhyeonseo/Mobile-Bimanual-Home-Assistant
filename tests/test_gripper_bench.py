from pathlib import Path
import json
import subprocess
import pytest

from tools.run.bench_left_gripper import trial
ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'
FAKE=ROOT/'tests/fixtures/servo_hal'


def test_mcu_bench_interlocks_and_watchdog(tmp_path):
    exe = tmp_path/'grip'
    subprocess.run(['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror','-DHOST_S3_LEFT_GRIP_BENCH=1',
                    '-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),
                    str(FAKE/'gripper_bench_harness.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def test_transport_isolates_left_jaw_and_bounds_all_writes(tmp_path):
    source=(FAKE/'left_calibration_gate.c').read_text()
    old=source.index(' for(unsigned id=')
    source=source[:old]+"""
    for(unsigned id=0;id<256;id++) for(unsigned addr=0;addr<256;addr++) for(unsigned v=0;v<256;v++){
        uint8_t p[8]={255,255,id,4,3,addr,v,0}; unsigned sum=0;
        for(unsigned i=2;i<7;i++)sum+=p[i];
        p[7]=~sum;
        uint32_t t;assert(ServoTransport_BeginService(&huart1,ACTUATOR_BUS_WORK_ARM,&t));
        assert((ServoTransport_Transmit(&huart1,t,p,8,10)==HAL_OK)==(id==6&&addr==40&&v<=1));
        assert(ServoTransport_End(&huart1,t,false)==HAL_OK);
    }
    for(unsigned pos=0;pos<4096;pos++) for(unsigned size=9;size<=15;size+=6){
        uint8_t p[15]={255,255,6,size-4,3,42,pos,pos>>8,0,0,75,0,80,0,0};
        unsigned sum=0;for(unsigned i=2;i<size-1;i++)sum+=p[i];p[size-1]=~sum;
        uint32_t t;assert(ServoTransport_BeginService(&huart1,ACTUATOR_BUS_WORK_ARM,&t));
        assert((ServoTransport_Transmit(&huart1,t,p,size,10)==HAL_OK)==(size==9&&pos>=1950&&pos<=3172));
        assert(ServoTransport_Transmit(&huart1,t,p,size,10)!=HAL_OK);
        assert(ServoTransport_End(&huart1,t,false)==HAL_OK);
        if(size==15){
            p[10]=0; p[14]+=75;
            assert(ServoTransport_BeginService(&huart1,ACTUATOR_BUS_WORK_ARM,&t));
            assert(ServoTransport_Transmit(&huart1,t,p,size,10)!=HAL_OK);
            assert(ServoTransport_End(&huart1,t,false)==HAL_OK);
        }
    }
    assert(other.sent==0);return 0;
    }
    """
    path=tmp_path/'gate.c';path.write_text(source); exe=tmp_path/'gate'
    subprocess.run(['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror','-DHOST_S3_LEFT_GRIP_BENCH=1',
                    '-DHOST_LEFT_ONLY_BUILD=1','-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),
                    str(path),str(BOARD/'Src/servo_transport.c'),str(CORE/'src/bus_router.c'),
                    str(CORE/'src/shared_bus.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=30)


class BenchPort:
    def __init__(self, fault=''):
        self.fault=fault; self.commands=[]; self.ticket=0; self.floor=0; self.entry=0

    def write(self, data):
        cmd=data.decode().split()
        self.commands.append(cmd[0])
        if cmd[0]=='BIND':
            self.ticket=int(cmd[1])
        if cmd[0]=='BEGIN':self.floor,self.entry=map(int,cmd[2:4])
        row=dict(firmware='left-grip-bench-v4',floor_raw=self.floor,entry_raw=self.entry,goal=2900,ticket=self.ticket,sequence=0,
                 active=0,bound=0,locked=0,off_verified=1,torque=0,sample_valid=1,
                 position=2900,reason='READY')
        if cmd[0]=='BIND':row['bound']=1
        if cmd[0]=='BEGIN':row.update(active=1,off_verified=0,reason='ANCHORED')
        if cmd[0]=='STEP':row.update(sequence=int(cmd[2]),reason='EFFORT_BOUND')
        if cmd[0]=='STOP' and self.fault=='unconfirmed':row['off_verified']=0
        self.reply=json.dumps(row).encode()+b'\n'
        if cmd[0]=='STEP' and self.fault=='timeout':self.reply=b''
        return len(data)

    def readline(self, size):return self.reply


@pytest.mark.parametrize('mode', ['identity','inspect'])
def test_readonly_paths_never_arm(mode):
    port=BenchPort();trial(port,mode,{'samples':[]})
    assert port.commands == (['INFO'] if mode=='identity' else ['INFO','BIND'])


@pytest.mark.parametrize('fault', ['','timeout','unconfirmed','pose_changed'])
def test_only_matching_explicit_trial_moves_and_always_confirms_off(fault):
    port=BenchPort(fault);report={'samples':[]}
    if fault:
        with pytest.raises(ValueError):
            trial(port,'empty',report,expected_start=2750 if fault=='pose_changed' else 2900,sleep=lambda _:None)
    else:
        trial(port,'empty',report,expected_start=2900,sleep=lambda _:None)
        assert report['endpoint']['reason']=='EFFORT_BOUND'
    assert port.commands[-1] == ('BIND' if fault=='pose_changed' else 'STOP')


def completed_record(label, start, end):
    common = dict(ticket=11, firmware='left-grip-bench-v3', sample_valid=1, torque_limit_raw=80)
    terminal = dict(common, sequence=1, active=0, off_verified=1, locked=0, torque=0, position=end,
                    reason='NO_PROGRESS', load_raw=1028, current_raw=32770)
    return dict(status='trial_recorded',operator_label=label,grasp_verified=False,
                before=dict(common,position=start,torque=0), start_reply=dict(common,reason='ANCHORED',active=1),
                samples=[terminal],endpoint=terminal,stop=terminal.copy())


def test_paired_closures_compare_raw_endpoints_without_calibrating_a_force():
    from tools.diagnostics.compare_gripper_bench import compare
    empty=completed_record('empty',2281,1980)
    target=completed_record('target',2288,2170)
    result=compare(empty,target)
    assert result['endpoint_difference_raw']==190
    assert result['empty']['peak_load_raw']==4 and result['empty']['peak_current_raw']==2
    assert not result['grasp_verified'] and not result['calibrated_contact_threshold']
    target['before']['position']+=30
    with pytest.raises(ValueError,match='starting aperture'):
        compare(empty,target)
    empty['stop']['off_verified']=0
    with pytest.raises(ValueError,match='confirmed torque-off'):
        compare(empty,target)


def target_plan(**changes):
    return dict(dict(mode='supported_left_jaw_contact',floor_raw=2794,upper_contact_raw=2822,
                     entry_raw=2950,capture_id='one',calibration_id='measured',label='object'),**changes)


@pytest.mark.parametrize('bad', [None, target_plan(floor_raw=1000), target_plan(upper_contact_raw=3000),
                                 target_plan(entry_raw=True), target_plan(upper_contact_raw=2899)])
def test_target_requires_its_limits_before_enabling(bad):
    port=BenchPort()
    with pytest.raises(ValueError):
        trial(port,'target',{},target_plan=bad,expected_start=2900,sleep=lambda _:None)
    assert 'BEGIN' not in port.commands


def test_target_caps_are_sent_to_mcu_and_an_effort_peak_is_inconclusive():
    port=BenchPort()
    report={'samples':[]}
    trial(port,'target',report,target_plan=target_plan(),expected_start=2900,sleep=lambda _:None)
    assert port.floor==2794 and port.entry==2950
    assert report['contact_outcome']=='EFFORT_OR_STALL_UNCONFIRMED'
    assert report['stop']['off_verified']==1


def test_persistent_resistance_must_be_inside_the_predicted_band():
    from tools.run.bench_left_gripper import target_outcome
    for pos, expected in ((2815,'CONTACT_CANDIDATE'),(2860,'UNEXPECTED_RESISTANCE')):
        assert target_outcome(dict(position=pos,reason='EFFORT_PERSISTED'),target_plan())==expected
    assert target_outcome(dict(position=2795,reason='LOWER_GUARD'),target_plan())=='NO_CONFIRMED_CONTACT_AT_FLOOR'


def test_activation_rechecks_acknowledged_target_limits():
    class WrongBounds(BenchPort):
        def write(self,data):
            n=super().write(data)
            if data.startswith(b'BEGIN '):
                row=json.loads(self.reply);row['floor_raw']=1950
                self.reply=json.dumps(row).encode()+b'\n'
            return n
    port=WrongBounds()
    with pytest.raises(ValueError,match='did not bind'):
        trial(port,'target',{'samples':[]},target_plan=target_plan(),expected_start=2900,sleep=lambda _:None)
    assert port.commands[-1]=='STOP' and 'STEP' not in port.commands


def test_contact_plan_is_recomputed_from_the_selected_observed_section():
    from tools.diagnostics.prepare_gripper_contact import prepare, stock_jaw, closure_plan, InstalledLeftJaw
    from home_robot_tasks.snapshot_io import FILES
    config=ROOT/'config/home_assistant'
    coords=json.loads((config/'s2_coordinates.development.json').read_text())
    calibration=json.loads((config/'left_pad_gap.observed.json').read_text())
    item={'width_m':.047,'source':'rule','closing_section':{'status':'VISIBLE_SPAN','low_m':-.017,'high_m':.018}}
    closure=closure_plan(stock_jaw(),item)
    jaw=InstalledLeftJaw(calibration,coords)
    frame='color_camera_optical_frame'
    obs=dict(capture_id='fixture',label='target',source_sha256=dict.fromkeys(FILES,'b'*64),
             frame_id=frame,mask_sha256='a'*64,status='accepted')
    group=dict(obs,perception_status='accepted',tcp_convention='graspnet_origin_plus_depth_to_pad_tip',
               coordinate_frame=frame,observation_index=0,refined_grasps=[item])
    diag=dict(jaw_closure=closure,left_installed_pad_preview=jaw.preview(closure))
    replay=dict(obs,group=group,observations=[obs.copy()],candidate_diagnostics=[diag],
                coordinate_frame=frame,mode='offline_grasp_replay',motion_authorized=False)
    plan=prepare(replay,0,calibration,coords)
    assert plan['floor_gap_mm'] == pytest.approx(30.) and plan['floor_raw'] < plan['upper_contact_raw'] < plan['entry_raw']
    # A stored legacy replay and a current replay must choose exactly the same motor target.
    del diag['jaw_closure']['target_gap_m']
    assert prepare(replay,0,calibration,coords)==plan and plan['target_raw']==plan['floor_raw']
    diag['jaw_closure']['target_gap_m']=closure['min_search_gap_m']-.001
    with pytest.raises(ValueError,match='inconsistent'):
        prepare(replay,0,calibration,coords)
    diag['jaw_closure']['target_gap_m']=closure['min_search_gap_m']
    replay['observations'][0]['frame_id']='base_link'
    with pytest.raises(ValueError,match='capture mismatch'):
        prepare(replay,0,calibration,coords)
    replay['observations'][0]['frame_id']=frame
    diag['left_installed_pad_preview']['lower_search_raw']-=1
    with pytest.raises(ValueError,match='calibration mismatch'):
        prepare(replay,0,calibration,coords)
    replay['observations'][0]['mask_sha256']='c'*64
    with pytest.raises(ValueError,match='capture mismatch'):
        prepare(replay,0,calibration,coords)


@pytest.mark.parametrize('target', [2790, True])
def test_target_and_controller_floor_cannot_disagree(target):
    from tools.run.bench_left_gripper import trial_limits
    with pytest.raises(ValueError,match='limits'):
        trial_limits('target',target_plan(target_raw=target))
    assert trial_limits('target',target_plan(target_raw=2794))==(2794,2950)


def test_trial_report_keeps_operator_observation_independent_of_encoder_travel():
    from tools.diagnostics.compare_gripper_bench import supported_trial
    r=completed_record('target',2802,2798)
    r.update(target_plan=target_plan(),contact_outcome='EFFORT_OR_STALL_UNCONFIRMED')
    for row in [r['before'],r['start_reply'],*r['samples'],r['stop']]:
        row.update(firmware='left-grip-bench-v4',floor_raw=2794,entry_raw=2950)
    r['start_reply'].update(goal=2802,position=2802)
    r['endpoint']['goal']=2796
    r['stop'].update(writes=20,tick_ms=700)
    after=dict(status='observed',operator_label='inspect',identity=r['stop'].copy(),
               before=dict(r['stop'],position=2799,ticket=12,sequence=0,tick_ms=720,status_raw=0,speed_raw=0))
    stationary=supported_trial(r,'appeared_stationary',after)
    tip=supported_trial(r,'tip_contact',after)
    assert stationary['encoder']==tip['encoder'] and stationary['encoder']['travel_raw']==4
    assert stationary['operator_tip_observation']!=tip['operator_tip_observation']
    assert not tip['bilateral_contact_observed'] and not tip['grasp_verified']
    after['before']['speed_raw']=1
    with pytest.raises(ValueError,match='post-stop'):
        supported_trial(r,'tip_contact',after)
    after['before']['speed_raw']=0
    after['before']['writes']+=1
    with pytest.raises(ValueError,match='post-stop'):
        supported_trial(r,'tip_contact',after)


class HoldingPort(BenchPort):
    def write(self,data):
        words=data.decode().split()
        if words[0]=='BEGIN_RIGID':
            super().write(('BEGIN '+' '.join(words[1:4])+'\n').encode())
            self.commands[-1]='BEGIN_RIGID'
        else:
            super().write(data)
        row=json.loads(self.reply)
        row.update(firmware='left-grip-bench-v5',hold_ms=2000,hold_elapsed_ms=0,hold_goal=-1,
                   hold_cause='NONE',writes=5,torque_limit_raw=80)
        if words[0]=='BEGIN_RIGID':
            row['torque']=1
        if words[0]=='STEP':
            seq=int(words[2])
            row.update(active=int(seq<21),torque=int(seq<21),off_verified=int(seq>=21),
                       reason='PRELOADING_TARGET' if seq<18 else ('HOLDING_TARGET' if seq<21 else 'HOLD_COMPLETE'),
                       goal=max(self.floor,2900-6*seq),hold_goal=-1 if seq<18 else self.floor,
                       hold_cause='EFFORT_PERSISTED',hold_elapsed_ms=max(0,seq-18)*700,
                       position=2898,writes=5+min(seq,18)+int(seq>=21))
            if self.fault=='goal_change' and seq>=18:row['goal']=self.floor-1
            if self.fault=='duration_change':row['hold_ms']=0
            if self.fault=='extra_write' and seq>=18:row['writes']+=1
            if self.fault=='early_complete' and seq>=18:
                row.update(active=0,torque=0,off_verified=1,reason='HOLD_COMPLETE',writes=24)
        self.reply=json.dumps(row).encode()+b'\n'
        return len(data)


@pytest.mark.parametrize('fault',['','goal_change','duration_change','extra_write','early_complete'])
def test_finite_hold_keeps_goal_bound_and_releases_after_observation(fault):
    port=HoldingPort(fault)
    report={'samples':[]}
    if fault:
        with pytest.raises(ValueError):
            trial(port,'target',report,target_plan=target_plan(),expected_start=2900,
                  hold_ms=2000,sleep=lambda _:None)
    else:
        trial(port,'target',report,target_plan=target_plan(),expected_start=2900,
              hold_ms=2000,sleep=lambda _:None)
        assert report['endpoint']['reason']=='HOLD_COMPLETE'
        assert report['contact_outcome']=='UNEXPECTED_RESISTANCE'
        assert len(report['samples'])==21 and report['stop']['torque']==0
    assert port.commands[-1]=='STOP'


def test_hold_requires_supported_target_and_v5_before_any_motor_command():
    port=BenchPort()
    with pytest.raises(ValueError,match='v5'):
        trial(port,'target',{},target_plan=target_plan(),expected_start=2900,hold_ms=2000)
    assert port.commands==['INFO']
    port=BenchPort()
    for mode,ms in [('empty',2000),('target',True),('target',400),('target',3001)]:
        with pytest.raises(ValueError,match='500'):
            trial(port,mode,{},hold_ms=ms)
    assert not port.commands


def test_supported_rigid_preload_can_start_at_contact_but_passive_contact_test_needs_an_approach():
    plan=target_plan(upper_contact_raw=2899)
    port=HoldingPort()
    with pytest.raises(ValueError,match='free approach'):
        trial(port,'target',{},target_plan=plan,expected_start=2900)
    assert port.commands==['INFO','BIND']
    report={'samples':[]}
    trial(HoldingPort(),'target',report,target_plan=plan,expected_start=2900,hold_ms=2000,sleep=lambda _:None)
    assert report['endpoint']['reason']=='HOLD_COMPLETE'
