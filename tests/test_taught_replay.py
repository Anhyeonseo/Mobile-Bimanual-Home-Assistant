import copy,ctypes,importlib.util,json,subprocess,sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/run'))
sys.path.insert(0,str(ROOT/'tools/setup/firmware'))
import play_left_demo as play
from generate_taught_profile import header
from pose_locked_profile import compile_profile
from so101_arm_bridge.fixed_pick_demo import smooth_leg,execute_leg,OWNER
from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint,BimanualStreamError
from test_left_only_pick import LeftTransport
from test_filming_workflows import Clock
P=compile_profile(json.loads((ROOT/'tests/fixtures/taught_capture.json').read_text()))
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'

@pytest.fixture(scope='module')
def native(tmp_path_factory):
    out=tmp_path_factory.mktemp('taught');h=out/'profile.h';h.write_text(header(P))
    subprocess.run(['gcc','-shared','-fPIC','-O2','-Wall','-Wextra','-Werror','-I',str(BOARD/'Inc'),f'-DTAUGHT_POSE_PROFILE_HEADER="{h}"',str(BOARD/'Src/taught_pose_guard.c'),'-o',str(out/'guard.so')],check=True)
    lib=ctypes.CDLL(str(out/'guard.so'));lib.TaughtPoseGuard_Check.argtypes=[ctypes.POINTER(ctypes.c_int32),ctypes.c_bool];lib.TaughtPoseGuard_Check.restype=ctypes.c_bool
    return lib,h,out

def qcheck(lib,q,terminal=False):return lib.TaughtPoseGuard_Check((ctypes.c_int32*12)(*q,*([0]*(12-len(q)))),terminal)
def segment(a,b):
    points=smooth_leg(a,b,play.limits_for(P))
    yield [round(x*1e6) for x in a],False
    for lo,hi in zip(points,points[1:]):
        count=(hi.offset_ms-lo.offset_ms)//5
        for step in range(1,count+1):
            fraction=step/count
            yield [round((x+(y-x)*fraction)*1e6) for x,y in zip(lo.positions_rad,hi.positions_rad)],hi is points[-1] and step==count

def test_actual_saved_route_return_repeat_and_sixty_second_hold(native):
    lib,_,_=native;lib.TaughtPoseGuard_Reset();targets=play.targets_for(P);current=0
    for target in (0,1,2,3,4,4,3,2,1,0,1,2,3,4):
        for q,terminal in segment(targets[current],targets[target]):assert qcheck(lib,q,terminal),(current,target,q)
        current=target
        if target==4:
            for _ in range(12000):assert qcheck(lib,P['targets_urad'][4],True)

@pytest.mark.parametrize('defect',['right_axis','speed','wrong_start','outside_path','skip_pose','midpoint_terminal','reverse_midleg'])
def test_mcu_rejects_path_bypass_and_latches_rejection(native,defect):
    lib,_,_=native;lib.TaughtPoseGuard_Reset();home=P['targets_urad'][0];assert qcheck(lib,home,True)
    if defect=='right_axis':bad=home+[1]*6
    elif defect=='speed':bad=home[:];bad[0]+=1101
    elif defect=='wrong_start':
        lib.TaughtPoseGuard_Reset();bad=P['targets_urad'][0][:];bad[0]=P['bounds'][0]['maximum_urad']+1
    elif defect=='outside_path':
        bad=home[:]
        for amount in range(1000,13001,1000):
            bad[4]=amount
            if amount<13000:assert qcheck(lib,bad)
    elif defect=='skip_pose':
        bad=None
        for q,t in segment(play.targets_for(P)[0],play.targets_for(P)[2]):
            if not qcheck(lib,q,t):break
        else:pytest.fail('Non-adjacent segment accepted')
    elif defect in ('midpoint_terminal','reverse_midleg'):
        samples=list(segment(play.targets_for(P)[0],play.targets_for(P)[1]));middle=len(samples)//2
        for q,t in samples[:middle]:assert qcheck(lib,q,t)
        if defect=='midpoint_terminal':bad=samples[middle][0]
        else:bad=samples[middle-2][0]
    if defect!='skip_pose':assert not qcheck(lib,bad,defect=='midpoint_terminal')
    assert not qcheck(lib,home,True)

def test_prearm_checks_all_offsets_and_home_before_torque(native):
    _,h,out=native
    source=(BOARD/'Src/binary_control.c').read_text();start=source.index('static uint8_t Host_ConfigureBimanualForTrajectory(void)');end=source.index('\n}\n',start)+3
    harness='''#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "profile.h"
#define HAL_OK 0
#define HOST_LEFT_ONLY_BUILD 1
static unsigned host_v2_executor_ready=1;
static struct { unsigned active; } host_binary_motion;
static struct { struct { int state; } diagnostics; } host_v2_stream_executor;
enum {ACTUATOR_V2_EXECUTOR_CLOSED, ACTUATOR_V2_EXECUTOR_PRIMING, ACTUATOR_V2_EXECUTOR_RUNNING, ACTUATOR_V2_EXECUTOR_HOLD};
static unsigned host_v2_executor_clock_active;
static int Host_BufferedExecutionIsActive(void){return 0;}
static int bad_offset=-1,bad_pose=-1,read_fail=0,enabled=0,start_pose=0;
static uint32_t host_bimanual_arm_watchdog_grace_started_ms;
static unsigned host_right_arm_output_active;
static int Servo_ReadAllPositions(uint16_t *p){memcpy(p,taught_raw[start_pose],12);if(bad_pose>=0)p[bad_pose]=taught_maximum_raw[bad_pose]+1;return read_fail;}
static int Servo_ReadData(unsigned id,unsigned a,unsigned n,uint8_t *b){assert(a==31&&n==2);uint16_t v=taught_offsets[id-1];if((int)id-1==bad_offset)v++;b[0]=v;b[1]=v>>8;return read_fail;}
static int Servo_ConfigureAllForTrajectory(uint16_t *p){(void)p;enabled++;return 0;}
static int Servo_DisableTorqueAll(void){return 0;}
static int MobileBoard_StopActive(void){return 0;}
static int MobileBoard_ArmsPrepared(void){return 1;}
static uint32_t HAL_GetTick(void){return 1;}
static void TaughtPoseGuard_Reset(void){}
'''+source[start:end]+'''
int main(void){
 host_v2_executor_ready=0;assert(!Host_ConfigureBimanualForTrajectory()&&enabled==0);host_v2_executor_ready=1;
 for(int j=0;j<6;j++){bad_offset=j;assert(!Host_ConfigureBimanualForTrajectory()&&enabled==0);}bad_offset=-1;
 for(int j=0;j<6;j++){bad_pose=j;assert(!Host_ConfigureBimanualForTrajectory()&&enabled==0);}bad_pose=-1;
 read_fail=1;assert(!Host_ConfigureBimanualForTrajectory()&&enabled==0);read_fail=0;
 for(start_pose=0;start_pose<5;start_pose++)assert(Host_ConfigureBimanualForTrajectory());
 assert(enabled==5&&host_right_arm_output_active==0);return 0;}
'''
    (out/'arm.c').write_text(harness)
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',f'-DTAUGHT_POSE_PROFILE_HEADER="{h}"',str(out/'arm.c'),'-o',str(out/'arm')],check=True)
    subprocess.run([str(out/'arm')],check=True)

class Transport(LeftTransport):
    def enter_binary_mode(self):
        h=super().enter_binary_mode();h.firmware_version=play.FIRMWARE;h.left_calibration_hash=h.right_calibration_hash=P['firmware_calibration_hash'];return h
    def arm(self,hash):
        assert hash==P['firmware_calibration_hash'];self.calls.append('arm')

def setup():
    clock=Clock();tr=Transport(clock,play.targets_for(P)[0]);a=play.TaughtAdapter(tr,P,motion_authorized=True);a.prepare();return clock,tr,a

def test_host_real_adapter_full_cycle_and_holds():
    clock,tr,a=setup();targets=play.targets_for(P);limits=play.limits_for(P);last=None
    for target in (0,1,2,3,4,4,3,2,1,0,1,2,3,4):
        execute_leg(a,targets[target],limits,commanded_start=last,clock=clock,sleep=clock.advance)
        last=targets[target]
    assert a.heartbeat_required
    assert a._wire._config.terminal_settle_gripper_tolerance_urad==22242
    a.stop(OWNER);assert tr.safe_stop_count==1

@pytest.mark.parametrize('defect',['skip','arbitrary','speed','trajectory','nonfinite'])
def test_host_rejects_non_recipe_commands_before_arm(defect):
    _,tr,a=setup();targets=play.targets_for(P);limits=play.limits_for(P)
    points=list(smooth_leg(targets[0],targets[2] if defect=='skip' else targets[1],limits));finite=True
    if defect=='arbitrary':points[-1]=TimedJointPoint(points[-1].offset_ms,tuple(x+.01 for x in points[-1].positions_rad))
    if defect=='speed':points[1]=TimedJointPoint(points[1].offset_ms+5,points[1].positions_rad)
    if defect=='trajectory':points[1]=TimedJointPoint(points[1].offset_ms,tuple(x+.002 for x in points[1].positions_rad))
    if defect=='nonfinite':finite=False
    with pytest.raises(ValueError):a.start(OWNER,points,finite=finite)
    assert 'arm' not in tr.calls

def test_binding_rejects_changed_profile(tmp_path):
    (tmp_path/'config').mkdir();cap=json.loads((ROOT/'tests/fixtures/taught_capture.json').read_text())
    for name,value in [('left-five-poses.json',cap),('profile.json',P),('firmware-binding.json',dict(firmware_version=play.FIRMWARE,profile_sha256=P['profile_sha256'],calibration_hash=P['firmware_calibration_hash']))]:
        (tmp_path/'config'/name).write_text(json.dumps(value))
    assert play.bundle(tmp_path)==P
    cap['poses']['home']['captured_at']='changed';(tmp_path/'config/left-five-poses.json').write_text(json.dumps(cap))
    with pytest.raises(ValueError):play.bundle(tmp_path)

@pytest.mark.parametrize('sign',[-1,1])
def test_loose_home_alignment_then_full_route(native,sign):
    lib,_,_=native;lib.TaughtPoseGuard_Reset();targets=play.targets_for(P)
    start=tuple(x+sign*.13 for x in targets[0])
    for q,t in segment(start,targets[0]):assert qcheck(lib,q,t)
    for q,t in segment(targets[0],targets[1]):assert qcheck(lib,q,t)
    clock=Clock();tr=Transport(clock,start);a=play.TaughtAdapter(tr,P,motion_authorized=True);a.prepare()
    execute_leg(a,targets[0],play.limits_for(P),clock=clock,sleep=clock.advance)
    execute_leg(a,targets[1],play.limits_for(P),commanded_start=targets[0],clock=clock,sleep=clock.advance)
    a.stop(OWNER)


def test_initial_home_rejects_outside_profile_bounds_and_outward_move(native):
    lib,_,_=native;lib.TaughtPoseGuard_Reset()
    bad=P['targets_urad'][0][:];bad[0]=P['bounds'][0]['maximum_urad']+1
    assert not qcheck(lib,bad)
    lib.TaughtPoseGuard_Reset();assert qcheck(lib,[130000]*6)
    assert not qcheck(lib,[130100]*6)
    clock=Clock();tr=Transport(clock,tuple(x/1e6 for x in bad));a=play.TaughtAdapter(tr,P,motion_authorized=True)
    with pytest.raises((ValueError,BimanualStreamError)):
        a.prepare();a.refresh_unarmed_anchor()
    assert 'arm' not in tr.calls


def test_native_actual_pose_mapping_and_loose_bounds(native):
    _,h,out=native
    code='''#include "bimanual_operational_limits.h"
#include "profile.h"
#include <assert.h>
#include <stdlib.h>
int main(void){
 int32_t q[12]={0};uint16_t l[6],r[6];uint8_t failed;
 for(unsigned p=0;p<5;p++){
  for(unsigned j=0;j<6;j++)q[j]=taught_targets[p][j];
  assert(BimanualOperationalLimits_MapExecutorOutput(q,l,r,&failed)==ACTUATOR_BIMANUAL_GOAL_MAP_OK);
  for(unsigned j=0;j<6;j++)assert(abs((int)l[j]-(int)taught_raw[p][j])<=1);
 }
 for(unsigned j=0;j<6;j++){
  const BimanualOperationalLimit *b=BimanualOperationalLimits_Get(BIMANUAL_ARM_LEFT,j);
  int32_t result;
  assert(BimanualOperationalLimits_UnwrapModuloRaw(BIMANUAL_ARM_LEFT,j,b->raw.minimum_unwrapped_raw,&result));
  assert(result==b->raw.minimum_unwrapped_raw);
  assert(BimanualOperationalLimits_UnwrapModuloRaw(BIMANUAL_ARM_LEFT,j,b->raw.maximum_unwrapped_raw,&result));
  assert(result==b->raw.maximum_unwrapped_raw);
 }
 return 0;
}'''
    (out/'map.c').write_text(code)
    core=ROOT/'firmware/stm32_actuator'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1',f'-DTAUGHT_POSE_PROFILE_HEADER="{h}"','-I',str(BOARD/'Inc'),'-I',str(core/'include'),str(out/'map.c'),str(BOARD/'Src/bimanual_operational_limits.c'),str(core/'src/bimanual_goal_map.c'),'-o',str(out/'map')],check=True)
    subprocess.run([str(out/'map')],check=True)

@pytest.mark.parametrize('initial_stage',[1,2,3,4])
def test_host_returns_current_pose_to_home_before_pick(initial_stage):
    clock=Clock();targets=play.targets_for(P);tr=Transport(clock,targets[initial_stage]);a=play.TaughtAdapter(tr,P,motion_authorized=True)
    a.prepare();execute_leg(a,targets[0],play.limits_for(P),clock=clock,sleep=clock.advance)
    execute_leg(a,targets[1],play.limits_for(P),commanded_start=targets[0],clock=clock,sleep=clock.advance)
    assert a.heartbeat_required; a.stop(OWNER)


def test_complete_cycle_survives_transient_feedback_bus_ownership():
    clock,tr,a=setup();original=tr.get_dispatch_diagnostics;calls=0
    def alternating_busy():
        nonlocal calls
        calls+=1;value=original();value.ready=(calls%2==0);value.active=False
        return value
    tr.get_dispatch_diagnostics=alternating_busy
    targets=play.targets_for(P);last=None
    for index in (0,1,2,3,4,4,3,2,1,0):
        execute_leg(a,targets[index],play.limits_for(P),commanded_start=last,clock=clock,sleep=clock.advance);last=targets[index]
    assert tr.safe_stop_count==0;a.stop(OWNER)

@pytest.mark.parametrize('blocked_open',[False,True])
def test_contact_preload_survives_lift_but_open_jaw_is_rejected(blocked_open):
    from dataclasses import replace
    from pose_locked_profile import gripper_tolerance_urad
    record=json.loads((ROOT/'tests/fixtures/taught_capture.json').read_text())
    record['gripper_close_extra_raw']=20;p=compile_profile(record)
    targets=play.targets_for(p);clock=Clock()
    contact=0 if blocked_open else P['targets_urad'][3][5]
    class ContactTransport(LeftTransport):
        def enter_binary_mode(self):
            h=super().enter_binary_mode();h.firmware_version=play.FIRMWARE
            h.left_calibration_hash=h.right_calibration_hash=p['firmware_calibration_hash'];return h
        def arm(self,hash):
            assert hash==p['firmware_calibration_hash'];self.calls.append('arm')
        def get_feedback_snapshot(self):
            snapshot=super().get_feedback_snapshot();q=list(snapshot.positions_urad)
            q[5]=max(q[5],contact)
            return replace(snapshot,positions_urad=tuple(q))
    tr=ContactTransport(clock,targets[0]);a=play.TaughtAdapter(tr,p,motion_authorized=True);a.prepare()
    assert gripper_tolerance_urad(p)==49087
    assert '#define TAUGHT_GRIPPER_TOLERANCE_URAD 49087' in header(p)
    last=None
    for stage in (0,1,2):
        execute_leg(a,targets[stage],play.limits_for(p),commanded_start=last,clock=clock,sleep=clock.advance);last=targets[stage]
    if blocked_open:
        with pytest.raises(BimanualStreamError,match='terminal settle'):
            execute_leg(a,targets[3],play.limits_for(p),commanded_start=last,clock=clock,sleep=clock.advance)
        assert tr.safe_stop_count>0
        return
    for stage in (3,4,4,3,2,1,0):
        execute_leg(a,targets[stage],play.limits_for(p),commanded_start=last,clock=clock,sleep=clock.advance)
        if stage==4:
            assert tr.route[-1].positions_urad[5]==p['targets_urad'][4][5]
            assert tr.get_feedback_snapshot().positions_urad[5]==contact
        last=targets[stage]
    a.stop(OWNER)
