"""Run the deployed profile through the real MCU executor, guard and goal map."""
import ctypes,subprocess,json,os
import pytest
from test_taught_replay import ROOT,BOARD,P,header,play
from so101_arm_bridge.fixed_pick_demo import smooth_leg
from so101_arm_bridge.stream_protocol_v2 import encode_stream_open_v2,encode_stream_batch_v2,BatchKindV2,StreamBatchV2,StreamSampleV2

if os.environ.get('TAUGHT_TEST_CAPTURE'):
    P=play.compile_profile(json.loads(__import__('pathlib').Path(os.environ['TAUGHT_TEST_CAPTURE']).read_text()))

def encode_batch_v2(batch):return encode_stream_batch_v2(batch,BatchKindV2.APPEND)

@pytest.fixture(scope='module')
def runtime(tmp_path_factory):
    out=tmp_path_factory.mktemp('taught-executor');h=out/'profile.h';h.write_text(header(P))
    src=(BOARD/'Src/binary_control.c').read_text();i=src.index('static actuator_v2_stream_hard_caps_t Host_V2HardCaps(void)');j=src.index('\n}\n',i)+3
    begin=src.index('static void Host_ServiceV2TrackingFeedback(void)');anchor_start=src.index('            memcpy(\n                host_v2_shadow_executor_anchor_urad,',begin);anchor_end=src.index('            host_v2_shadow_anchor_ready = 1U;',anchor_start)
    anchor_code=src[anchor_start:anchor_end]
    wrapper='''#include "single_arm_config.h"
#include "bimanual_operational_limits.h"
#include "taught_pose_guard.h"
#include "bimanual_feedback_snapshot.h"
#include "actuator_core/stream_executor_v2.h"
#include <string.h>
static actuator_v2_stream_executor_t e;
'''+src[i:j]+'''
int setup(void){actuator_v2_joint_limit_t limits[12];BimanualOperationalLimits_LoadExecutorLimits(limits);actuator_v2_stream_hard_caps_t c=Host_V2HardCaps();TaughtPoseGuard_Reset();return actuator_v2_stream_executor_init(&e,&c,limits);}
int inverted(void){actuator_v2_joint_limit_t limits[12];BimanualOperationalLimits_LoadExecutorLimits(limits);limits[6].minimum_urad=1;actuator_v2_stream_hard_caps_t c=Host_V2HardCaps();return actuator_v2_stream_executor_init(&e,&c,limits);}
int open_route(const uint8_t *b,unsigned n,unsigned t){return actuator_v2_stream_executor_open(&e,b,n,t);}
int append(const uint8_t *b,unsigned n,unsigned t){return actuator_v2_stream_executor_admit(&e,b,n,ACTUATOR_V2_BATCH_APPEND,t);}
int start(unsigned t,const int32_t *q){return actuator_v2_stream_executor_start(&e,t,q);}
int step(unsigned t,int32_t *q){
 int r=actuator_v2_stream_executor_step(&e,t,q);
 if(r==ACTUATOR_V2_EXECUTOR_OUTPUT_READY||r==ACTUATOR_V2_EXECUTOR_TERMINAL){
  if(!TaughtPoseGuard_Check(q,r==ACTUATOR_V2_EXECUTOR_TERMINAL))return 100;
  uint16_t l[6],right[6];uint8_t failed;
  if(BimanualOperationalLimits_MapExecutorOutput(q,l,right,&failed)!=ACTUATOR_BIMANUAL_GOAL_MAP_OK)return 101;
 }
 return r;
}
'''
    wrapper+='''
void next_anchor(const int32_t *measured,const int32_t *goal,int32_t *out){
 BimanualFeedbackSnapshot snapshot;int32_t host_v2_shadow_executor_anchor_urad[12],host_v2_output_urad[12];
 memcpy(snapshot.positions_urad,measured,sizeof(snapshot.positions_urad));
 memcpy(host_v2_output_urad,goal,sizeof(host_v2_output_urad));
'''+anchor_code+'''
 memcpy(out,host_v2_shadow_executor_anchor_urad,sizeof(host_v2_shadow_executor_anchor_urad));
}
'''
    (out/'runtime.c').write_text(wrapper);core=ROOT/'firmware/stm32_actuator'
    subprocess.run(['gcc','-shared','-fPIC','-std=c11','-Wall','-Wextra','-Werror','-O2','-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1','-DHOST_BIMANUAL_GRIPPER_TERMINAL_SETTLE_BUILD=1',f'-DTAUGHT_POSE_PROFILE_HEADER="{h}"','-I',str(BOARD/'Inc'),'-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(core/'include'),str(out/'runtime.c'),str(BOARD/'Src/taught_pose_guard.c'),str(BOARD/'Src/bimanual_operational_limits.c'),*[str(core/'src'/f'{n}.c') for n in ('stream_executor_v2','stream_contract_v2','stream_session_v2','bimanual_goal_map')],'-o',str(out/'runtime.so')],check=True)
    return ctypes.CDLL(str(out/'runtime.so'))


def test_fixed_inactive_slots_initialize_and_inverted_limits_reject(runtime):
    assert runtime.setup()==0
    assert runtime.inverted()==6


@pytest.mark.parametrize('phase_offset',range(5))
@pytest.mark.parametrize('start_name',['hardware','offset_home','lift','grasp','mixed_bounds'])
@pytest.mark.parametrize('residual_scale',[0,1,2,-2])
def test_real_executor_opens_replays_full_measured_cycle_and_repeat(runtime,residual_scale,start_name,phase_offset):
    assert runtime.setup()==0
    targets=play.targets_for(P);limits=play.limits_for(P)
    # Actual non-exact starting Home reported in the failure log.
    start=tuple(x/1e6 for x in (49087,67495,-53689,-41417,0,-105845))
    if start_name=='hardware':start=tuple(q/1e6 for q in json.loads((ROOT/'tests/fixtures/taught_guard_failure.json').read_text())['initial_anchor_urad'][:6])
    if start_name in ('lift','grasp'):start=targets[P['stage_order'].index(start_name)]
    elif start_name=='mixed_bounds':start=tuple(b['minimum_urad' if j%2 else 'maximum_urad']/1e6 for j,b in enumerate(P['bounds']))
    tick=1000;resume=None;wire=play.TaughtWire(None,P,motion_authorized=False)
    for index in (0,1,2,3,4,4,3,2,1,0,1,2,3,4):
        points=smooth_leg(start,targets[index],limits);base=tick+100+phase_offset;horizon=base+points[-1].offset_ms
        terminal_tick=horizon+(tick-horizon)%5
        policy=encode_stream_open_v2(wire._policy(horizon_end_tick=horizon));assert runtime.open_route(policy,len(policy),tick)==0
        samples=[StreamSampleV2(base+p.offset_ms,tuple(round(x*1e6) for x in p.positions_rad)+(0,)*6) for p in points]
        cursor=0
        def feed():
            nonlocal cursor
            while cursor<len(samples) and samples[cursor].apply_tick<=tick+350:
                b=encode_batch_v2(StreamBatchV2(horizon_end_tick=horizon,arbiter_epoch=1,samples=(samples[cursor],),arm_mask=1))
                assert runtime.append(b,len(b),tick)==0,(index,cursor,tick)
                cursor+=1
        feed();anchor=(ctypes.c_int32*12)(*[round(x*1e6) for x in start],*([0]*6));assert runtime.start(tick,anchor if resume is None else resume)==0
        out=(ctypes.c_int32*12)();previous=list(anchor if resume is None else resume)
        while tick<terminal_tick:
            tick+=5;feed();result=runtime.step(tick,out)
            assert result==(3 if tick==terminal_tick else 2),(index,tick,result)
            assert list(out)[6:]==[0]*6
            assert max(abs(a-b) for a,b in zip(out,previous))<=1001,(index,tick,list(out),previous)
            previous=list(out)
        assert list(out)[:6]==P['targets_urad'][index]
        # Reproduce the measured Home residual from the hardware failure;
        # the motors still hold the previous goal between finite segments.
        residual=(6136,0,-12272,-9204,0,-9204)
        measured=(ctypes.c_int32*12)(*[q+residual_scale*d for q,d in zip(P['targets_urad'][index],residual)],*([0]*6))
        resume=(ctypes.c_int32*12)();runtime.next_anchor(measured,out,resume)
        start=targets[index];tick+=50


def test_reserved_axis_nonzero_rejected_by_real_executor(runtime):
    assert runtime.setup()==0
    wire=play.TaughtWire(None,P,motion_authorized=False)
    p=encode_stream_open_v2(wire._policy(horizon_end_tick=1200));assert runtime.open_route(p,len(p),1000)==0
    b=encode_batch_v2(StreamBatchV2(horizon_end_tick=1200,arbiter_epoch=1,samples=(StreamSampleV2(1100,(0,)*6+(1,)+(0,)*5),),arm_mask=1))
    assert runtime.append(b,len(b),1000)==9


def test_exact_captured_hardware_home_timeline_has_no_velocity_spike(runtime):
    capture=json.loads((ROOT/'tests/fixtures/taught_guard_failure.json').read_text())
    assert runtime.setup()==0
    initial=capture['initial_anchor_urad'];start=tuple(q/1e6 for q in initial[:6])
    points=smooth_leg(start,play.targets_for(P)[0],play.limits_for(P))
    base=43077;epoch=43095;horizon=base+points[-1].offset_ms
    assert horizon==44477
    wire=play.TaughtWire(None,P,motion_authorized=False)
    p=encode_stream_open_v2(wire._policy(horizon_end_tick=horizon));assert runtime.open_route(p,len(p),epoch)==0
    samples=[StreamSampleV2(base+x.offset_ms,tuple(round(q*1e6) for q in x.positions_rad)+(0,)*6) for x in points]
    cursor=0;out=(ctypes.c_int32*12)();previous=initial[:]
    for tick in range(epoch,horizon+(epoch-horizon)%5+1,5):
        while cursor<len(samples) and samples[cursor].apply_tick<=tick+350:
            b=encode_batch_v2(StreamBatchV2(horizon_end_tick=horizon,arbiter_epoch=1,samples=(samples[cursor],),arm_mask=1))
            assert runtime.append(b,len(b),tick)==0;cursor+=1
        if tick==epoch:assert runtime.start(tick,(ctypes.c_int32*12)(*initial))==0
        result=runtime.step(tick,out)
        assert result in (2,3),(tick,result,list(out),previous)
        assert max(abs(a-b) for a,b in zip(out,previous))<=1001,(tick,list(out),previous)
        previous=list(out)
    assert result==3 and list(out)==[0]*12
