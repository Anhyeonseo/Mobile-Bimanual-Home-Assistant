import copy
import json
from pathlib import Path
import subprocess
import sys

import pytest
from so101_arm_bridge.development_ranges import (
    read_development_ranges, validate_development_ranges, unwrap_in_range,
    shortest_target, operational_limits_for_simulation, calibration_hash,
    point_move_samples,
)
from so101_arm_bridge.bimanual_stream_adapter import ResidentBimanualStreamAdapter, BimanualStreamContractError, TimedJointPoint

ROOT=Path(__file__).resolve().parents[1]
PROFILE=ROOT/'config/home_assistant/arm_ranges.development.json'
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'


def test_generated_outputs_match_canonical_profile():
    subprocess.run([sys.executable,str(ROOT/'tools/setup/firmware/generate_development_ranges.py'),'--check'],check=True)
    _,entries,digest=read_development_ranges(PROFILE)
    assert len(entries)==12 and calibration_hash(digest)!=0x2D90167E


@pytest.mark.parametrize('mutation',[
    lambda d:d.update(hardware_enabled=True),
    lambda d:d.update(firmware_limit_authorized=True),
    lambda d:d.update(zero_raw=True),
    lambda d:d['arms']['right']['base'].update(measured_on_this_arm=True),
    lambda d:d['arms']['right']['base'].update(source_report_sha256='0'*64),
    lambda d:d['arms']['left']['wrist_roll'].update(maximum_unwrapped_raw=4000),
    lambda d:d['arms']['left']['elbow'].update(positive_raw_direction=1),
    lambda d:d['arms']['left']['elbow'].update(minimum_urad=123),
    lambda d:d['calibration']['left'].update(offsets_raw=[1]*5),
])
def test_rejects_profile_drift_and_false_provenance(mutation):
    d=json.loads(PROFILE.read_text());mutation(d)
    with pytest.raises(ValueError):validate_development_ranges(d)


def test_all_encoder_branches_round_trip_without_wrapping_endpoints():
    _,entries,_=read_development_ranges(PROFILE)
    for entry in entries:
        for raw in range(entry['minimum_unwrapped_raw'],entry['maximum_unwrapped_raw']+1):
            assert unwrap_in_range(entry,raw%4096)==raw


def test_point_targets_take_nearest_valid_branch_or_reject_never_detour():
    _,entries,_=read_development_ranges(PROFILE)
    roll,shoulder=entries[4],entries[1]
    assert shortest_target(roll,200,500)==500
    assert shortest_target(roll,500,200)==200
    assert shortest_target(shoulder,4090,10,allow_seam=True)==4106
    with pytest.raises(ValueError,match='seam'):
        shortest_target(shoulder,4090,10)
    with pytest.raises(ValueError,match='no long-way fallback'):
        shortest_target(roll,2900,3400) # -696 is in the envelope but is the long way.
    with pytest.raises(ValueError,match='half turn'):
        shortest_target(roll,200,2248)


def test_host_simulation_uses_new_ranges_and_rejects_hardware_opt_in():
    limits=operational_limits_for_simulation(PROFILE)
    with pytest.raises(BimanualStreamContractError,match='cannot authorize'):
        ResidentBimanualStreamAdapter(object(),limits,motion_authorized=True)
    adapter=ResidentBimanualStreamAdapter(object(),limits)
    # Entire new angular vector enters the real host preflight without a robot.
    endpoint=tuple(v/1_000_000 for v in limits.minimum_urad)
    assert adapter._validate_points([TimedJointPoint(100,endpoint)],first_reference_urad=None,first_span_ms=100)
    invalid=list(endpoint);invalid[4]-=.1
    with pytest.raises(BimanualStreamContractError,match='outside approved range'):
        adapter._validate_points([TimedJointPoint(100,tuple(invalid))],first_reference_urad=None,first_span_ms=100)


def test_left_adapter_preserves_development_only_restriction():
    from so101_arm_bridge.left_only_adapter import LeftOnlyAdapter, left_limits
    limits=left_limits(operational_limits_for_simulation(PROFILE))
    with pytest.raises(BimanualStreamContractError,match='cannot authorize'):
        LeftOnlyAdapter(object(),limits,motion_authorized=True)


def test_point_move_runs_through_real_host_preflight_without_long_rotation():
    _,entries,_=read_development_ranges(PROFILE)
    current=[2048]*12;target=current.copy();current[4]=200;target[4]=500
    samples=point_move_samples(entries,current,target)
    adapter=ResidentBimanualStreamAdapter(object(),operational_limits_for_simulation(PROFILE))
    converted=adapter._validate_finite_plan_points(samples)
    values=[p[1][4] for p in converted]
    assert all(a<=b for a,b in zip(values,values[1:]))
    assert abs((values[-1]-values[0])*4096/6283185-300)<.001
    assert all(p.positions_rad[:4]==(0,)*4 and p.positions_rad[5:]==(0,)*7 for p in samples)
    with pytest.raises(ValueError,match='speed'):
        point_move_samples(entries,current,target,duration_ms=5)


def test_actual_c_maps_every_profile_position_and_guards_output_transitions(tmp_path):
    harness=tmp_path/'ranges.c'
    harness.write_text(r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bimanual_operational_limits.h"
int main(void) {
    unsigned count=0;
    for(unsigned i=0;i<12;i++) {
        const BimanualOperationalLimit *l=BimanualOperationalLimits_Get(i/6,i%6);
        for(int32_t raw=l->raw.minimum_unwrapped_raw;raw<=l->raw.maximum_unwrapped_raw;raw++) {
            int32_t a[12]={0},b[12]={0},roundtrip=0;
            uint16_t left[6],right[6]; uint8_t failed=255;
            assert(actuator_unwrapped_raw_to_urad(2048,l->positive_raw_direction,raw,&a[i])==ACTUATOR_UNWRAP_OK);
            assert(BimanualOperationalLimits_MapExecutorOutput(a,left,right,&failed)==ACTUATOR_BIMANUAL_GOAL_MAP_OK);
            uint16_t mod=i<6?left[i]:right[i-6];
            assert(BimanualOperationalLimits_UnwrapModuloRaw(i/6,i%6,mod,&roundtrip));
            assert(roundtrip==raw);
            if(raw<l->raw.maximum_unwrapped_raw) {
                memcpy(b,a,sizeof(b));
                assert(actuator_unwrapped_raw_to_urad(2048,l->positive_raw_direction,raw+1,&b[i])==ACTUATOR_UNWRAP_OK);
                assert(BimanualOperationalLimits_CheckTransition(a,b,8,true,&failed));
                assert(BimanualOperationalLimits_CheckTransition(b,a,8,true,&failed));
                bool crosses=(raw%4096==4095 || (raw+1)%4096==0);
                assert(BimanualOperationalLimits_CheckTransition(a,b,8,false,&failed)==!crosses);
            }
            if(raw+9<=l->raw.maximum_unwrapped_raw) {
                memcpy(b,a,sizeof(b));
                assert(actuator_unwrapped_raw_to_urad(2048,l->positive_raw_direction,raw+9,&b[i])==ACTUATOR_UNWRAP_OK);
                assert(!BimanualOperationalLimits_CheckTransition(a,b,8,true,&failed));
                assert(failed==i);
            }
            count++;
        }
        int32_t invalid[12]={0};uint16_t left[6],right[6];uint8_t failed=255;
        for(unsigned j=0;j<6;j++)left[j]=right[j]=123;
        assert(actuator_unwrapped_raw_to_urad(2048,l->positive_raw_direction,l->raw.maximum_unwrapped_raw+1,&invalid[i])==ACTUATOR_UNWRAP_OK);
        assert(BimanualOperationalLimits_MapExecutorOutput(invalid,left,right,&failed)==ACTUATOR_BIMANUAL_GOAL_MAP_LIMIT_VIOLATION);
        for(unsigned j=0;j<6;j++)assert(left[j]==123 && right[j]==123);
    }
    printf("%u encoder positions and transitions verified\n",count);
}
''')
    exe=tmp_path/'ranges'
    subprocess.run(['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror',
        '-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1',
        '-DS1_ARM_RANGE_PROFILE_HEADER="generated_s1_arm_ranges.h"',
        '-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),str(harness),
        str(BOARD/'Src/bimanual_operational_limits.c'),str(CORE/'src/bimanual_goal_map.c'),str(CORE/'src/joint_unwrap.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def test_verification_transport_blocks_every_single_byte_write_on_both_buses(tmp_path):
    fake=ROOT/'tests/fixtures/servo_hal'
    source=(fake/'left_calibration_gate.c').read_text()
    source=source.replace('assert(!ServoTransport_Register(&other));','assert(ServoTransport_Register(&other));')
    source=source.replace(' for(unsigned id=',' for(unsigned bus=0;bus<2;bus++)\n for(unsigned id=')
    for function in ('BeginService','Transmit','End'):
        source=source.replace('ServoTransport_'+function+'(&huart1,','ServoTransport_'+function+'(bus?&other:&huart1,')
    source=source.replace('int allowed=id>=1&&id<=6&&address==40&&(value==0||value==128);','int allowed=0;')
    source=source.replace('assert(other.sent==0);',r'''
 assert(other.sent==0 && huart1.sent==0);
 for(unsigned bus=0;bus<2;bus++) {
  UART_HandleTypeDef *u=bus?&other:&huart1;
  uint8_t read[8]={255,255,1,4,2,56,2,0};
  unsigned sum=0;for(unsigned i=2;i<7;i++)sum+=read[i];read[7]=~sum;
  uint32_t token;
  assert(ServoTransport_BeginService(u,ACTUATOR_BUS_WORK_FEEDBACK,&token));
  assert(ServoTransport_TransmitDMA(u,token,read,8)==HAL_OK);
  assert(ServoTransport_TransmitIT(u,token,read,8)==HAL_ERROR);
  assert(ServoTransport_End(u,token,false)==HAL_OK);
  read[7]^=1;
  assert(ServoTransport_BeginService(u,ACTUATOR_BUS_WORK_FEEDBACK,&token));
  assert(ServoTransport_Transmit(u,token,read,8,10)==HAL_ERROR);
  assert(ServoTransport_End(u,token,false)==HAL_OK);
 }
 assert(other.sent==1 && huart1.sent==1);
''')
    path=tmp_path/'gate.c';path.write_text(source);exe=tmp_path/'gate'
    subprocess.run(['gcc','-O2','-std=c11','-Wall','-Wextra','-Werror','-DHOST_S1_RANGE_VERIFY_ONLY=1',
        '-I',str(fake),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),str(path),
        str(BOARD/'Src/servo_transport.c'),str(CORE/'src/bus_router.c'),str(CORE/'src/shared_bus.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=30)
