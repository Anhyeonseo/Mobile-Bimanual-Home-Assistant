"""Execute the real left-only MCU transport/feedback implementations on the PC."""
from pathlib import Path
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'firmware/stm32_g474_single_arm/Core'
CORE=ROOT/'firmware/stm32_actuator'
FAKE=ROOT/'tests/fixtures/servo_hal'

def compile_native(output,sources):
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
        '-DHOST_LEFT_ONLY_BUILD=1U','-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U',
        '-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),
        *map(str,sources),'-o',str(output)],check=True)

@pytest.fixture(scope='module')
def dispatch(tmp_path_factory):
    output=tmp_path_factory.mktemp('left-dispatch')/'test'
    compile_native(output,[FAKE/'left_dispatch_harness.c',BOARD/'Src/servo_transport.c',
        BOARD/'Src/left_servo_dispatch.c',*[CORE/'src'/f'{n}.c' for n in
        ('motor_groups','sts3215_packet','shared_bus','bus_router')]])
    return output

@pytest.mark.parametrize('scenario',range(7))
def test_left_bus_dispatch_isolation_and_faults(dispatch,scenario):
    subprocess.run([str(dispatch),str(scenario)],check=True)

def test_feedback_reads_only_left_and_preserves_missing_right_bits(tmp_path):
    binary=tmp_path/'feedback'
    compile_native(binary,[FAKE/'left_tracking_harness.c',BOARD/'Src/left_tracking_feedback.c',
        BOARD/'Src/bimanual_feedback_snapshot.c'])
    subprocess.run([str(binary)],check=True)

def test_real_shadow_capture_and_terminal_settle_need_only_six_present_joints(tmp_path):
    source=(BOARD/'Src/binary_control.c').read_text()
    functions=[]
    for name in ('Host_V2JointLimits','Host_SendV2ShadowSnapshot','Host_ServiceV2TrackingFeedback'):
        begin=source.index('static void '+name+'(')
        end=source.index('\n}\n',begin)+3
        functions.append(source[begin:end])
    harness=tmp_path/'control.c'
    harness.write_text((FAKE/'left_control_prefix.c').read_text()+'\n'+'\n'.join(functions)+r'''
int main(void){
 actuator_v2_joint_limit_t limits[12];Host_V2JointLimits(limits);
 for(unsigned j=0;j<6;j++)assert(limits[j].minimum_urad<limits[j].maximum_urad);
 for(unsigned j=6;j<12;j++)assert(limits[j].minimum_urad==0&&limits[j].maximum_urad==0);
 actuator_frame_t request={0};request.sequence=9;
 Host_SendV2ShadowSnapshot(&request);
 assert(response.payload[0]==0&&response.payload[1]==6&&response.payload[2]==0x3f&&response.payload[3]==0);
 assert(host_v2_shadow_anchor_ready&&stop_count==0);
 for(unsigned j=6;j<12;j++)assert(host_v2_shadow_raw[j]==0&&host_v2_shadow_anchor_urad[j]==0&&!host_v2_shadow_unwrappers[j].bound);
 /* A real shoulder encoder wrap stays on its commissioned branch. */
 physical[1]=50;Host_SendV2ShadowSnapshot(&request);
 assert(response.payload[0]==0&&host_v2_shadow_unwrapped_raw[1]==4146);
 /* Two complete six-axis sweeps allow finite completion with right absent. */
 tracking.active=1;tracking.completed_pairs=12;next_result=BIMANUAL_TRACKING_IDLE;
 host_v2_terminal_settle_active=1;host_v2_terminal_settle_started_ms=100;
 Host_ServiceV2TrackingFeedback(true);assert(!host_v2_terminal_settle_active&&!tracking.active&&stop_count==0);
 /* Missing LEFT feedback must still stop; do not relax the completeness check. */
 tracking.active=1;host_v2_terminal_settle_active=1;BimanualFeedbackSnapshot_InvalidatePair(1);
 Host_ServiceV2TrackingFeedback(true);assert(stop_count==1);
 /* Right values never get checked as physical feedback. Left error resets settle. */
 tracking.active=1;next_result=BIMANUAL_TRACKING_SAMPLE_READY;
 next_sample.joint_index=0;next_sample.left_position_raw=60000;next_sample.left_commanded_urad=0;
 next_sample.right_commanded_urad=999999;next_sample.right_position_raw=0;
 Host_ServiceV2TrackingFeedback(true);assert(host_v2_terminal_settle_baseline_completed_pairs==12);
 disable_fails=1;Host_SendV2ShadowSnapshot(&request);assert(response.payload[0]==2&&fault_count==1);
 disable_fails=0;read_fails=1;Host_SendV2ShadowSnapshot(&request);assert(response.payload[0]==4&&fault_count==2);
 return 0;
}
''')
    binary=tmp_path/'control'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
        '-DHOST_LEFT_ONLY_BUILD=1U','-DHOST_BIMANUAL_DMA_DISPATCH_BUILD=1U',
        '-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U',
        '-DHOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD=1U',
        '-DHOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U',
        '-DHOST_BIMANUAL_RESIDENT_FINITE_BUILD=1U','-DHOST_BIMANUAL_TERMINAL_SETTLE_BUILD=1U',
        '-I',str(FAKE),'-I',str(BOARD/'Inc'),'-I',str(CORE/'include'),str(harness),
        str(BOARD/'Src/bimanual_feedback_snapshot.c'),str(BOARD/'Src/bimanual_operational_limits.c'),
        str(CORE/'src/joint_unwrap.c'),str(CORE/'src/bimanual_goal_map.c'),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
