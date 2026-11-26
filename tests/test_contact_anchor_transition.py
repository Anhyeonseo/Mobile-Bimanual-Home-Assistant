"""Reproduce the close -> probe failure with the MCU's real anchor/start code."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]

def test_contact_residual_does_not_rebase_the_held_gripper_goal(tmp_path):
    board=ROOT/'firmware/stm32_g474_single_arm/Core';core=ROOT/'firmware/stm32_actuator'
    source=(board/'Src/binary_control.c').read_text();begin=source.index('static bool Host_PrepareArmMotionReference(void)');end=source.index('\n}\n',begin)+3
    harness=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bimanual_operational_limits.h"
#include "actuator_core/joint_unwrap.h"
#include "actuator_core/stream_executor_v2.h"
static int32_t host_v2_shadow_executor_anchor_urad[12];
static actuator_joint_unwrapper_t host_v2_shadow_unwrappers[12];
static int32_t measured[12]={21476,926524,403437,-547631,1576932,64427,21476,-1630621,-1027767,-1414330,1520175,-69029};
static bool ready=true,observed=true;
static bool MobileArmObserver_Suspend(void){return true;}
static bool MobileBoard_IsConfigured(void){return true;}
static void S3Platform_Enable(bool enable){(void)enable;}
static bool MobileBoard_PrepareArmMotion(int32_t *a){if(!ready)return false;if(observed)memcpy(a,measured,sizeof(measured));return true;}
static bool MobileArmObserver_HasStarted(void){return observed;}
''' + source[begin:end] + r'''
static actuator_v2_stream_executor_t executor;
static void prime(uint32_t lead){
 memset(&executor,0,sizeof(executor));executor.diagnostics.state=ACTUATOR_V2_EXECUTOR_PRIMING;
 executor.session.validated_sample_count=8;executor.session.policy.minimum_start_samples=8;
 executor.session.validated_samples[0].apply_tick=165380+lead;
 for(unsigned j=0;j<12;j++){
  executor.joint_limits[j].minimum_urad=-4000000;executor.joint_limits[j].maximum_urad=4000000;
  executor.session.policy.maximum_step_urad_per_tick[j]=9000;
  executor.session.validated_samples[0].position_urad[j]=measured[j];
 }
 executor.session.validated_samples[0].position_urad[5]=130388;
 executor.session.validated_samples[0].position_urad[11]=-73631;
}
int main(void){
 /* Real failure: 35 ms to the first sample allows 63000 urad, but the
    contacted jaw has a 65961 urad residual from the retained goal. */
 prime(35);assert(actuator_v2_stream_executor_start(&executor,165380,measured)==ACTUATOR_V2_EXECUTOR_INVALID_ANCHOR);
 for(unsigned lead=20;lead<=50;lead+=5){
  memcpy(host_v2_shadow_executor_anchor_urad,measured,sizeof(measured));
  host_v2_shadow_executor_anchor_urad[0]+=1000; /* Arm anchor must refresh. */
  host_v2_shadow_executor_anchor_urad[5]=130388;host_v2_shadow_executor_anchor_urad[11]=-73631;
  assert(Host_PrepareArmMotionReference());
  prime(lead);assert(actuator_v2_stream_executor_start(&executor,165380,host_v2_shadow_executor_anchor_urad)==ACTUATOR_V2_EXECUTOR_OK);
  assert(host_v2_shadow_executor_anchor_urad[0]==measured[0]);
  assert(host_v2_shadow_executor_anchor_urad[5]==130388);
  assert(host_v2_shadow_executor_anchor_urad[11]==-73631);
  /* Unwrapping continues from actual feedback, not the contact preload. */
  assert(host_v2_shadow_unwrappers[5].unwrapped_raw==2006);
 }
 prime(35);executor.session.validated_samples[0].position_urad[0]+=70000;
 assert(actuator_v2_stream_executor_start(&executor,165380,host_v2_shadow_executor_anchor_urad)==ACTUATOR_V2_EXECUTOR_INVALID_ANCHOR);
 ready=false;assert(!Host_PrepareArmMotionReference());ready=true;
 measured[0]=99000000;assert(!Host_PrepareArmMotionReference());
 return 0;
}
'''
    src=tmp_path/'contact.c';src.write_text(harness);exe=tmp_path/'contact'
    cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-DHOST_S3_STATIONARY_PICK=1U',
      '-DS1_ARM_RANGE_PROFILE_HEADER="generated_s1_arm_ranges.h"','-DHOST_BIMANUAL_DMA_DISPATCH_BUILD=1U','-DHOST_BIMANUAL_DISPATCH_REFACTOR_BUILD=1U',
      '-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(board/'Inc'),'-I',str(core/'include'),str(src),str(board/'Src/bimanual_operational_limits.c')]
    cmd += [str(x) for x in (core/'src').glob('*.c')]+['-o',str(exe)]
    subprocess.run(cmd,check=True);subprocess.run([str(exe)],check=True)
