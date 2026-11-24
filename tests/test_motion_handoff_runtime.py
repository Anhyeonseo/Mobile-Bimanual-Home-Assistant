"""Exercise the real service handoff with pending, completed and failed reads."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]

def test_motion_handoff_waits_before_start(tmp_path):
    source=(ROOT/'firmware/stm32_g474_single_arm/Core/Src/binary_control.c').read_text()
    start=source.index('static void Host_ServiceV2MotionHandoff(void)')
    end=source.index('\n#endif\n\nstatic void Host_ServiceV2Executor',start)
    harness=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define HOST_BIMANUAL_TERMINAL_SETTLE_BUILD 1
#include "mobile_arm_observer.h"
typedef struct { uint32_t completed_count; } actuator_bimanual_dispatch_snapshot_t;
static actuator_bimanual_dispatch_snapshot_t dispatch={60};
static unsigned host_v2_reference_pending,host_stop_latched,host_v2_coordinated_stop_pending;
static unsigned host_v2_tracking_next_joint,host_v2_tracking_last_dispatch_completed;
static unsigned host_v2_terminal_settle_active,host_v2_terminal_settle_baseline_completed_pairs;
static unsigned host_v2_executor_start_pending,host_v2_executor_clock_active;
static unsigned prepare_calls,begin_calls,tick_clears,stop_calls;
static bool reference_ok=true;static int begin_result;
static MobileArmHandoff response=MOBILE_ARM_HANDOFF_WAIT;
MobileArmHandoff MobileArmObserver_PrepareMotion(void){return response;}
static bool Host_PrepareArmMotionReference(void){prepare_calls++;return reference_ok;}
static int BimanualTrackingFeedback_Begin(void){begin_calls++;return begin_result;}
static const actuator_bimanual_dispatch_snapshot_t *BimanualServoDispatch_GetSnapshot(void){return &dispatch;}
static void ControlTick_ClearPending(void){tick_clears++;}
static void Host_RequestV2CoordinatedStop(void){stop_calls++;}
'''+source[start:end]+r'''
int main(void){
 host_v2_reference_pending=1;
 for(unsigned i=0;i<20;i++)Host_ServiceV2MotionHandoff();
 assert(host_v2_reference_pending&&!prepare_calls&&!begin_calls&&!tick_clears&&!stop_calls);
 assert(!host_v2_executor_clock_active&&!host_v2_executor_start_pending);
 response=MOBILE_ARM_HANDOFF_READY;Host_ServiceV2MotionHandoff();
 assert(!host_v2_reference_pending&&prepare_calls==1&&begin_calls==1&&tick_clears==1);
 assert(host_v2_executor_start_pending&&host_v2_executor_clock_active);
 assert(host_v2_tracking_last_dispatch_completed==60);
 Host_ServiceV2MotionHandoff();assert(prepare_calls==1&&begin_calls==1);
 host_v2_executor_clock_active=host_v2_executor_start_pending=0;
 host_v2_reference_pending=1;response=MOBILE_ARM_HANDOFF_FAULT;
 Host_ServiceV2MotionHandoff();assert(stop_calls==1&&!host_v2_reference_pending&&!host_v2_executor_clock_active);
 host_v2_reference_pending=1;response=MOBILE_ARM_HANDOFF_READY;reference_ok=false;
 Host_ServiceV2MotionHandoff();assert(stop_calls==2&&begin_calls==1);
 host_v2_reference_pending=1;reference_ok=true;begin_result=1;
 Host_ServiceV2MotionHandoff();assert(stop_calls==3&&!host_v2_executor_clock_active);
 host_v2_reference_pending=1;host_stop_latched=1;begin_result=0;
 Host_ServiceV2MotionHandoff();assert(begin_calls==2&&!host_v2_executor_clock_active);
 return 0;
}
'''
    src=tmp_path/'handoff.c';src.write_text(harness);exe=tmp_path/'handoff'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(ROOT/'firmware/stm32_g474_single_arm/Core/Inc'),str(src),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
