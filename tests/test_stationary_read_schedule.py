"""Run the production feedback scheduler with a deterministic timer/bus.

A read begun just before an already pending output tick makes the executor
stop despite healthy replies. Exercise all phases, including wraparound.
"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / 'firmware/stm32_g474_single_arm/Core'


def c_function(source, signature):
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


def run_c(tmp_path, name, code):
    path = tmp_path / (name + '.c')
    path.write_text(code)
    exe = tmp_path / name
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT/'tests/fixtures/servo_hal'),
                    '-I', str(BOARD/'Inc'), str(path), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


def test_handoff_reads_oldest_platform_sample_without_restamping(tmp_path):
    code = r'''
#include <assert.h>
#include <string.h>
#include "s3_platform_snapshot.h"
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
int main(void){
 uint8_t before[44],after[44];
 S3Platform_Enable(true);assert(S3Platform_Select(0)==0); /* Cold boot. */
 for(unsigned wrap=0;wrap<2;wrap++)for(unsigned oldest=0;oldest<4;oldest++){
  uint32_t base=wrap?UINT32_MAX-50:1000;
  for(unsigned j=0;j<4;j++)S3Platform_Update(6+(oldest+j)%4,100+j,base+j*35);
  /* Cancellation after scheduling advanced next but acquired no sample. */
  S3Platform_Scheduled(6+oldest);
  S3Platform_Enable(false);S3Platform_Copy(base+120,before);
  S3Platform_Enable(true);S3Platform_Copy(base+120,after);
  assert(!memcmp(before+2,after+2,42));
  assert(S3Platform_Select(0)==6+oldest);
  assert(u32(after+28+oldest*4)==120);
  S3Platform_Scheduled(6+oldest);
  assert(S3Platform_Select(0)==0);
  S3Platform_Enable(true);assert(S3Platform_Select(0)==0); /* Idempotent. */
  S3Platform_Scheduled(5);assert(S3Platform_Select(0)==6+(oldest+1)%4);
 }
 S3Platform_Enable(false);S3Platform_Retry(3);S3Platform_Enable(true);
 assert(S3Platform_Select(0)==3); /* Pending error recovery still wins. */
 S3Platform_Scheduled(3);S3Platform_CancelRetry();
 return 0;
}
'''
    path=tmp_path/'platform-resume.c';path.write_text(code);exe=tmp_path/'platform-resume'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-I',str(BOARD/'Inc'),
                    str(path),str(BOARD/'Src/s3_platform_snapshot.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def test_normal_wire_busy_never_runs_maintenance_recovery_in_runtime(tmp_path):
    source = (BOARD/'Src/servo_bus.c').read_text()
    # Both real preparation paths, with a deterministic healthy UART and the
    # maintenance recovery's documented quiet/stability delay. The old path
    # demonstrates why a pre-admission timer check alone did not bound the read.
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#define HOST_S3_STATIONARY_PICK 1
#define ACTUATOR_BUS_WORK_FEEDBACK 1
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {uint32_t ErrorCode;} DMA;
typedef struct {DMA *hdmarx; bool busy;} UART;
typedef UART UART_HandleTypeDef;
static DMA dma;
static UART uart={.hdmarx=&dma};
static UART *servo_uart_handle=&uart;
#define UART_FLAG_BUSY 1
#define RESET 0
#define HAL_UART_ERROR_NONE 0
#define HAL_DMA_ERROR_NONE 0
#define SERVO_BUS_PREFLIGHT_IDLE_TIMEOUT_MS 2
#define SERVO_BUS_FAILURE_RECOVERY 9
#define SERVO_BUS_FAILURE_UART 3
#define SERVO_BUS_FAILURE_DMA 11
#define __HAL_UART_GET_FLAG(u,f) ((void)(f),(u)->busy)
static struct {uint8_t servo_id;uint32_t recovery_count;} servo_bus_diagnostics;
static struct {uint32_t transaction_count,recovery_count;uint16_t dma_error_count;} servo_bus_health;
static uint32_t servo_uart_async_errors,servo_dma_async_error,servo_transaction_start_absolute;
static uint8_t servo_transaction_active;
static uint32_t servo_feedback_token=42,now_us,hardware_errors,cursor=8192,recoveries;
static bool healthy=true,service_allowed=true,prepare_fails,servo_reader_shared;
static unsigned arm_count;
static struct {uint8_t enabled;} servo_in_motion_telemetry;
static uint8_t servo_in_motion_snapshot[40];
bool ServoTransport_ServiceAllowed(void){return service_allowed;}
bool ServoTransport_BeginService(UART*u,int work,uint32_t*t){(void)u;(void)work;*t=42;return !prepare_fails;}
HAL_StatusTypeDef ServoTransport_End(UART*u,uint32_t t,bool abort){(void)u;(void)t;(void)abort;return HAL_OK;}
void ServoTransport_RequestStop(UART*u){(void)u;}
bool ServoBus_PrepareReader(UART*u,bool proof);
uint32_t HAL_GetTick(void){return now_us++/1000;}
uint32_t ServoBus_CurrentUartErrorCode(void){return hardware_errors;}
uint8_t ServoBus_DmaHardwareActive(void){return healthy;}
void ServoBus_CountUartErrors(uint32_t x){(void)x;}
void ServoBus_IncrementU16(uint16_t*x){(*x)++;}
HAL_StatusTypeDef ServoBus_Recover(void){recoveries++;now_us+=2000;uart.busy=false;healthy=false;return HAL_OK;}
HAL_StatusTypeDef ServoBus_ArmReceiver(void){arm_count++;now_us+=3000;healthy=true;return HAL_OK;}
void ServoBus_RecordFailure(int r,uint8_t i,int h,int s,int b){(void)r;(void)i;(void)h;(void)s;(void)b;}
uint32_t ServoBus_ProducerAbsolute(void){return cursor;}
bool ServoBus_ReaderCursor(UART*u,uint32_t token,uint32_t*out){
 if(u!=&uart || token!=42 || !healthy || hardware_errors || servo_uart_async_errors || servo_dma_async_error || dma.ErrorCode)return false;
 *out=cursor;return true;
}
''' + c_function(source, 'static HAL_StatusTypeDef ServoBus_PrepareTransaction(') + \
        c_function(source, 'bool ServoBus_PrepareReader(') + \
        c_function(source, 'HAL_StatusTypeDef Servo_InMotionTelemetryBegin(') + \
        c_function(source, 'static HAL_StatusTypeDef ServoBus_PrepareRuntimeRead(') + r'''
int main(void){
 uint32_t out=0;
 uart.busy=true;now_us=400;
 assert(ServoBus_PrepareTransaction(1,&out)==HAL_OK);
 assert(now_us>=5400 && recoveries==1); /* Old path crosses next 5 ms output. */
 for(unsigned phase=0;phase<5000;phase++){
  now_us=phase;uart.busy=true;recoveries=0;servo_transaction_active=0;
  uint32_t n=servo_bus_health.transaction_count;
  assert(ServoBus_PrepareRuntimeRead(1,&out)==HAL_BUSY);
  assert(now_us==phase && recoveries==0 && !servo_transaction_active);
  assert(n==servo_bus_health.transaction_count);
  uart.busy=false;assert(ServoBus_PrepareRuntimeRead(1,&out)==HAL_OK);
  assert(now_us==phase && out==cursor && servo_transaction_active);
 }
 hardware_errors=1;assert(ServoBus_PrepareRuntimeRead(1,&out)==HAL_ERROR);
 hardware_errors=0;healthy=false;assert(ServoBus_PrepareRuntimeRead(1,&out)==HAL_ERROR);
 healthy=true;dma.ErrorCode=1;assert(ServoBus_PrepareRuntimeRead(1,&out)==HAL_ERROR);
 dma.ErrorCode=0;assert(ServoBus_PrepareRuntimeRead(1,NULL)==HAL_ERROR);
 assert(recoveries==0);
 /* Cold boot and the maintenance inventory/anchor both leave RX disarmed.
    Preparation must occur before the periodic timer starts, including each
    idle-reader -> anchor -> trajectory and terminal -> idle-reader handoff. */
 servo_feedback_token=0;
 for(unsigned cycle=0;cycle<32;cycle++){
  healthy=false;servo_in_motion_telemetry.enabled=0;now_us=0;
  assert(ServoBus_PrepareRuntimeRead(1,&out)==HAL_ERROR);
  assert(Servo_InMotionTelemetryBegin()==HAL_OK);
  assert(healthy && servo_reader_shared && servo_in_motion_telemetry.enabled);
  assert(now_us==3000);
  unsigned armed=arm_count;
  servo_feedback_token=42;now_us=100400; /* First output has completed. */
  assert(ServoBus_PrepareRuntimeRead(1,&out)==HAL_OK);
  assert(now_us==100400 && arm_count==armed);
  assert(Servo_InMotionTelemetryBegin()==HAL_BUSY);servo_feedback_token=0;
 }
 servo_in_motion_telemetry.enabled=0;healthy=false;prepare_fails=true;
 assert(Servo_InMotionTelemetryBegin()==HAL_ERROR && !servo_in_motion_telemetry.enabled);
 prepare_fails=false;service_allowed=false;
 unsigned armed=arm_count;
 assert(Servo_InMotionTelemetryBegin()==HAL_ERROR && arm_count==armed);
 healthy=true;assert(Servo_InMotionTelemetryBegin()==HAL_OK && arm_count==armed);
 return 0;
}
'''
    run_c(tmp_path, 'runtime-reader', code)


def test_output_boundary_consumes_completed_reply_but_rejects_real_fault(tmp_path):
    source = (BOARD/'Src/binary_control.c').read_text()
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include "control_tick.h"
static bool host_stop_latched,host_v2_coordinated_stop_pending;
static bool pending=true,reply_arrived,reply_bad,dispatcher_ready=true;
static unsigned polls;
void Host_ServiceV2TrackingFeedback(bool allow_start){
 assert(!allow_start);polls++;
 if(reply_arrived)pending=false;
 if(reply_bad)host_v2_coordinated_stop_pending=true;
}
uint8_t BimanualServoDispatch_Ready(void){return dispatcher_ready && !pending;}
uint8_t BimanualTrackingFeedback_Pending(void){return pending;}
''' + c_function(source, 'static bool Host_V2OutputReady(') + r'''
int main(void){
 ControlTickEvent event={0};
 assert(!Host_V2OutputReady(&event)); /* Missing response remains an error. */
 reply_arrived=true;
 assert(!BimanualServoDispatch_Ready()); /* Old check rejects the completed DMA reply. */
 assert(Host_V2OutputReady(&event) && polls==2);
 event.missed_count=1;assert(!Host_V2OutputReady(&event));event.missed_count=0;
 dispatcher_ready=false;assert(!Host_V2OutputReady(&event));dispatcher_ready=true;
 reply_bad=true;assert(!Host_V2OutputReady(&event));reply_bad=false;
 host_v2_coordinated_stop_pending=false;host_stop_latched=true;assert(!Host_V2OutputReady(&event));
 return 0;
}
'''
    run_c(tmp_path, 'output-boundary', code)


def test_first_stop_record_is_not_overwritten_by_cleanup(tmp_path):
    source=(BOARD/'Src/binary_control.c').read_text()
    start=source.index('static uint32_t s3_first_stop_line')
    end=source.index('#define Host_RequestV2CoordinatedStop()',start)
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include "bimanual_tracking_feedback.h"
#include "bimanual_servo_dispatch.h"
static uint32_t tick=90195;
static uint8_t host_v2_executor_clock_active=1,host_v2_terminal_settle_active;
static uint8_t host_v2_shadow_anchor_ready,host_stop_latched;
static bool pending=true,observer=false;
static actuator_bimanual_dispatch_snapshot_t d={.completed_count=1};
static BimanualTrackingFeedbackSnapshot t;
uint32_t HAL_GetTick(void){return tick;}
uint8_t BimanualTrackingFeedback_Pending(void){return pending;}
bool MobileArmObserver_OwnsFeedback(void){return observer;}
const actuator_bimanual_dispatch_snapshot_t *BimanualServoDispatch_GetSnapshot(void){return &d;}
const BimanualTrackingFeedbackSnapshot *BimanualTrackingFeedback_GetSnapshot(void){return &t;}
'''+source[start:end]+r'''
int main(void){
 Host_RecordS3Stop(3200);assert(s3_first_stop_recorded);
 assert(s3_first_stop_line==3200 && s3_first_stop_tick==90195);
 assert(s3_first_stop_flags==17 && s3_stop_dispatch_count==1 && s3_stop_tracking_count==0);
 host_stop_latched=1;host_v2_executor_clock_active=0;pending=false;tick+=1000;d.completed_count=9;
 Host_RecordS3Stop(0);Host_RecordS3Stop(9999);
 assert(s3_first_stop_line==3200 && s3_first_stop_tick==90195 && s3_first_stop_flags==17);
 assert(s3_stop_dispatch_count==1);
 host_stop_latched=0;host_v2_shadow_anchor_ready=1;observer=true;
 assert(Host_S3RuntimeFlags()==36);
 host_v2_terminal_settle_active=1;assert(!(Host_S3RuntimeFlags()&4));
 (void)s3_read_window_deferrals;return 0;
}
'''
    path=tmp_path/'first-stop.c';path.write_text(code);exe=tmp_path/'first-stop'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
        '-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(BOARD/'Inc'),
        '-I',str(ROOT/'firmware/stm32_actuator/include'),str(path),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def test_production_read_window_checks_pending_event_budget_and_timer_wrap(tmp_path):
    source=(BOARD/'Src/control_tick.c').read_text()
    start=source.index('bool ControlTick_ReadWindowAvailable(')
    end=source.index('\n}\n',start)+3
    code=r'''
#include <assert.h>
#include "stm32g4xx_hal.h"
#include "control_tick.h"
static uint32_t previous_started_us,now;
static uint8_t previous_started_valid,control_tick_pending;
uint32_t Timebase_NowUs(void){return now;}
'''+source[start:end]+r'''
int main(void){
 assert(!ControlTick_ReadWindowAvailable(4000));
 previous_started_valid=1;
 for(unsigned wrap=0;wrap<2;wrap++){
  previous_started_us=wrap?UINT32_MAX-700:10000;
  for(unsigned phase=0;phase<=10000;phase++){
   now=previous_started_us+phase;control_tick_pending=0;
   assert(ControlTick_ReadWindowAvailable(4000)==(phase<1000));
   control_tick_pending=1;assert(!ControlTick_ReadWindowAvailable(4000));
  }
 }
 now=previous_started_us;control_tick_pending=0;
 assert(!ControlTick_ReadWindowAvailable(0));
 assert(!ControlTick_ReadWindowAvailable(5000));return 0;
}
'''
    path=tmp_path/'window.c';path.write_text(code);exe=tmp_path/'window'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-DHOST_BIMANUAL_DMA_DISPATCH_BUILD=1U',
        '-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(BOARD/'Inc'),str(path),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def test_feedback_never_claims_bus_across_next_output_tick(tmp_path):
    source = (BOARD / 'Src/binary_control.c').read_text()
    start = source.index('static void Host_ServiceV2TrackingFeedback(bool allow_start)')
    end = source.index('\n}\n', start) + 3
    harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "single_arm_config.h"
#include "bimanual_tracking_feedback.h"
#include "bimanual_servo_dispatch.h"
#include "actuator_core/stream_executor_v2.h"
#include "control_tick.h"
#include "s3_platform_snapshot.h"
static uint32_t now_us, epoch_us, reads, stopped;
static bool tick_pending, bus_pending;
static uint8_t host_v2_executor_clock_active=1,host_v2_tracking_next_joint;
static uint32_t host_v2_tracking_last_dispatch_completed;
static uint32_t s3_read_window_deferrals;
static actuator_v2_stream_executor_t host_v2_stream_executor;
static uint16_t host_v2_last_left_raw[6],host_v2_last_right_raw[6];
static int32_t host_v2_output_urad[12];
static actuator_bimanual_dispatch_snapshot_t dispatch;
uint32_t HAL_GetTick(void){return now_us/1000;}
uint32_t Timebase_NowUs(void){return now_us;}
bool MobileBoard_StopActive(void){return false;}
bool MobileArmObserver_OwnsFeedback(void){return false;}
uint8_t BimanualTrackingFeedback_Active(void){return 1;}
uint8_t BimanualTrackingFeedback_CanStart(void){return !bus_pending;}
BimanualTrackingFeedbackResult BimanualTrackingFeedback_Poll(uint32_t n,BimanualTrackingFeedbackSample*s){(void)n;(void)s;return BIMANUAL_TRACKING_IDLE;}
void Host_RequestV2CoordinatedStop(void){stopped++;}
uint8_t Host_V2TrackingRawToUrad(uint8_t j,uint16_t r,int32_t*p){(void)j;*p=r;return 1;}
actuator_v2_executor_result_t actuator_v2_stream_executor_check_joint_feedback(actuator_v2_stream_executor_t*e,uint32_t n,uint8_t j,int32_t c,int32_t a){(void)e;(void)n;(void)j;(void)c;(void)a;return ACTUATOR_V2_EXECUTOR_OK;}
const actuator_bimanual_dispatch_snapshot_t *BimanualServoDispatch_GetSnapshot(void){return &dispatch;}
HAL_StatusTypeDef BimanualTrackingFeedback_Start(uint8_t j,uint32_t t,const uint16_t l[6],const uint16_t r[6],int32_t a,int32_t b){
 (void)t;(void)l;(void)r;(void)a;(void)b;reads++;bus_pending=true;S3Platform_Scheduled(j);return HAL_OK;
}
bool ControlTick_ReadWindowAvailable(uint32_t budget_us){
 uint32_t elapsed=now_us-epoch_us;
 return !tick_pending && elapsed<CONTROL_TICK_PERIOD_US && budget_us<CONTROL_TICK_PERIOD_US-elapsed;
}
'''+source[start:end]+r'''
int main(void){
 S3Platform_Enable(true);
 host_v2_stream_executor.output_valid=true;
 host_v2_stream_executor.diagnostics.state=ACTUATOR_V2_EXECUTOR_RUNNING;
 dispatch.completed_count=1;
 /* Boundary polling can never acquire another read lease. */
 epoch_us=2000;now_us=2400;Host_ServiceV2TrackingFeedback(false);
 assert(reads==0 && !bus_pending);
 /* Reproduce the dump: the first output completed and an auxiliary read
    (slot 6, motor 7) is due while the next output event is pending. */
 S3Platform_Scheduled(5);epoch_us=10000;now_us=10050;tick_pending=true;
 Host_ServiceV2TrackingFeedback(true);assert(reads==0 && !bus_pending);
 /* A late pass must defer even before the timer ISR has fired. */
 for(unsigned phase=0;phase<5000;phase+=10){
  now_us=20000+phase;epoch_us=20000;tick_pending=false;bus_pending=false;
  reads=0;host_v2_tracking_last_dispatch_completed=0;
  Host_ServiceV2TrackingFeedback(true);
  if(phase>=750)assert(reads==0);
  if(reads)assert(phase+4450<5000);
 }
 /* Early phase admits both arm and platform reads and accounts completion. */
 for(unsigned j=0;j<28;j++){
  epoch_us=30000+j*5000;now_us=epoch_us+400;dispatch.completed_count++;
  bus_pending=false;reads=0;Host_ServiceV2TrackingFeedback(true);assert(reads==1);
 }
 epoch_us=UINT32_MAX-300;now_us=epoch_us+400;bus_pending=false;
 dispatch.completed_count++;reads=0;Host_ServiceV2TrackingFeedback(true);assert(reads==1);
 /* Finite settling has no next output deadline. */
 host_v2_executor_clock_active=0;tick_pending=true;bus_pending=false;reads=0;
 dispatch.completed_count++;Host_ServiceV2TrackingFeedback(true);assert(reads==1);
 assert(!stopped);(void)s3_read_window_deferrals;return 0;
}
'''
    path = tmp_path/'scheduler.c';path.write_text(harness)
    exe = tmp_path/'scheduler'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',
                    '-DHOST_BIMANUAL_TRACKING_FEEDBACK_BUILD=1U','-DHOST_S3_STATIONARY_PICK=1U',
                    '-I',str(ROOT/'tests/fixtures/servo_hal'),'-I',str(BOARD/'Inc'),
                    '-I',str(ROOT/'firmware/stm32_actuator/include'),str(path),
                    str(BOARD/'Src/s3_platform_snapshot.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
