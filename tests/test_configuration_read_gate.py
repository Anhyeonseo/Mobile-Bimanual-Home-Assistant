"""Execute the board's actual setup entry point with host-only I/O counters."""
from pathlib import Path
import subprocess

from test_stm32_servo_uart_circular_dma_contract import BINARY_CONTROL, function_body


def test_configuration_never_reads_or_enables_while_another_motion_owns_io(tmp_path):
    body = function_body(BINARY_CONTROL, 'static uint8_t Host_ConfigureBimanualForTrajectory(void)')
    source = r'''
#include <stdint.h>
#include <assert.h>
#define TAUGHT_POSE_PROFILE_HEADER 1
#define HOST_LEFT_ONLY_BUILD 1
enum { ACTUATOR_V2_EXECUTOR_CLOSED, ACTUATOR_V2_EXECUTOR_PRIMING,
 ACTUATOR_V2_EXECUTOR_RUNNING, ACTUATOR_V2_EXECUTOR_HOLD, ACTUATOR_V2_EXECUTOR_SUCCEEDED };
static struct { uint8_t active; } host_binary_motion;
static struct { struct { int state; } diagnostics; } host_v2_stream_executor;
static uint8_t host_v2_executor_ready=1, host_v2_executor_clock_active, buffered, stop;
static uint8_t host_right_arm_output_active;
static uint32_t host_bimanual_arm_watchdog_grace_started_ms;
static uint16_t taught_minimum_raw[6]={0}, taught_maximum_raw[6]={4095,4095,4095,4095,4095,4095};
static uint16_t taught_offsets[6]={0};
static int reads, enables;
enum {HAL_OK=0};
static int Host_BufferedExecutionIsActive(void){return buffered;}
static int MobileBoard_StopActive(void){return stop;}
static int MobileBoard_ArmsPrepared(void){return 1;}
static int Servo_ReadAllPositions(uint16_t *q){reads++;for(int i=0;i<6;i++)q[i]=2048;return HAL_OK;}
static int Servo_ReadData(int id,int reg,int len,uint8_t *data){(void)id;(void)reg;(void)len;reads++;data[0]=data[1]=0;return HAL_OK;}
static int Servo_ConfigureAllForTrajectory(uint16_t *q){(void)q;enables++;return HAL_OK;}
static int Servo_DisableTorqueAll(void){return HAL_OK;}
static void TaughtPoseGuard_Reset(void){}
static uint32_t HAL_GetTick(void){return 1;}
'''
    source += '\nstatic uint8_t Host_ConfigureBimanualForTrajectory(void) {\n'+body+'\n}\n'
    source += r'''
int main(void){
 for(int fault=0;fault<8;fault++){
  host_binary_motion.active=buffered=host_v2_executor_clock_active=stop=0;
  host_v2_stream_executor.diagnostics.state=ACTUATOR_V2_EXECUTOR_CLOSED;
  host_v2_executor_ready=1;
  if(fault==0)host_binary_motion.active=1;
  if(fault==1)buffered=1;
  if(fault==2)host_v2_executor_clock_active=1;
  if(fault>=3&&fault<=5)host_v2_stream_executor.diagnostics.state=fault-2;
  if(fault==6)stop=1;
  if(fault==7)host_v2_executor_ready=0;
  assert(Host_ConfigureBimanualForTrajectory()==0);
  assert(reads==0 && enables==0);
 }
 host_v2_executor_ready=1;
 assert(Host_ConfigureBimanualForTrajectory()==1);
 assert(reads==7 && enables==1);
 assert(host_right_arm_output_active==0);
 assert(host_bimanual_arm_watchdog_grace_started_ms==1);
 return 0;
}
'''
    path = tmp_path/'configuration.c'; path.write_text(source)
    executable = tmp_path/'configuration'
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',str(path),'-o',str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
