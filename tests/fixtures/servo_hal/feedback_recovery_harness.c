#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "servo_bus.h"
#include "right_servo_bus.h"
#include "servo_transport.h"
#include "bimanual_servo_dispatch.h"
#include "servo_feedback_completion.h"
#include "servo_rx_window.h"
#include "bimanual_tracking_feedback.h"
#include "s3_platform_snapshot.h"
#include "actuator_core/sts3215_packet.h"
#define SERVO_BUS_DMA_RING_CAPACITY 256U
#define RIGHT_SERVO_TELEMETRY_RING_CAPACITY 256U
#define SERVO_IN_MOTION_TELEMETRY_TIMEOUT_MS 4U
#define RIGHT_SERVO_TELEMETRY_TIMEOUT_MS 4U
#define RIGHT_SERVO_WRITE_SETTLE_MS 4U
#define SERVO_BUS_RECOVERY_QUIET_MS 2U
#define RIGHT_SERVO_REQUEST_SIZE 8U
#define RIGHT_SERVO_READ_ADDRESS 56U
#define RIGHT_SERVO_READ_LENGTH 2U
/* PRODUCTION_TYPES */
static ServoInMotionTelemetry servo_in_motion_telemetry;
static RightServoTelemetry right_servo_telemetry;
static ServoInMotionTelemetrySnapshot servo_in_motion_snapshot;
static RightServoInMotionTelemetrySnapshot right_servo_telemetry_snapshot;
static ServoBusDiagnostics servo_bus_diagnostics;
static ServoBusHealth servo_bus_health;
static uint32_t servo_in_motion_telemetry_failure[8],right_servo_telemetry_failure[8];
static DMA dma[2];
static REGS registers[2];
static UART_HandleTypeDef uart[2];
static UART_HandleTypeDef *servo_uart_handle=&uart[0],*right_servo_uart=&uart[1];
static uint32_t servo_feedback_token,right_feedback_token;
static bool servo_reader_shared=true,right_reader_shared=false;
static uint8_t servo_dma_rx_ring[256],right_servo_rx_ring[256];
static uint32_t cursor[2],now_us,delay_calls;
static uint32_t servo_uart_async_errors,servo_dma_async_error;
static uint32_t right_servo_uart_async_errors,right_servo_dma_async_error;
static uint32_t servo_transaction_start_absolute;
static uint8_t servo_transaction_active;
static bool dma_active[2]={true,true};
static int selected_bus,fault_mode,setup_failure;
static uint32_t right_servo_rx_wrap_count;
static bool timed_io;
static uint32_t tx_due[2],rx_due[2],wire_idle[2];
static uint8_t io_kind[2];
static bool reply_drop[2];
static void service_hardware(void);
static uint8_t requested_id[2];
static uint16_t commands[6]={2048,2048,2048,2048,2048,2048};
static uint32_t HALtick_base;
static uint64_t simulation_us;
static void advance_us(uint32_t n){now_us+=n;simulation_us+=n;service_hardware();}
uint32_t HAL_GetTick(void){return HALtick_base+(uint32_t)(simulation_us/1000);}
uint32_t Timebase_NowUs(void){return now_us;}
void HAL_Delay(uint32_t ms){delay_calls++;advance_us(ms*1000);}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*u){(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef*u){if(u->fail_abort)return HAL_ERROR;u->gState=HAL_UART_STATE_READY;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*u,const uint8_t*d,uint16_t n,uint32_t t){(void)u;(void)d;(void)n;(void)t;assert(0);return HAL_ERROR;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*u,const uint8_t*d,uint16_t n){
 (void)d;(void)n;assert(timed_io);int side=u==&uart[1];
 u->gState=HAL_UART_STATE_BUSY_TX;u->wire_busy=1;io_kind[side]=1;
 tx_due[side]=now_us+300;wire_idle[side]=now_us+325;return HAL_OK;
}
static void packet(int side,uint8_t id,int corrupt,int status){
 uint8_t p[8]={255,255,id,4,(uint8_t)status,0,8,0};
 unsigned sum=0;for(unsigned i=2;i<7;i++)sum+=p[i];p[7]=(uint8_t)(~sum);if(corrupt)p[7]^=1;
 uint8_t *ring=side?right_servo_rx_ring:servo_dma_rx_ring;
 for(unsigned i=0;i<8;i++)ring[cursor[side]++%256]=p[i];
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*u,const uint8_t*d,uint16_t n){
 int side=u==&uart[1];assert(n==8);requested_id[side]=d[2];u->sent++;
 int f=side==selected_bus?fault_mode:0;
 if(u->fail_tx)return HAL_ERROR;
 if(timed_io){
  u->gState=HAL_UART_STATE_BUSY_TX;u->wire_busy=1;io_kind[side]=2;
  tx_due[side]=now_us+80;rx_due[side]=now_us+180;wire_idle[side]=now_us+200;
  reply_drop[side]=f==1;return HAL_OK;
 }
 if(f==7){u->gState=HAL_UART_STATE_BUSY_TX;return HAL_OK;}
 u->gState=HAL_UART_STATE_READY;
 /* Immediate ISR exercises state-before-transmit ordering. */
 if(side)RightServoBus_InMotionTelemetryOnTxComplete(u);else Servo_InMotionTelemetryOnTxComplete(u);
 if(f==3)u->ErrorCode=4;
 if(f==4)dma_active[side]=false;
 if(f==12)dma[side].ErrorCode=1;
 if(f==8){cursor[side]+=257;return HAL_OK;}
 if(f==1 || f==6 || f==10 || f==11)return HAL_OK;
 packet(side,d[2],f==2||f==14,f==5?1:0);if(f==15)u->wire_busy=1;return HAL_OK;
}
static void service_hardware(void){
 if(!timed_io)return;
 for(unsigned i=0;i<2;i++){
  if(!io_kind[i])continue;
  if(uart[i].gState==HAL_UART_STATE_BUSY_TX && (int32_t)(now_us-tx_due[i])>=0){
   uart[i].gState=HAL_UART_STATE_READY;
   if(io_kind[i]==1)BimanualServoDispatch_OnTxComplete(&uart[i]);
   else if(i)RightServoBus_InMotionTelemetryOnTxComplete(&uart[i]);
   else Servo_InMotionTelemetryOnTxComplete(&uart[i]);
  }
  if(io_kind[i]==2 && rx_due[i] && (int32_t)(now_us-rx_due[i])>=0){
   if(!reply_drop[i])packet(i,requested_id[i],0,0);
   rx_due[i]=0;
  }
  if((int32_t)(now_us-wire_idle[i])>=0){uart[i].wire_busy=0;io_kind[i]=0;}
 }
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*u,uint8_t*d,uint16_t n){
 (void)d;(void)n;int side=u==&uart[1];
 if(setup_failure==21 && side==selected_bus)return HAL_ERROR;
 dma_active[side]=true;cursor[side]=0;
 if(setup_failure==22 && side==selected_bus){u->wire_busy=1;u->fail_abort=1;}
 return HAL_OK;
}
static HAL_StatusTypeDef ServoBus_ArmReceiver(void){
 if(setup_failure==21 && selected_bus==0)return HAL_ERROR;
 dma_active[0]=true;cursor[0]=0;
 if(setup_failure==22 && selected_bus==0){uart[0].wire_busy=1;uart[0].fail_abort=1;}
 return HAL_OK;
}
static uint32_t ServoBus_ProducerAbsolute(void){return cursor[0];}
static uint32_t RightServo_TelemetryProducerAbsolute(void){return cursor[1];}
static uint8_t ServoBus_DmaHardwareActive(void){return dma_active[0];}
static uint8_t RightServo_TelemetryDmaActive(void){return dma_active[1];}
static uint32_t ServoBus_CurrentUartErrorCode(void){return uart[0].ErrorCode;}
static bool ServoBus_ReaderCursor(UART_HandleTypeDef*u,uint32_t t,uint32_t*out){
 if(!ServoTransport_Owns(u,t)||!dma_active[0]||u->ErrorCode||dma[0].ErrorCode)return false;
 *out=cursor[0];return true;
}
static void ServoBus_RecordFailure(ServoBusFailureReason r,uint8_t id,HAL_StatusTypeDef st,uint8_t ss,uint16_t ds){
 servo_bus_diagnostics.reason=r;servo_bus_diagnostics.servo_id=id;servo_bus_diagnostics.hal_status=st;
 servo_bus_diagnostics.servo_status=ss;servo_bus_diagnostics.discarded_bytes=ds;
}
static void ServoBus_CountUartErrors(uint32_t e){(void)e;}
static ServoBusFailureReason ServoBus_MapRejectReason(ServoResponseRejectReason r){(void)r;return SERVO_BUS_FAILURE_CHECKSUM;}
static HAL_StatusTypeDef ServoBus_DisarmReceiver(void){dma_active[0]=false;return HAL_OK;}
static void RightServo_ClearReceiveState(void){}
static uint16_t RightServo_ReadU16Le(const uint8_t*d){return d[0]|((uint16_t)d[1]<<8);}
/* PRODUCTION_FUNCTIONS */
static BimanualTrackingFeedbackResult read_slot(uint8_t slot){
 assert(BimanualTrackingFeedback_CanStart());
 uint32_t start=now_us;unsigned delays=delay_calls;
 assert(BimanualTrackingFeedback_Start(slot,HAL_GetTick(),commands,commands,0,0)==HAL_OK);
 BimanualTrackingFeedbackSample sample={0};BimanualTrackingFeedbackResult result;
 do{
  uint32_t elapsed=now_us-start;
  if(fault_mode==10 && elapsed==2800)packet(selected_bus,requested_id[selected_bus],0,0);
  if(fault_mode==14 && elapsed==1000)packet(selected_bus,requested_id[selected_bus],0,0);
  if(fault_mode==15 && elapsed==100)uart[selected_bus].wire_busy=0;
  if(fault_mode==11 && elapsed>=4000 && elapsed%50==0){
   uint8_t *ring=selected_bus?right_servo_rx_ring:servo_dma_rx_ring;
   ring[cursor[selected_bus]++%256]=0xFF;
  }
  result=BimanualTrackingFeedback_Poll(HAL_GetTick(),&sample);
  if(result!=BIMANUAL_TRACKING_PENDING)break;
  assert(!BimanualTrackingFeedback_CanStart());
  advance_us(25);
 }while((uint32_t)(now_us-start)<4500);
#ifndef REPRO_OLD_END_ON_TIMEOUT
 assert(delay_calls==delays); /* Runtime polling must never block/recover. */
#else
 (void)delays; /* Continue to the original permanent CanStart failure. */
#endif
 assert(result!=BIMANUAL_TRACKING_PENDING);
 if(result==BIMANUAL_TRACKING_SAMPLE_READY)assert(sample.left_position_raw==2048&&sample.right_position_raw==2048);
 return result;
}
int main(int argc,char**argv){
 assert(argc==4);selected_bus=atoi(argv[1]);uint8_t slot=(uint8_t)atoi(argv[2]);int mode=atoi(argv[3]);
 for(unsigned i=0;i<2;i++){uart[i].gState=HAL_UART_STATE_READY;uart[i].hdmarx=&dma[i];uart[i].Instance=&registers[i];assert(ServoTransport_Register(&uart[i]));}
 if(mode==21||mode==22){
  setup_failure=mode;
  assert(BimanualTrackingFeedback_Begin()==HAL_ERROR);
  assert(!BimanualTrackingFeedback_CanStart());
  if(mode==22){
   assert(BimanualTrackingFeedback_Active());
   assert(!ServoTransport_ReadReady(&uart[selected_bus]));
   assert(BimanualTrackingFeedback_Begin()==HAL_BUSY);
  }
  setup_failure=0;uart[selected_bus].wire_busy=0;uart[selected_bus].fail_abort=0;
  BimanualTrackingFeedback_End();assert(!BimanualTrackingFeedback_Active());
  assert(ServoTransport_ReadReady(&uart[0])&&ServoTransport_ReadReady(&uart[1]));
  return 0;
 }
 /* These are the installed IDs; the hardware itself is not linked. */
 assert(BimanualTrackingFeedback_Begin()==HAL_OK);S3Platform_Enable(true);
 uint8_t initial_slot=mode==25?0:slot;
 fault_mode=0;assert(read_slot(initial_slot)==(initial_slot<6?BIMANUAL_TRACKING_SAMPLE_READY:BIMANUAL_TRACKING_AUX_READY));
 if(mode==20){
  for(unsigned phase=0;phase<32;phase++){
   /* Inventory stops DMA between idle capture/anchor/stream phases. */
   BimanualTrackingFeedback_End();assert(!BimanualTrackingFeedback_Active());
   assert(!servo_in_motion_telemetry.enabled&&!right_servo_telemetry.enabled);
   dma_active[0]=dma_active[1]=false;
   assert(BimanualTrackingFeedback_Begin()==HAL_OK);
   assert(read_slot(phase%10)==(phase%10<6?BIMANUAL_TRACKING_SAMPLE_READY:BIMANUAL_TRACKING_AUX_READY));
  }
  BimanualTrackingFeedback_End();return 0;
 }
 if(mode==23){
  uart[selected_bus].fail_tx=1;unsigned before=delay_calls;
  assert(BimanualTrackingFeedback_Start(2,HAL_GetTick(),commands,commands,0,0)==HAL_ERROR);
  assert(before==delay_calls && !BimanualTrackingFeedback_CanStart());
  BimanualTrackingFeedbackSample sample;
  assert(BimanualTrackingFeedback_Poll(HAL_GetTick(),&sample)==BIMANUAL_TRACKING_FAULT);
  assert(BimanualTrackingFeedback_Start(2,HAL_GetTick(),commands,commands,0,0)!=HAL_OK);
  BimanualTrackingFeedback_End();assert(!BimanualTrackingFeedback_Active());return 0;
 }
 if(mode==24){
  fault_mode=3;assert(read_slot(2)==BIMANUAL_TRACKING_FAULT);
  uint32_t failures=BimanualTrackingFeedback_GetSnapshot()->failed_pairs;
  for(unsigned n=0;n<10;n++){
   BimanualTrackingFeedbackSample sample;
   assert(BimanualTrackingFeedback_Poll(HAL_GetTick(),&sample)==BIMANUAL_TRACKING_FAULT);
   assert(BimanualTrackingFeedback_GetSnapshot()->failed_pairs==failures);
   assert(!BimanualTrackingFeedback_CanStart());
  }
  BimanualTrackingFeedback_End();return 0;
 }
 if(mode==25){
  timed_io=true;now_us=1000000;BimanualServoDispatch_Init(&uart[0],&uart[1]);
  uint8_t next=0;unsigned completed=0,expected_reads=0,expected_drops=0;
  for(unsigned cycle=0;cycle<12000;cycle++){
   uint32_t tick=now_us;bool read_started=false;
   bool late=cycle%101==0;
   if(late)advance_us(850); /* Defer reads after a late foreground iteration. */
   assert(!BimanualTrackingFeedback_Pending() && BimanualServoDispatch_Ready());
   assert(BimanualServoDispatch_Launch(commands,commands,HAL_GetTick(),tick)==HAL_OK);
   while((uint32_t)(now_us-tick)<5000){
    BimanualServoDispatch_Poll();assert(!BimanualServoDispatch_Faulted());
    BimanualTrackingFeedbackSample sample;
    BimanualTrackingFeedbackResult v=BimanualTrackingFeedback_Poll(HAL_GetTick(),&sample);
    assert(v!=BIMANUAL_TRACKING_FAULT);
    if(v==BIMANUAL_TRACKING_SAMPLE_READY||v==BIMANUAL_TRACKING_AUX_READY)completed++;
    if(!read_started && BimanualServoDispatch_Ready() && BimanualTrackingFeedback_CanStart() &&
        (uint32_t)(now_us-tick)+4450<5000){
     uint8_t j=S3Platform_Select(next);fault_mode=cycle%401==0?1:0;
     assert(BimanualTrackingFeedback_Start(j,HAL_GetTick(),commands,commands,0,0)==HAL_OK);
     if(j<6)next=(j+1)%6;
     expected_reads++;if(fault_mode==1)expected_drops++;
     read_started=true;
    }
    uint32_t step=slot?slot:25;
    uint32_t remaining=5000-(uint32_t)(now_us-tick);
    advance_us(step<remaining?step:remaining);
   }
   assert(read_started==!late && !BimanualTrackingFeedback_Pending());
   if(cycle>60){
    uint8_t out[44];S3Platform_Copy(HAL_GetTick(),out);assert(out[2]==15);
    for(unsigned j=0;j<4;j++){
     uint32_t age=0;for(unsigned b=0;b<4;b++)age|=(uint32_t)out[28+4*j+b]<<(8*b);
     assert(age<200); /* Cache timestamps remain real through drops/deferred slots. */
    }
   }
  }
  assert(completed==expected_reads-expected_drops && completed>11000);
  assert(BimanualServoDispatch_GetSnapshot()->completed_count==12000);
  assert(BimanualTrackingFeedback_GetSnapshot()->failed_pairs==expected_drops);
  BimanualTrackingFeedback_End();return 0;
 }
 if(mode==9){
  /* 60 seconds of 5 ms slots, repeated packet loss, parser/ring wrap and
     independent microsecond/millisecond counter wrap. No motion commands. */
  uint8_t next=0;now_us=UINT32_MAX-9000000U;HALtick_base=UINT32_MAX-1000U;
  for(unsigned k=0;k<12000;k++){
   uint32_t start=now_us;uint8_t j=S3Platform_Select(next);
   fault_mode=(k%401==0)?1:0;
   BimanualTrackingFeedbackResult v=read_slot(j);
   assert(v==BIMANUAL_TRACKING_SAMPLE_READY||v==BIMANUAL_TRACKING_AUX_READY||v==BIMANUAL_TRACKING_TRANSIENT_FAILURE);
   if(j<6)next=(j+1)%6;
   assert(servo_in_motion_telemetry.enabled && right_servo_telemetry.enabled);
   assert(ServoTransport_ReadReady(&uart[0])&&ServoTransport_ReadReady(&uart[1]));
   advance_us(5000-(uint32_t)(now_us-start));
  }
  assert(BimanualTrackingFeedback_GetSnapshot()->failed_pairs==30);
  return 0;
 }
 advance_us(5000);fault_mode=mode;
 BimanualTrackingFeedbackResult v=read_slot(slot);
 if(mode==0||mode==10||mode==14||mode==15)assert(v==(slot<6?BIMANUAL_TRACKING_SAMPLE_READY:BIMANUAL_TRACKING_AUX_READY));
 else if(mode==1||mode==2||mode==6){
  assert(v==BIMANUAL_TRACKING_TRANSIENT_FAILURE);
  assert(BimanualTrackingFeedback_CanStart());
  assert(servo_in_motion_telemetry.enabled && right_servo_telemetry.enabled);
  uint32_t *failure=selected_bus?right_servo_telemetry_failure:servo_in_motion_telemetry_failure;
  assert(failure[0]==1&&failure[2]==(selected_bus?slot%6:slot)&&failure[3]==HAL_TIMEOUT);
  assert(S3Platform_Select((slot+1)%6)==slot);
  /* Late bytes from the retired transaction must not become the next sample. */
  packet(selected_bus,requested_id[selected_bus],0,0);
  advance_us(5000);
  if(mode==6){
   assert(read_slot(slot)==BIMANUAL_TRACKING_TRANSIENT_FAILURE);advance_us(5000);
   assert(read_slot(slot)==BIMANUAL_TRACKING_FAULT);
  }else{
   uint8_t initial_slot=mode==25?0:slot;
 fault_mode=0;assert(read_slot(initial_slot)==(initial_slot<6?BIMANUAL_TRACKING_SAMPLE_READY:BIMANUAL_TRACKING_AUX_READY));
   assert(failure[0]==1); /* Successful reads cannot erase the first failure. */
  }
 }else{
  assert(v==BIMANUAL_TRACKING_FAULT);
  assert(!BimanualTrackingFeedback_CanStart());
 }
 BimanualTrackingFeedback_End();assert(!BimanualTrackingFeedback_Active());
 assert(!servo_in_motion_telemetry.enabled&&!right_servo_telemetry.enabled);
 return 0;
}
