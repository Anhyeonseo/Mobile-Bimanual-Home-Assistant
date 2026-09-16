/* Real board assembly + both writers + core, synthetic independent sensors.
 * UART receive parsing is covered by mobile_feedback_harness separately. */
#include "mobile_board.h"
#include "mobile_hold_output.h"
#include "mobile_arm_observer.h"
#include "binary_control.h"
#include "bimanual_feedback_snapshot.h"
#include "bimanual_operational_limits.h"
#include "bimanual_tracking_feedback.h"
#include "actuator_core/crc32c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"line %d us=%u: %s\n",__LINE__,elapsed,#x);exit(1);}}while(0)
static uint32_t elapsed;
static UART_HandleTypeDef left={.Init={1000000},.gState=HAL_UART_STATE_READY};
static UART_HandleTypeDef right={.Init={1000000},.gState=HAL_UART_STATE_READY};
static actuator_mobile_feedback_reader_t reader;
static uint8_t modes[4]={1,1,1,1},locks[4]={1,1,1,1},torque;
static bool provisioning_bad_readback;
static unsigned provisioning_writes;
static unsigned reads,reader_starts,right_starts,stops,zero_writes,hold_writes;
static bool stale_arms,tracking,pending,moved,load=true,feed=true;
static uint8_t joint;
static uint32_t read_stamp;
static const MobileBoardProfile profile={
 .devices={{777,777,777,777},0,1000},
 .mobile={{1000,1000,1000,100},2,200,200,0,500000},
 .feedback={.axes={{1,90,150,70,300},{1,90,150,70,300},{1,90,150,70,300},{1,90,150,70,300}},
    .sample_timeout_ms=200,.mode_timeout_ms=500,.response_timeout_ms=5,.maximum_sample_skew_ms=100},
 .output={{5000,1000,200,10000,20000,5000,500},500,400,600,4000,{1,1,1,1}},
 .lift={409600,500000,100,100,20,2,10000,500,1000,100,1000,20,200,1},
 .read_period_us=5000,.read_budget_us=600,.quiet_us=50,.current_microamps_per_raw=1000,
 .stop={4000,600,600,200,100},.arm_hold_tolerance_raw=5,.arm_hold_dwell_ms=20,.arm_read_period_ms=10
};
uint32_t Timebase_NowUs(void){return UINT32_MAX-10000u+elapsed;}
uint32_t HAL_GetTick(void){return elapsed/1000;}
bool ControlTick_PeekEpoch(uint32_t *epoch){*epoch=UINT32_MAX-10000u+(elapsed/5000)*5000;return true;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){
 CHECK(n==20||n==26);CHECK(p[4]==131&&p[6]==2);
 if(n==20){CHECK(u==&left&&p[5]==46);zero_writes++;for(unsigned i=0;i<4;i++)CHECK(p[7+3*i]==8+i&&!p[8+3*i]&&!p[9+3*i]);}
 else {CHECK(p[5]==42);if(u==&left)CHECK(zero_writes==1);hold_writes++;
  for(unsigned i=0;i<6;i++)CHECK(p[7+3*i]==i+1&&p[8+3*i]==0&&p[9+3*i]==8);}
 u->sent++;if(u->fail_tx)return HAL_ERROR;
 u->gState=HAL_UART_STATE_READY;MobileServoOutput_OnTxComplete(u);MobileHoldOutput_OnTxComplete(u);return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t timeout){(void)u;(void)p;(void)n;(void)timeout;CHECK(false);return HAL_ERROR;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){u->aborted++;u->gState=HAL_UART_STATE_READY;return HAL_OK;}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t n,uint8_t *out){
 reads++;CHECK(id>=8&&id<=11);if(addr==3){CHECK(n==4);out[0]=9;out[1]=3;out[2]=id;out[3]=0;}
 else if(addr==40){CHECK(n==1);out[0]=torque;}
 else {CHECK(n==1&&(addr==33||addr==55));out[0]=addr==33?modes[id-8]:locks[id-8];}return HAL_OK;
}
bool MobileBootIdentity_Advance(bool initialize,uint32_t *boot){(void)initialize;*boot=42;return true;}
void BinaryControl_LatchStop(void){MobileBoard_RequestStop();}
bool BinaryControl_AttachMobileEndpoint(actuator_mobile_endpoint_t *e){return e!=NULL;}
HAL_StatusTypeDef RightServoBus_PreparePeriodicReader(void){right_starts++;return HAL_OK;}
bool MobileServoFeedback_Configure(UART_HandleTypeDef *u,actuator_mobile_supervisor_t *s,
 const actuator_mobile_feedback_config_t *c,const actuator_bus_schedule_config_t *timing,
 uint32_t period,uint32_t budget,uint32_t quiet,bool proof){
 (void)timing;(void)period;(void)budget;(void)quiet;CHECK(u==&left&&proof&&ServoTransport_ServiceAllowed());
 reader_starts++;return actuator_mobile_feedback_reader_init(&reader,s,c);
}
const actuator_mobile_feedback_reader_t *MobileServoFeedback_State(void){return &reader;}
void MobileServoFeedback_SetLiftEvidence(const actuator_mobile_lift_evidence_t *e){(void)e;}
bool MobileServoFeedback_AcceptsOutput(const actuator_mobile_supervisor_t *s,const actuator_mobile_output_config_t *c){(void)c;return s==reader.supervisor;}
void BimanualServoDispatch_LatchFault(void){stops++;}
void BimanualTrackingFeedback_End(void){tracking=false;pending=false;}
HAL_StatusTypeDef BimanualTrackingFeedback_Begin(void){CHECK(!tracking);tracking=true;return HAL_OK;}
uint8_t BimanualTrackingFeedback_Active(void){return tracking;}
uint8_t BimanualTrackingFeedback_CanStart(void){return tracking&&!pending;}
uint8_t BimanualTrackingFeedback_Pending(void){return pending;}
HAL_StatusTypeDef BimanualTrackingFeedback_Start(uint8_t j,uint32_t stamp,const uint16_t l[6],const uint16_t r[6],int32_t lc,int32_t rc){
 (void)l;(void)r;(void)lc;(void)rc;CHECK(tracking&&!pending&&(MobileBoard_StopState()->actions_completed||MobileArmObserver_OwnsFeedback()));joint=j;read_stamp=stamp;pending=true;return HAL_OK;
}
BimanualTrackingFeedbackResult BimanualTrackingFeedback_Poll(uint32_t now,BimanualTrackingFeedbackSample *s){
 if(!pending||now==read_stamp)return BIMANUAL_TRACKING_PENDING;
 *s=(BimanualTrackingFeedbackSample){.joint_index=joint,.observed_ms=read_stamp,.left_position_raw=moved?2100:2048,.right_position_raw=2048};pending=false;return BIMANUAL_TRACKING_SAMPLE_READY;
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t address,const uint8_t *data,uint8_t length){
 CHECK(id>=8&&id<=11&&length==1&&(address==33||address==55));provisioning_writes++;
 if(!provisioning_bad_readback){if(address==33)modes[id-8]=data[0];else locks[id-8]=data[0];}
 return HAL_OK;
}
static void inject(void){
 if(!reader.configured||!feed)return;
 uint32_t ms=HAL_GetTick();for(unsigned i=0;i<4;i++)reader.axes[i]=(actuator_mobile_axis_sample_t){.position_raw=2048,.observed_ms=ms,.mode_ms=ms,.valid=true,.mode_verified=true};
 actuator_mobile_feedback_t f={.observed_ms=ms,.velocity_modes_verified=true,.lift_homed=true,.hardware_ok=true};
 (void)actuator_mobile_feedback(reader.supervisor,&f,ms);
 MobileBoard_ObserveInterlocks(ms,true,true);MobileBoard_ObserveLoad(ms,load);
}
static void advance(uint32_t until){for(;elapsed<until;elapsed+=100){if(elapsed%1000==0)inject();MobileBoard_Poll();MobileServoOutput_Poll();MobileHoldOutput_Poll();}}
static void w32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
int main(int argc,char **argv){
 CHECK(argc==2);unsigned scenario=(unsigned)atoi(argv[1]);
 CHECK(!MobileBoard_Boot(&left,&right));MobileBoard_Poll();CHECK(!reads&&!reader_starts);
 CHECK(ServoTransport_Register(&left)&&ServoTransport_Register(&right));
 if(scenario>=7&&scenario<=9){
  for(unsigned i=0;i<4;i++)modes[i]=0;
  CHECK(!MobileBoard_ProvisionModes(&profile.devices,false)&&!provisioning_writes);
  if(scenario==8)provisioning_bad_readback=true;
  if(scenario==9)torque=1;
  CHECK(MobileBoard_ProvisionModes(&profile.devices,true)==(scenario==7));
  CHECK(!MobileBoard_ProvisionModes(&profile.devices,true));
  CHECK(provisioning_writes==(scenario==7?12u:scenario==8?1u:0u));
  CHECK(!MobileBoard_IsConfigured());return 0;
 }

 MobileBoardProfile bad=profile;bad.stop.hold_budget_us=300;CHECK(!MobileBoard_Prepare(&left,&right,&bad,true));CHECK(!reads);
 CHECK(MobileBoard_Prepare(&left,&right,&profile,true));CHECK(reads==12&&!reader_starts&&!right_starts);
 advance(20000);CHECK(!MobileServoOutput_State()->configured&&ServoTransport_ServiceAllowed());
 if(scenario==4){MobileBoard_RequestStop();CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_UNCONFIRMED);CHECK(!MobileBoard_ArmsPrepared());return 0;}
 CHECK(MobileBoard_ArmsPrepared());CHECK(reader_starts==1&&right_starts==1);CHECK(MobileBoard_ArmsPrepared());CHECK(reader_starts==1);
 actuator_lift_t *lift=MobileBoard_LiftEndpoint()->lift;
 /* Independent simulated home establishes fixture state; no physical homing. */
 actuator_lift_feedback_t home={.observed_ms=HAL_GetTick(),.position_raw=2048,.healthy=true,.hold_verified=true};
 CHECK(actuator_lift_observe(lift,&home,HAL_GetTick()));
 lift->homed=true;lift->state=LIFT_HOLD_PENDING;
 advance(23000);CHECK(MobileServoOutput_State()->configured&&!ServoTransport_ServiceAllowed());
 int32_t initial[12]={0};BimanualFeedbackSnapshot_Seed(initial,scenario==1?HAL_GetTick()-100:HAL_GetTick());
 if(scenario==5||scenario==6){MobileArmObserver_Resume();advance(1000000);BimanualFeedbackSnapshot snapshot;BimanualFeedbackSnapshot_Copy(HAL_GetTick(),&snapshot);CHECK(snapshot.present_mask==4095);for(unsigned i=0;i<12;i++)CHECK(snapshot.sample_age_ms[i]<100);}
 if(scenario==6){
  while(!pending)advance(elapsed+1000);
  int32_t anchor[12];CHECK(MobileBoard_PrepareArmMotion(anchor));CHECK(!tracking&&!pending&&!MobileArmObserver_OwnsFeedback());
  CHECK(BimanualTrackingFeedback_Begin()==HAL_OK);advance(elapsed+10000);CHECK(tracking&&!pending);
  BimanualTrackingFeedback_End();MobileArmObserver_Resume();advance(elapsed+1000000);
 }
 if(scenario==10){
  uint8_t q[36]={'A','E',1,1},out[64];w32(q+4,1);w32(q+8,42);w32(q+12,HAL_GetTick());w32(q+16,HAL_GetTick()+30);q[20]=7;w32(q+32,actuator_crc32c(q,32));
  CHECK(MobileBoard_Evidence(q,out));CHECK(!MobileBoard_Evidence(q,out));
  advance(elapsed+31000);CHECK(MobileBoard_StopActive());CHECK(MobileBoard_StopState()->state!=SYSTEM_STOP_CONFIRMED);return 0;
 }
 stale_arms=scenario==1;load=scenario!=2;if(scenario==3)right.fail_tx=1;
 MobileBoard_RequestStop();MobileBoard_RequestStop();CHECK(stops==1);
 CHECK(!MobileServoOutput_CommandAllowed(MobileBoard_Supervisor()));
 uint32_t after=elapsed;advance(after+167000);
 if(scenario==0||scenario==5||scenario==6){
  CHECK(zero_writes==1&&hold_writes==2);
  CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_CONFIRMED);
  uint8_t q[36]={'A','Q',1,1},out[64];w32(q+4,7);w32(q+32,actuator_crc32c(q,32));
  CHECK(MobileBoard_StopQuery(q,out));for(unsigned i=0;i<64;i++)printf("%02x",out[i]);puts("");
  moved=true;advance(after+187000);CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_UNCONFIRMED);
 }else{advance(after+217000);CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_UNCONFIRMED);if(scenario==1)CHECK(!hold_writes);}
 CHECK(MobileBoard_StopActive());return 0;
}
