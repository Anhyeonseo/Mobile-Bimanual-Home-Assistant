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
#include "actuator_core/mobile_ids.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"line %d us=%u: %s\n",__LINE__,elapsed,#x);exit(1);}}while(0)
static uint32_t elapsed;
void HAL_Delay(uint32_t ms){elapsed+=ms*1000;}
static UART_HandleTypeDef left={.Init={1000000},.gState=HAL_UART_STATE_READY};
static UART_HandleTypeDef right={.Init={1000000},.gState=HAL_UART_STATE_READY};
static actuator_mobile_feedback_reader_t reader;
static uint8_t modes[4]={1,1,1,1},locks[4]={1,1,1,1},torque;
static bool provisioning_bad_readback;
static unsigned provisioning_writes;
static unsigned inventory_calls;
static bool inventory_torque,inventory_changed,inventory_moving;
static bool wheel_noise,wheel_drift,wheel_missing;
static unsigned boot_lift_fault;
static unsigned maintenance_case;
static bool idle_speed_noise,idle_position_drift;
static unsigned powered_fault;
static unsigned inventory_change_after=34,power_writes;
static uint8_t mobile_torque[4];
static uint16_t mobile_goal[4];
static bool power_write_failure;
static bool recovery_arm_torque[2],recovery_arm_failure;
static unsigned recovery_arm_writes;
static bool inject_torque_loss;
static int lift_report_speed,lift_report_position;
static uint8_t lift_report_moving;
static bool test_base_lift_evidence;
static actuator_mobile_lift_evidence_t base_lift_evidence;
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
 .lift={409600,500000,100,100,20,2,10000,500,1000,100,1000,20,200,1,643847},
 .read_period_us=5000,.read_budget_us=600,.quiet_us=50,.current_microamps_per_raw=1000,
 .stop={4000,600,600,200,100},.arm_hold_tolerance_raw=5,.arm_hold_dwell_ms=20,.arm_read_period_ms=10
};
uint32_t Timebase_NowUs(void){return UINT32_MAX-10000u+elapsed;}
uint32_t HAL_GetTick(void){return elapsed/1000;}
bool ControlTick_PeekEpoch(uint32_t *epoch){*epoch=UINT32_MAX-10000u+(elapsed/5000)*5000;return true;}
bool ControlTick_PeekClock(uint32_t *now,uint32_t *epoch){bool valid=ControlTick_PeekEpoch(epoch);*now=Timebase_NowUs();return valid;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){
 CHECK(n==11||n==17||n==20||n==26);CHECK(p[4]==131&&p[6]==2);
 if(n==11||n==17){CHECK(u==&left&&p[5]==46);for(unsigned i=0;i<(n==11?1u:3u);i++)CHECK(!p[8+3*i]&&!p[9+3*i]);}
 else if(n==20){CHECK(u==&left&&p[5]==46);zero_writes++;for(unsigned i=0;i<4;i++)CHECK(p[7+3*i]==ACTUATOR_MOBILE_FIRST_ID+i&&!p[8+3*i]&&!p[9+3*i]);}
 else {CHECK(p[5]==42);if(u==&left)CHECK(zero_writes==1);hold_writes++;
  for(unsigned i=0;i<6;i++)CHECK(p[7+3*i]==i+1&&p[8+3*i]==0&&p[9+3*i]==8);}
 u->sent++;if(u->fail_tx)return HAL_ERROR;
 u->gState=HAL_UART_STATE_READY;MobileServoOutput_OnTxComplete(u);MobileHoldOutput_OnTxComplete(u);return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t timeout){(void)u;(void)p;(void)n;(void)timeout;CHECK(false);return HAL_ERROR;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){u->aborted++;u->gState=HAL_UART_STATE_READY;return HAL_OK;}
HAL_StatusTypeDef Servo_ReadData(uint8_t id,uint8_t addr,uint8_t n,uint8_t *out){
 reads++;CHECK(id>=ACTUATOR_MOBILE_FIRST_ID&&id<=ACTUATOR_LIFT_ID);if(addr==3){CHECK(n==4);out[0]=9;out[1]=3;out[2]=id;out[3]=0;}
 else if(addr==40){CHECK(n==1);out[0]=torque?torque:mobile_torque[id-ACTUATOR_MOBILE_FIRST_ID];}
 else if(addr==46){CHECK(n==2);out[0]=(uint8_t)mobile_goal[id-ACTUATOR_MOBILE_FIRST_ID];out[1]=(uint8_t)(mobile_goal[id-ACTUATOR_MOBILE_FIRST_ID]>>8);}
 else {CHECK(n==1&&(addr==33||addr==55));out[0]=addr==33?modes[id-ACTUATOR_MOBILE_FIRST_ID]:locks[id-ACTUATOR_MOBILE_FIRST_ID];}return HAL_OK;
}
void DeviceInventory_Read(const uint8_t *q,uint16_t n,bool permitted,uint8_t out[56]){
 CHECK(n==4&&permitted);memset(out,0,56);out[0]=1;out[4]=31;out[17]=q[2];out[36]=1;
 out[23]=(uint8_t)(q[2]+77+(inventory_changed&&inventory_calls>=inventory_change_after));
 out[26]=inventory_torque?1:0;out[47]=inventory_moving?1:0;
 if(recovery_arm_torque[q[1]])out[26]=1;
 if(q[1]==0&&q[2]>=ACTUATOR_MOBILE_FIRST_ID){
  unsigned axis=q[2]-ACTUATOR_MOBILE_FIRST_ID;
  out[26]=mobile_torque[axis];out[32]=(uint8_t)mobile_goal[axis];out[33]=(uint8_t)(mobile_goal[axis]>>8);
 }
 out[25]=q[1]==0&&q[2]>=ACTUATOR_MOBILE_FIRST_ID?modes[q[2]-ACTUATOR_MOBILE_FIRST_ID]:0;
 if(q[1]==0&&q[2]==10&&(mobile_torque[3]||maintenance_case)&&boot_lift_fault){
  out[39]=50;out[40]=128;out[47]=1;
  out[37]=100+(inventory_calls/16)%2;
  if(boot_lift_fault==2)out[37]=100+3*(inventory_calls/16);
  if(boot_lift_fault==3){out[1]=3;out[4]=15;}
  if(boot_lift_fault==4)out[39]=51;
  if(maintenance_case==81&&!mobile_torque[3])out[37]=100+3*(inventory_calls/16);
 }
 if(q[1]==0&&q[2]>=7&&q[2]<=9){
  if(wheel_missing){out[1]=3;out[4]=15;}
  if(wheel_noise){out[39]=50;out[40]=(inventory_calls%2)?128:0;out[47]=1;out[37]=100+inventory_calls%2;}
  if(wheel_drift)out[37]=(uint8_t)(inventory_calls*2);
 }
 if(q[1]==0&&q[2]==7&&mobile_torque[0]&&powered_fault){
  if(powered_fault==1)out[37]=100+((inventory_calls/4)%2)*3;
  if(powered_fault==2)out[39]=51;
  if(powered_fault==3)out[26]=0;
  if(powered_fault==4)out[32]=1;
 }
 inventory_calls++;
}
bool MobileBootIdentity_Advance(bool initialize,uint32_t *boot){(void)initialize;*boot=42;return true;}
HAL_StatusTypeDef Servo_DisableTorqueAll(void){recovery_arm_writes++;recovery_arm_torque[0]=false;return HAL_OK;}
HAL_StatusTypeDef RightServoBus_DisableTorqueAll(void){
 recovery_arm_writes++;if(recovery_arm_failure)return HAL_ERROR;
 recovery_arm_torque[1]=false;return HAL_OK;
}
void BinaryControl_LatchStop(void){MobileBoard_RequestStop();}
bool BinaryControl_AttachMobileEndpoint(actuator_mobile_endpoint_t *e){return e!=NULL;}
HAL_StatusTypeDef RightServoBus_PreparePeriodicReader(void){right_starts++;return HAL_OK;}
bool MobileServoFeedback_Configure(UART_HandleTypeDef *u,actuator_mobile_supervisor_t *s,
 const actuator_mobile_feedback_config_t *c,const actuator_bus_schedule_config_t *timing,
 uint32_t period,uint32_t budget,uint32_t quiet,bool proof){
 (void)timing;(void)period;(void)budget;(void)quiet;CHECK(u==&left&&proof&&ServoTransport_ServiceAllowed());
 reader_starts++;return actuator_mobile_feedback_reader_init(&reader,s,c);
}
bool MobileServoFeedback_EnterTransport(void){
 if(!reader.configured||reader.fault!=MOBILE_FEEDBACK_OK||reader.active)return false;
 reader.supervisor->transport_only=true;reader.cursor=0;reader.dirty=0;return true;
}
const actuator_mobile_feedback_reader_t *MobileServoFeedback_State(void){return &reader;}
void MobileServoFeedback_SetLiftEvidence(const actuator_mobile_lift_evidence_t *e){base_lift_evidence=*e;}
bool MobileServoFeedback_AcceptsOutput(const actuator_mobile_supervisor_t *s,const actuator_mobile_output_config_t *c){(void)c;return s==reader.supervisor;}
void BimanualServoDispatch_LatchFault(void){stops++;}
void BimanualTrackingFeedback_End(void){tracking=false;pending=false;}
HAL_StatusTypeDef BimanualTrackingFeedback_Begin(void){CHECK(!tracking);tracking=true;return HAL_OK;}
uint8_t BimanualTrackingFeedback_Active(void){return tracking;}
uint8_t BimanualTrackingFeedback_CanStart(void){return tracking&&!pending;}
uint8_t BimanualTrackingFeedback_Pending(void){return pending;}
HAL_StatusTypeDef BimanualTrackingFeedback_BeginObservation(void){return BimanualTrackingFeedback_Begin();}
HAL_StatusTypeDef BimanualTrackingFeedback_Start(uint8_t j,uint32_t stamp,const uint16_t l[6],const uint16_t r[6],int32_t lc,int32_t rc){
 (void)l;(void)r;(void)lc;(void)rc;CHECK(tracking&&!pending&&(MobileBoard_StopState()->actions_completed||MobileArmObserver_OwnsFeedback()));joint=j;read_stamp=stamp;pending=true;return HAL_OK;
}
BimanualTrackingFeedbackResult BimanualTrackingFeedback_Poll(uint32_t now,BimanualTrackingFeedbackSample *s){
 if(!pending||now==read_stamp)return BIMANUAL_TRACKING_PENDING;
 *s=(BimanualTrackingFeedbackSample){.joint_index=joint,.observed_ms=read_stamp,.left_position_raw=moved?2100:2048,.right_position_raw=2048};pending=false;return BIMANUAL_TRACKING_SAMPLE_READY;
}
HAL_StatusTypeDef Servo_WriteData(uint8_t id,uint8_t address,const uint8_t *data,uint8_t length){
 if(address==40||address==46){
  CHECK(id>=ACTUATOR_MOBILE_FIRST_ID&&id<=ACTUATOR_LIFT_ID);power_writes++;
  unsigned axis=id-ACTUATOR_MOBILE_FIRST_ID;
  if(address==46){CHECK(length==2&&!data[0]&&!data[1]&&modes[axis]==1);mobile_goal[axis]=0;mobile_torque[axis]=1;}
  else {CHECK(length==1&&data[0]<=1);mobile_torque[axis]=data[0];}
  if(power_write_failure&&address==40&&axis==1)return HAL_ERROR;
  return HAL_OK;
 }
 CHECK(id>=ACTUATOR_MOBILE_FIRST_ID&&id<=ACTUATOR_LIFT_ID&&length==1&&(address==33||address==55));provisioning_writes++;
 if(!provisioning_bad_readback){if(address==33)modes[id-ACTUATOR_MOBILE_FIRST_ID]=data[0];else locks[id-ACTUATOR_MOBILE_FIRST_ID]=data[0];}
 return HAL_OK;
}
static void inject(void){
 if(!reader.configured||!feed)return;
 uint32_t ms=HAL_GetTick();for(unsigned i=0;i<4;i++)reader.axes[i]=(actuator_mobile_axis_sample_t){.position_raw=2048,.observed_ms=ms,.mode_ms=ms,.valid=true,.mode_verified=true,.drive_valid=true,.torque_enabled=inject_torque_loss?0:1};
 for(unsigned i=0;i<3;i++){
  if(idle_speed_noise)reader.axes[i].velocity_raw=50;
  if(idle_position_drift)reader.axes[i].position_raw+=(ms/20)%100;
 }
 reader.axes[3].velocity_raw=lift_report_speed;reader.axes[3].moving=lift_report_moving;
 reader.axes[3].position_raw+=lift_report_position;
 actuator_mobile_feedback_t f={.observed_ms=ms,.velocity_modes_verified=true,.lift_homed=true,.hardware_ok=true};
 if(test_base_lift_evidence)f.lift_position_um=base_lift_evidence.position_um;
 (void)actuator_mobile_feedback(reader.supervisor,&f,ms);
 MobileBoard_ObserveInterlocks(ms,true,true);MobileBoard_ObserveLoad(ms,load);
}
static void advance(uint32_t until){for(;elapsed<until;elapsed+=100){if(elapsed%1000==0)inject();MobileBoard_Poll();MobileServoOutput_Poll();MobileHoldOutput_Poll();}}
static void w32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
#if HOST_S4_UNIFIED_CANDIDATE
static void power_request(uint8_t q[36],uint8_t out[64],uint8_t kind){
 memset(q,0,36);q[0]='A';q[1]='C';q[2]=1;q[3]=1;w32(q+4,100);w32(q+32,actuator_crc32c(q,32));
 CHECK(MobileBoard_Commission(q,out,true)&&out[3]==1);
 uint32_t ticket=(uint32_t)out[16]|((uint32_t)out[17]<<8)|((uint32_t)out[18]<<16)|((uint32_t)out[19]<<24);
 CHECK(ticket);q[3]=kind;w32(q+4,101);w32(q+8,ticket);w32(q+12,0x53454355);w32(q+32,actuator_crc32c(q,32));
}
#endif
int main(int argc,char **argv){
 CHECK(argc==2);unsigned scenario=(unsigned)atoi(argv[1]);
 unsigned top_case=scenario==90;
 if(top_case){scenario=42;test_base_lift_evidence=true;}
 (void)top_case;
 unsigned transport_case=scenario>=82&&scenario<=89?scenario:0;
 if(transport_case)scenario=42;
 (void)transport_case;
 if(scenario==80||scenario==81){maintenance_case=scenario;scenario=65;}
 unsigned bottom_case=scenario==79;
 if(bottom_case)scenario=42;
 (void)bottom_case;
 unsigned idle_case=scenario>=77&&scenario<=78?scenario:0;
 if(idle_case)scenario=42;
 (void)idle_case;
 unsigned reader_case=scenario>=75&&scenario<=76?scenario:0;
 if(reader_case)scenario=42;
 (void)reader_case;
 unsigned report_case=scenario>=46&&scenario<=49?scenario:0;
 if(report_case)scenario=42;
 (void)report_case;
 CHECK(!MobileBoard_Boot(&left,&right));MobileBoard_Poll();CHECK(!reads&&!reader_starts);
 CHECK(ServoTransport_Register(&left)&&ServoTransport_Register(&right));
#if HOST_S4_UNIFIED_CANDIDATE
 if(scenario>=70){
  uint8_t q[36],out[64];
  power_request(q,out,2);CHECK(MobileBoard_Commission(q,out,true)&&out[3]==2);
  wheel_noise=true;powered_fault=scenario-70;
  power_request(q,out,3);CHECK(MobileBoard_Commission(q,out,true));
  if(powered_fault){CHECK(out[3]==131&&!MobileBoard_PowerReady());return 0;}
  CHECK(out[3]==3&&out[24]==113&&MobileBoard_PowerReady());
  CHECK(power_writes==8&&!reader_starts&&!right_starts&&!zero_writes&&!hold_writes);
  power_request(q,out,4);CHECK(MobileBoard_Commission(q,out,true)&&out[3]==4&&out[24]==17);
  CHECK(power_writes==16&&!reader_starts&&!right_starts&&!zero_writes&&!hold_writes);
  return 0;
 }
 if(scenario>=50){
  uint8_t q[36],out[64];
  recovery_arm_torque[0]=recovery_arm_torque[1]=true;
  for(unsigned i=0;i<4;i++)mobile_torque[i]=1;
  power_request(q,out,5);
  if(scenario==51)inventory_moving=true;
  if(scenario==52)wheel_drift=true;
  if(scenario==53)wheel_missing=true;
  if(scenario==54)mobile_goal[2]=1;
  if(scenario==55)modes[3]=0;
  if(scenario==56){w32(q+12,0);w32(q+32,actuator_crc32c(q,32));}
  if(scenario==57)elapsed=2000000;
  if(scenario==58)power_write_failure=true;
  if(scenario==59)recovery_arm_failure=true;
  if(scenario==60)inventory_changed=true;
  if(scenario==63||scenario==64)wheel_noise=true;
  if(scenario==64)wheel_drift=true;
  if(scenario>=65&&scenario<=68)boot_lift_fault=scenario-64;
  if(scenario==69){boot_lift_fault=1;mobile_goal[3]=1;}
  if(scenario==61){
   recovery_arm_torque[0]=recovery_arm_torque[1]=false;
   for(unsigned i=0;i<4;i++)mobile_torque[i]=0;
   power_request(q,out,2);CHECK(MobileBoard_Commission(q,out,true)&&out[3]==2);
   power_request(q,out,5);
  }
  CHECK(MobileBoard_Commission(q,out,scenario!=62));
  if(scenario==50||scenario==63||(scenario==65&&maintenance_case!=81)){
   CHECK(out[3]==5&&!out[24]&&!out[25]&&!MobileBoard_IsConfigured());
   CHECK(power_writes==8&&recovery_arm_writes==2);
   for(unsigned i=0;i<4;i++)CHECK(!mobile_torque[i]&&!mobile_goal[i]);
   CHECK(!recovery_arm_torque[0]&&!recovery_arm_torque[1]);
   CHECK(!reader_starts&&!right_starts&&!provisioning_writes);
   /* Replayed release cannot write. Fresh preparation remains possible. */
   unsigned writes=power_writes;CHECK(MobileBoard_Commission(q,out,true)&&out[3]==133&&power_writes==writes);
   power_request(q,out,2);CHECK(MobileBoard_Commission(q,out,true)&&out[3]==2&&out[24]==17);
   if(maintenance_case==80){
    power_request(q,out,3);CHECK(MobileBoard_Commission(q,out,true)&&out[3]==3);
    power_request(q,out,4);CHECK(MobileBoard_Commission(q,out,true)&&out[3]==4);
    CHECK(!mobile_torque[3]&&!reader_starts&&!right_starts);
   }
  }else{
   CHECK(out[3]==133&&MobileBoard_IsConfigured()==(scenario==61));
   if(scenario!=58&&scenario!=59&&maintenance_case!=81)CHECK(!power_writes&&!recovery_arm_writes);
   unsigned writes=power_writes,arm_writes=recovery_arm_writes;
   CHECK(MobileBoard_Commission(q,out,true)&&out[3]==133);
   CHECK(power_writes==writes&&recovery_arm_writes==arm_writes);
  }
  return 0;
 }
 if(scenario>=20){
  uint8_t q[36]={'A','C',1,1},out[64];w32(q+4,1);w32(q+32,actuator_crc32c(q,32));
  modes[3]=0;
  CHECK(MobileBoard_Commission(q,out,true)&&out[3]==1&&!out[24]&&!reads&&!provisioning_writes);
  uint32_t ticket=(uint32_t)out[16]|((uint32_t)out[17]<<8)|((uint32_t)out[18]<<16)|((uint32_t)out[19]<<24);
  CHECK(ticket);
  q[3]=2;w32(q+4,2);w32(q+8,ticket);w32(q+12,0x53454355);
  if(scenario==21)inventory_torque=true;
  if(scenario==22)inventory_moving=true;
  if(scenario==23)inventory_changed=true;
  if(scenario==24)elapsed=2000000;
  if(scenario==25)w32(q+8,ticket+1);
  if(scenario==26)w32(q+12,0);
  if(scenario==28)provisioning_bad_readback=true;
  if(scenario>=43&&scenario<=45)wheel_noise=true;
  if(scenario==44)wheel_drift=true;
  if(scenario==45)wheel_missing=true;
  w32(q+32,actuator_crc32c(q,32));
  if(scenario==29){q[17]=1;CHECK(!MobileBoard_Commission(q,out,true));CHECK(!reads&&!provisioning_writes);return 0;}
  CHECK(MobileBoard_Commission(q,out,scenario!=27));
  if(scenario==20||(scenario>=30&&scenario<=43)){
   CHECK(out[3]==2&&out[24]==17&&out[25]==0&&provisioning_writes==3&&modes[3]==1);
   CHECK(MobileBoard_IsConfigured()&&MobileBoard_MaintenanceAllowed());
   CHECK(!reader_starts&&!right_starts&&!zero_writes&&!hold_writes);
   if(scenario==43)return 0;
   CHECK(MobileBoard_Commission(q,out,true)&&out[3]==130&&provisioning_writes==3);
   if(scenario>=30){
    CHECK(!MobileBoard_PowerReady());power_request(q,out,3);
    if(scenario==31)inventory_torque=true;
    if(scenario==32)mobile_goal[0]=1;
    if(scenario==33)modes[3]=0;
    if(scenario==34)elapsed+=2000000;
    if(scenario==35){w32(q+12,0);w32(q+32,actuator_crc32c(q,32));}
    if(scenario==37)power_write_failure=true;
    if(scenario==38){inventory_changed=true;inventory_change_after=inventory_calls+34;}
    CHECK(MobileBoard_Commission(q,out,scenario!=36));
    if(scenario>=31&&scenario<=38){
     CHECK(out[3]==131&&!MobileBoard_PowerReady());
     if(scenario<=36)CHECK(!power_writes);
     if(scenario==37){
      power_write_failure=false;power_request(q,out,4);CHECK(MobileBoard_Commission(q,out,true));
      CHECK(out[3]==4&&out[24]==17&&!MobileBoard_PowerReady());
      for(unsigned i=0;i<4;i++)CHECK(mobile_torque[i]==0);
     }
    }else{
     CHECK(out[3]==3&&out[24]==113&&MobileBoard_PowerReady());
     CHECK(power_writes==8&&!reader_starts&&!right_starts&&!zero_writes&&!hold_writes);
     for(unsigned i=0;i<4;i++)CHECK(mobile_torque[i]==1&&!mobile_goal[i]);
     if(scenario==42){
      CHECK(MobileBoard_ArmsPrepared());advance(elapsed+40000);
      CHECK(MobileServoOutput_State()->configured);
      actuator_lift_endpoint_t *endpoint=MobileBoard_LiftEndpoint();
      CHECK(!endpoint->lift->homed);
      if(idle_case){
       idle_speed_noise=true;idle_position_drift=idle_case==78;
       advance(elapsed+500000);
       CHECK(!MobileBoard_StopActive());
      }
      uint8_t ref[36]={'A','L',1,top_case?8:7},ls[64];
      if(top_case)w32(ref+16,643847);
      w32(ref+4,1);w32(ref+8,1);w32(ref+12,HAL_GetTick()+150);w32(ref+20,42);w32(ref+32,actuator_crc32c(ref,32));
      CHECK(actuator_lift_endpoint_exchange(endpoint,ref,36,HAL_GetTick(),ls));
      if(idle_case==78){CHECK(ls[3]==135&&!endpoint->active&&!endpoint->lift->homed);return 0;}
      CHECK(ls[3]==(top_case?8:7));
      CHECK(endpoint->lift->state==LIFT_HOLD_PENDING&&!endpoint->lift->zero_sent);
      advance(elapsed+30000);
      CHECK(endpoint->lift->state==LIFT_HOLDING&&endpoint->lift->zero_sent);
      MobileBoard_ObserveInterlocks(HAL_GetTick()+1,true,false);advance(elapsed+1000);
      CHECK(endpoint->lift->state==LIFT_HOLDING); /* no host fake-hold dependency */
      if(top_case){
       CHECK(endpoint->lift->height_um==643847&&endpoint->lift->target_um==643847);
       CHECK(!actuator_mobile_arm(reader.supervisor,1,HAL_GetTick()));
       int32_t held_arms[12]={0};BimanualFeedbackSnapshot_Seed(held_arms,HAL_GetTick());
       MobileBoard_RequestStop();advance(elapsed+500000);
       CHECK(endpoint->lift->state==LIFT_HOLDING&&endpoint->lift->height_um==643847);
       CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_CONFIRMED);return 0;
      }
      if(transport_case){
       MobileArmObserver_Resume();advance(elapsed+65000);
       /* Finish the current observation without launching another. */
       BimanualTrackingFeedbackSample finished;
       if(pending){elapsed+=1000;(void)BimanualTrackingFeedback_Poll(HAL_GetTick(),&finished);}
       int32_t positions[12]={0};BimanualFeedbackSnapshot_Seed(positions,HAL_GetTick());
       uint8_t ev[36]={'A','E',1,1},ack[64];uint32_t now=HAL_GetTick();
       w32(ev+4,1);w32(ev+8,42);w32(ev+12,now);w32(ev+16,now+100);ev[20]=5;
       w32(ev+32,actuator_crc32c(ev,32));CHECK(MobileBoard_Evidence(ev,ack));
       ev[3]=2;w32(ev+32,actuator_crc32c(ev,32));
       if(transport_case==83){BimanualFeedbackSnapshot_Reset();CHECK(!MobileBoard_Evidence(ev,ack));CHECK(!MobileBoard_TransportOnly());return 0;}
       if(transport_case==84){endpoint->lift->state=LIFT_FAULT;CHECK(!MobileBoard_Evidence(ev,ack));CHECK(!MobileBoard_TransportOnly());return 0;}
       if(transport_case==85){advance(elapsed+101000);CHECK(!MobileBoard_Evidence(ev,ack));CHECK(!MobileBoard_TransportOnly());return 0;}
       if(transport_case==86){pending=true;CHECK(MobileBoard_Evidence(ev,ack)&&ack[3]==130);CHECK(!MobileBoard_TransportOnly());pending=false;}
       if(transport_case==87){reader.active=true;CHECK(MobileBoard_Evidence(ev,ack)&&ack[3]==130);CHECK(!MobileBoard_TransportOnly());reader.active=false;}
       CHECK(MobileBoard_Evidence(ev,ack)&&ack[3]==2&&MobileBoard_TransportOnly());
       CHECK(!MobileBoard_LiftEndpoint());CHECK(!MobileBoard_PrepareArmMotion(positions));
       /* No false renewal and no new arm/lift observations for > stale limit. */
       BimanualFeedbackSnapshot_Reset();lift_report_speed=1000;lift_report_position=300;
       advance(elapsed+500000);CHECK(!MobileBoard_StopActive());CHECK(!tracking);
       CHECK(actuator_mobile_arm(reader.supervisor,1,HAL_GetTick()));
       CHECK(!actuator_mobile_command(reader.supervisor,1,1,(int32_t[4]){0,0,0,1},HAL_GetTick()));
       CHECK(actuator_mobile_command(reader.supervisor,1,1,(int32_t[4]){0,0,0,0},HAL_GetTick()));
       if(transport_case==88){reader.fault=MOBILE_FEEDBACK_DEVICE;advance(elapsed+1000);CHECK(MobileBoard_StopActive());return 0;}
       if(transport_case==89){advance(elapsed+210000);CHECK(MobileBoard_StopActive());return 0;}
       MobileBoard_RequestStop();advance(elapsed+500000);
       uint8_t sq[36]={'A','Q',1,1};w32(sq+4,7);w32(sq+32,actuator_crc32c(sq,32));
       CHECK(MobileBoard_StopQuery(sq,ack));CHECK(ack[24]==2&&ack[25]==31&&ack[26]==1);
       CHECK(MobileBoard_StopState()->state!=SYSTEM_STOP_CONFIRMED);CHECK(!hold_writes&&zero_writes==1);
       for(unsigned i=0;i<64;i++){printf("%02x",ack[i]);}puts("");return 0;
      }
      if(bottom_case){
       endpoint->lift->config.um_per_turn=84000;endpoint->lift->config.tolerance_um=2000;test_base_lift_evidence=true;
       CHECK(actuator_mobile_arm(reader.supervisor,1,HAL_GetTick()));
       lift_report_position=-1;advance(elapsed+5000);
       CHECK(endpoint->lift->height_um==-20&&base_lift_evidence.position_um==0);
       CHECK(reader.supervisor->state==ACTUATOR_MOBILE_READY&&!MobileBoard_StopActive());
       lift_report_position=-2;advance(elapsed+5000);
       CHECK(endpoint->lift->height_um==-41&&base_lift_evidence.position_um==0);
       CHECK(reader.supervisor->state==ACTUATOR_MOBILE_READY&&!MobileBoard_StopActive());
       lift_report_position=-90;lift_report_speed=-100;lift_report_moving=1;advance(elapsed+500000);
       CHECK(endpoint->lift->state==LIFT_HOLDING&&base_lift_evidence.position_um==0&&!MobileBoard_StopActive());
       int32_t held_bottom_arms[12]={0};BimanualFeedbackSnapshot_Seed(held_bottom_arms,HAL_GetTick());
       MobileBoard_RequestStop();advance(elapsed+500000);
       CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_CONFIRMED);
       lift_report_position=-100;advance(elapsed+5000);
       CHECK(base_lift_evidence.position_um<0&&MobileBoard_StopActive());return 0;
      }
      if(reader_case){
       reader.axis=reader_case==75?2:3;reader.fault=MOBILE_FEEDBACK_DEVICE;
       MobileBoard_Poll();
       uint32_t first[24];MobileBoard_Diagnostics(first);
       CHECK(first[0]==6&&first[2]==MOBILE_FEEDBACK_DEVICE&&MobileBoard_StopActive());
       if(reader_case==75){
        CHECK(endpoint->lift->state==LIFT_HOLDING);reader.fault=MOBILE_FEEDBACK_OK;
        advance(elapsed+50000);CHECK(endpoint->lift->state==LIFT_HOLDING);
       }else CHECK(endpoint->lift->state==LIFT_FAULT);
       return 0;
      }
      if(report_case){
       /* Elevated hold keeps the original precision/quiet-speed contract. */
       endpoint->lift->home_raw-=100;endpoint->lift->height_um=10000;endpoint->lift->target_um=10000;
       lift_report_speed=50;lift_report_moving=1;
       if(report_case==48)lift_report_position=1;
       if(report_case==49)lift_report_speed=51;
       advance(elapsed+1000);
       if(report_case==48){
        CHECK(endpoint->lift->state==LIFT_HOLDING&&!MobileBoard_StopActive());
        lift_report_position=3;advance(elapsed+1000);
        CHECK(endpoint->lift->state==LIFT_FAULT&&MobileBoard_StopActive());return 0;
       }
       if(report_case==49){CHECK(endpoint->lift->state==LIFT_FAULT&&MobileBoard_StopActive());return 0;}
       CHECK(endpoint->lift->state==LIFT_HOLDING&&!MobileBoard_StopActive());
       CHECK(endpoint->lift->feedback.velocity_raw==50&&endpoint->lift->feedback.hold_report_remaining_ms);
       if(report_case==47){
        advance(elapsed+1000000);CHECK(endpoint->lift->state==LIFT_HOLDING&&!MobileBoard_StopActive());
        feed=false;advance(elapsed+101000);CHECK(endpoint->lift->state==LIFT_FAULT&&MobileBoard_StopActive());return 0;
       }
       advance(elapsed+20000);int32_t held_arms[12]={0};BimanualFeedbackSnapshot_Seed(held_arms,HAL_GetTick());
       MobileBoard_RequestStop();advance(elapsed+50000);
       CHECK(endpoint->lift->state==LIFT_HOLDING&&MobileBoard_StopActive());
       CHECK(MobileBoard_StopState()->state!=SYSTEM_STOP_CONFIRMED);
       lift_report_speed=0;lift_report_moving=0;advance(elapsed+500000);
       CHECK(endpoint->lift->state==LIFT_HOLDING);
       CHECK(!endpoint->lift->feedback.hold_report_remaining_ms);
       CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_CONFIRMED);return 0;
      }
      if(idle_case==77){CHECK(!MobileBoard_StopActive()&&endpoint->lift->state==LIFT_HOLDING);return 0;}
      inject_torque_loss=true;advance(elapsed+1000);
      CHECK(endpoint->lift->state==LIFT_FAULT&&MobileBoard_StopActive());
      uint32_t first[24],later[24];MobileBoard_Diagnostics(first);
      CHECK(first[0]==2&&first[3]==LIFT_FAULT&&first[4]==LIFT_BAD_FEEDBACK&&first[12]==0);
      inject_torque_loss=false;advance(elapsed+50000);MobileBoard_RequestStop();MobileBoard_Diagnostics(later);
      CHECK(!memcmp(first,later,sizeof(first)));return 0;
     }
     CHECK(MobileBoard_Commission(q,out,true)&&out[3]==131&&power_writes==8);
     power_request(q,out,4);
     if(scenario==40){CHECK(MobileBoard_Commission(q,out,false)&&out[3]==132&&power_writes==8);return 0;}
     CHECK(MobileBoard_Commission(q,out,true)&&out[3]==4&&out[24]==17&&power_writes==16);
     for(unsigned i=0;i<4;i++)CHECK(mobile_torque[i]==0);
     CHECK(!MobileBoard_PowerReady()&&!reader_starts&&!right_starts&&!zero_writes&&!hold_writes);
     if(scenario==41){power_request(q,out,3);CHECK(MobileBoard_Commission(q,out,true)&&out[3]==3);}
    }
   }
  }else{
   CHECK(out[3]==130&&!MobileBoard_IsConfigured());
   CHECK(provisioning_writes==(scenario==23?3u:scenario==28?1u:0u));
   if(scenario==21||scenario==22||scenario==44||scenario==45){
    CHECK(!out[24]); /* No write happened; another explicit read preflight is legal. */
    q[3]=1;w32(q+8,0);w32(q+12,0);w32(q+32,actuator_crc32c(q,32));
    CHECK(MobileBoard_Commission(q,out,true)&&out[16]);
   }
  }
  return 0;
 }
#endif
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
  BimanualFeedbackSnapshot stop_snapshot;BimanualFeedbackSnapshot_Copy(HAL_GetTick(),&stop_snapshot);
  CHECK(stop_snapshot.present_mask==4095);
  for(unsigned i=0;i<12;i++)CHECK(stop_snapshot.sample_age_ms[i]<100);
  uint8_t q[36]={'A','Q',1,1},out[64];w32(q+4,7);w32(q+32,actuator_crc32c(q,32));
  CHECK(MobileBoard_StopQuery(q,out));for(unsigned i=0;i<64;i++)printf("%02x",out[i]);puts("");
  moved=true;advance(after+187000);CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_UNCONFIRMED);
 }else{advance(after+217000);CHECK(MobileBoard_StopState()->state==SYSTEM_STOP_UNCONFIRMED);if(scenario==1)CHECK(!hold_writes);}
 CHECK(MobileBoard_StopActive());return 0;
}
