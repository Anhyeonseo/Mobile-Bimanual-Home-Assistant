#include "mobile_arm_observer.h"
#include "bimanual_tracking_feedback.h"
#include "bimanual_feedback_snapshot.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"line %d time %u: %s\n",__LINE__,now,#x);exit(1);}}while(0)
static uint32_t now,stamp;
static uint8_t joint;
static bool active,pending,quiet=true,fail_end,fail_reply,missing;
static uint16_t shoulder=2048,other=2048;
static UART_HandleTypeDef left,right;
uint32_t HAL_GetTick(void){return now;}
bool ServoTransport_ReadReady(UART_HandleTypeDef *u){(void)u;return quiet;}
uint8_t BimanualTrackingFeedback_Active(void){return active;}
uint8_t BimanualTrackingFeedback_CanStart(void){return active&&!pending&&quiet;}
void BimanualTrackingFeedback_End(void){if(!fail_end){active=false;pending=false;}}
HAL_StatusTypeDef BimanualTrackingFeedback_Begin(void){CHECK(!active);active=true;return HAL_OK;}
HAL_StatusTypeDef BimanualTrackingFeedback_Start(uint8_t j,uint32_t ms,const uint16_t l[6],const uint16_t r[6],int32_t lp,int32_t rp){
 (void)l;(void)r;(void)lp;(void)rp;CHECK(active&&!pending);joint=j;stamp=ms;pending=true;return HAL_OK;
}
BimanualTrackingFeedbackResult BimanualTrackingFeedback_Poll(uint32_t ms,BimanualTrackingFeedbackSample *s){
 if(!pending)return BIMANUAL_TRACKING_IDLE;
 if(ms==stamp||missing)return BIMANUAL_TRACKING_PENDING;
 pending=false;
 if(fail_reply)return BIMANUAL_TRACKING_TRANSIENT_FAILURE;
 *s=(BimanualTrackingFeedbackSample){.joint_index=joint,.observed_ms=stamp,.left_position_raw=joint==1?shoulder:other,.right_position_raw=2048};
 return BIMANUAL_TRACKING_SAMPLE_READY;
}
static bool advance(uint32_t until){for(;now<until;now++)if(!MobileArmObserver_Poll())return false;return true;}
int main(int argc,char **argv){
 CHECK(argc==2);unsigned scenario=(unsigned)atoi(argv[1]);
 CHECK(MobileArmObserver_Poll()&&!MobileArmObserver_OwnsFeedback());
 CHECK(!MobileArmObserver_Configure(&left,&right,20,100));
 CHECK(MobileArmObserver_Configure(&left,&right,10,100));
 CHECK(!MobileArmObserver_Configure(&left,&right,10,100));
 CHECK(advance(1000)&&!active); /* No reads before first finite completion. */
 if(scenario==8){active=true;MobileArmObserver_Resume();CHECK(!MobileArmObserver_Poll());return 0;}
 if(scenario==5)now=UINT32_MAX-50;
 MobileArmObserver_Resume();
 if(scenario==5){for(unsigned i=0;i<200;i++,now++)CHECK(MobileArmObserver_Poll());return 0;}
 CHECK(advance(2000));BimanualFeedbackSnapshot s;BimanualFeedbackSnapshot_Copy(now,&s);
 CHECK(s.present_mask==4095&&s.completed_pairs>=99);
 for(unsigned i=0;i<12;i++)CHECK(s.sample_age_ms[i]<=60);
 if(scenario==0){CHECK(MobileArmObserver_Suspend()&&!active);CHECK(advance(3000));BimanualFeedbackSnapshot_Copy(now,&s);CHECK(s.sample_age_ms[0]>=1000);
  MobileArmObserver_Resume();CHECK(advance(3100));BimanualFeedbackSnapshot_Copy(now,&s);CHECK(s.sample_age_ms[0]<=60);return 0;}
 if(scenario==1){quiet=false;CHECK(!advance(2200));return 0;}
 if(scenario==2){other=100;CHECK(!advance(2100));BimanualFeedbackSnapshot_Copy(now,&s);CHECK(s.present_mask!=4095);return 0;}
 if(scenario==3){fail_end=true;CHECK(!MobileArmObserver_Suspend());CHECK(MobileArmObserver_OwnsFeedback()&&active&&!MobileArmObserver_Poll());return 0;}
 if(scenario==4){shoulder=4095;CHECK(advance(2080));BimanualFeedbackSnapshot_Copy(now,&s);int32_t a=s.positions_urad[1];
  shoulder=1;CHECK(advance(2160));BimanualFeedbackSnapshot_Copy(now,&s);CHECK(s.positions_urad[1]>a&&s.positions_urad[1]-a<4000);return 0;}
 if(scenario==6){missing=true;CHECK(!advance(2200));return 0;}
 if(scenario==7){fail_reply=true;CHECK(!advance(2100));BimanualFeedbackSnapshot_Copy(now,&s);CHECK(s.present_mask!=4095);return 0;}
 if(scenario==9){CHECK(MobileArmObserver_Suspend());MobileArmObserver_Resume();active=true;CHECK(!MobileArmObserver_Poll());return 0;}
 CHECK(false);return 1;
}
