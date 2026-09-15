#include "mobile_servo_feedback.h"
#include "mobile_servo_output.h"
#include "servo_bus_reader.h"
#include "control_tick.h"
#include "actuator_core/mobile_endpoint.h"
#include "actuator_core/crc32c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d time %u: %s\n",__LINE__,elapsed,#x);exit(1);}}while(0)
static uint32_t elapsed,origin,tx_at,producer,last_lift_ms=UINT32_MAX;
static uint8_t ring[256],packet[32];static size_t packet_size;
static bool rx_healthy=true,automatic=true,answer=true,reply_pending,clock_valid=true,provide_lift=true;
static bool corrupt,wrong_mode,duplicate,overflow;
static unsigned reads,writes,zero_writes;
static int32_t device_speed[4];
static bool check_direction;
static UART_HandleTypeDef left={.Init={1000000},.gState=HAL_UART_STATE_READY};
static UART_HandleTypeDef right={.Init={1000000},.gState=HAL_UART_STATE_READY};
static actuator_mobile_supervisor_t supervisor;
static const actuator_mobile_config_t limits={{1000,1000,1000,100},2,200,200,0,500000};
static const actuator_mobile_output_config_t output_config={{5000,1000,200,10000,20000,5000,500},500,400,600,4000,{1,-1,1,1}};
uint32_t Timebase_NowUs(void){return origin+elapsed;}
uint32_t HAL_GetTick(void){return elapsed/1000;}
bool ControlTick_PeekEpoch(uint32_t *epoch){*epoch=origin+(elapsed/5000)*5000;return clock_valid;}
static void complete(void){left.gState=HAL_UART_STATE_READY;MobileServoOutput_OnTxComplete(&left);MobileServoFeedback_OnTxComplete(&left);}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){
    CHECK(n<=sizeof(packet));memcpy(packet,p,n);packet_size=n;tx_at=elapsed;
    if(u->fail_tx)return HAL_ERROR;
    u->sent++;u->gState=HAL_UART_STATE_BUSY_TX;
    if(n==8){reads++;reply_pending=true;}else {writes++;if(n==20)zero_writes++;
        if(check_direction&&n==17)CHECK(p[10]==9&&p[11]==200&&p[12]==0x80);}
    if(u->immediate)complete();
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n){return HAL_UART_Transmit_DMA(u,p,n);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t timeout){
    (void)u;(void)p;(void)n;(void)timeout;CHECK(false);return HAL_ERROR;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){
    u->aborted++;if(u->fail_abort)return HAL_ERROR;u->gState=HAL_UART_STATE_READY;return HAL_OK;
}
bool ServoBus_PrepareReader(UART_HandleTypeDef *u,bool proof){return u==&left && proof && rx_healthy;}
bool ServoBus_RecoverReader(UART_HandleTypeDef *u,uint32_t token){
    if(!ServoTransport_Owns(u,token)||HAL_UART_AbortTransmit(u)!=HAL_OK)return false;
    rx_healthy=true;reply_pending=false;return true;
}
bool ServoBus_ReaderCursor(UART_HandleTypeDef *u,uint32_t token,uint32_t *p){
    if(!rx_healthy || !ServoTransport_Owns(u,token))return false;
    *p=producer;return true;
}
bool ServoBus_ReaderSlice(UART_HandleTypeDef *u,uint32_t token,uint32_t *p,uint8_t *b,size_t cap,size_t *n){
    if(!rx_healthy||!ServoTransport_Owns(u,token)||producer-*p>cap)return false;
    *n=producer-*p;for(size_t i=0;i<*n;i++)b[i]=ring[(*p+i)%256];*p=producer;return true;
}
bool ServoBus_ReaderQuiet(UART_HandleTypeDef *u,uint32_t token,uint32_t p){
    return rx_healthy&&ServoTransport_Owns(u,token)&&p==producer&&!u->wire_busy;
}
static void push_reply(void){
    uint8_t b[21]={255,255};size_t n=packet[6];b[2]=packet[2];b[3]=(uint8_t)(n+2);
    if(n==1)b[5]=wrong_mode?0:1;
    else {b[5]=100;b[11]=120;b[12]=30;b[18]=20;
        int32_t v=device_speed[packet[2]-8];
        uint16_t raw=(uint16_t)(v<0?-v:v);if(v<0)raw|=0x8000;
        b[7]=(uint8_t)raw;b[8]=(uint8_t)(raw>>8);
    }
    b[n+5]=actuator_sts3215_checksum(b+2,n+3);if(corrupt)b[n+5]^=1;
    for(unsigned repeat=0;repeat<(duplicate?2u:1u);repeat++)
        for(size_t i=0;i<n+6;i++)ring[producer++%256]=b[i];
    if(overflow)producer+=300;
}
static void poll(void){
    if(automatic&&left.gState==HAL_UART_STATE_BUSY_TX&&elapsed-tx_at>=100)complete();
    if(answer&&reply_pending&&elapsed-tx_at>=350){push_reply();reply_pending=false;}
    if(provide_lift&&last_lift_ms!=HAL_GetTick()){
        actuator_mobile_lift_evidence_t f={HAL_GetTick(),12000,true,true};
        MobileServoFeedback_SetLiftEvidence(&f);last_lift_ms=HAL_GetTick();
    }
    MobileServoOutput_Poll();MobileServoFeedback_Poll();
}
static void advance(uint32_t until){for(;elapsed<until;elapsed+=50)poll();poll();}
static void setup(void){
    CHECK(actuator_mobile_init(&supervisor,&limits));CHECK(ServoTransport_Register(&right));
    actuator_mobile_feedback_config_t f={.sample_timeout_ms=150,.mode_timeout_ms=400,.response_timeout_ms=5,.maximum_sample_skew_ms=40};
    for(unsigned i=0;i<4;i++)f.axes[i]=(actuator_mobile_axis_feedback_config_t){i==1?-1:1,90,140,70,1000};
    CHECK(!MobileServoFeedback_Configure(&left,&supervisor,&f,&output_config.schedule,5000,1000,50,false));
    CHECK(!MobileServoFeedback_Configure(&left,&supervisor,&f,&output_config.schedule,5000,100,50,true));
    CHECK(MobileServoFeedback_Configure(&left,&supervisor,&f,&output_config.schedule,5000,1000,50,true));
}
static void bind_output(void){
    advance(60000);CHECK(supervisor.have_feedback&&actuator_mobile_measured_stopped(&supervisor,HAL_GetTick()));
    actuator_mobile_output_config_t mismatched=output_config;mismatched.velocity_direction[1]=1;
    CHECK(!MobileServoOutput_Configure(&left,&supervisor,&mismatched,50,true));
    CHECK(MobileServoOutput_Configure(&left,&supervisor,&output_config,50,true));
    CHECK(actuator_mobile_arm(&supervisor,1,HAL_GetTick()));
}
int main(int argc,char **argv){
    CHECK(argc==2);unsigned which=(unsigned)atoi(argv[1]);uint32_t token;
    if(which==0){MobileServoFeedback_Poll();CHECK(left.sent==0);return 0;}
    if(which==12)origin=UINT32_MAX-3000;
    setup();
    switch(which){
    case 1: /* Continuous readback plus actual output/gate, no synthetic supervisor writes. */
        bind_output();advance(500000);
        CHECK(supervisor.state==ACTUATOR_MOBILE_READY&&reads>75&&writes>35);
        CHECK(MobileServoFeedback_State()->fault==MOBILE_FEEDBACK_OK);
        CHECK(supervisor.feedback.velocity_modes_verified&&supervisor.feedback.lift_homed);break;
    case 2: /* DMA completion alone cannot commit/release response-bearing lease. */
        answer=false;advance(1300);CHECK(left.gState==HAL_UART_STATE_READY&&reads==1);
        CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&token));
        CHECK(ServoTransport_Begin(&right,ACTUATOR_BUS_WORK_ARM,&token));
        CHECK(!MobileServoFeedback_State()->axes[0].mode_verified);
        advance(2500);CHECK(MobileServoFeedback_State()->fault!=MOBILE_FEEDBACK_OK);break;
    case 3: /* RX fault/failed recovery retains physical ownership. */
        advance(1100);rx_healthy=false;poll();
        CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_STOP,&token));
        left.fail_abort=1;CHECK(!MobileServoFeedback_RecoverTransport());
        CHECK(!MobileServoFeedback_Reset());
        CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_STOP,&token));
        left.fail_abort=0;CHECK(MobileServoFeedback_RecoverTransport());
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_STOP,&token));
        CHECK(ServoTransport_End(&left,token,false)==HAL_OK);
        CHECK(MobileServoFeedback_Reset());
        advance(60000);CHECK(supervisor.have_feedback);break;
    case 4: left.immediate=1;advance(60000);CHECK(supervisor.have_feedback);break;
    case 5: left.fail_tx=1;advance(2000);CHECK(MobileServoFeedback_State()->fault!=MOBILE_FEEDBACK_OK);
        CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_STOP,&token));break;
    case 6: bind_output();answer=false;advance(90000);
        CHECK(supervisor.state==ACTUATOR_MOBILE_STOP_LATCHED);
        CHECK(zero_writes==0); /* failed read still owns UART until explicit recovery */
        CHECK(MobileServoFeedback_RecoverTransport());CHECK(!MobileServoFeedback_Reset());
        advance(95000);CHECK(zero_writes==1);
        CHECK(MobileServoFeedback_Reset());answer=true;advance(160000);
        CHECK(supervisor.state==ACTUATOR_MOBILE_STOP_LATCHED&&supervisor.feedback.hardware_ok);
        CHECK(MobileServoOutput_Rearm(2,true));break;
    case 7: /* Post-STOP reads must work without clearing the output latch. */
        bind_output();MobileServoOutput_Stop();advance(140000);
        CHECK(zero_writes==1&&supervisor.state==ACTUATOR_MOBILE_STOP_LATCHED);
        CHECK(supervisor.feedback.observed_ms>MobileServoOutput_State()->stop_written_ms);
        CHECK(!MobileServoOutput_CommandAllowed(&supervisor));
        CHECK(MobileServoOutput_Rearm(2,true));advance(160000);CHECK(supervisor.state==ACTUATOR_MOBILE_READY);break;
    case 8: corrupt=true;advance(2500);CHECK(MobileServoFeedback_State()->fault!=MOBILE_FEEDBACK_OK);break;
    case 9: wrong_mode=true;advance(2500);CHECK(MobileServoFeedback_State()->fault==MOBILE_FEEDBACK_MODE);break;
    case 10: duplicate=true;advance(2500);CHECK(MobileServoFeedback_State()->fault==MOBILE_FEEDBACK_AMBIGUOUS);break;
    case 11: overflow=true;advance(2500);CHECK(MobileServoFeedback_State()->fault!=MOBILE_FEEDBACK_OK);break;
    case 12: bind_output();advance(100000);CHECK(supervisor.state==ACTUATOR_MOBILE_READY);break;
    case 13: /* Stop latch cannot be bypassed using a forged read-only write. */
    {
        ServoTransport_RequestStop(&left);CHECK(!ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&token));
        CHECK(ServoTransport_BeginReadOnly(&left,&token));uint8_t req[8];
        CHECK(actuator_sts3215_build_read(8,33,1,req)==ACTUATOR_STS3215_PACKET_OK);
        req[4]=3;req[7]=actuator_sts3215_checksum(req+2,5);
        CHECK(ServoTransport_TransmitDMA(&left,token,req,8)==HAL_ERROR);
        req[4]=2;req[7]=actuator_sts3215_checksum(req+2,5);
        CHECK(ServoTransport_TransmitDMA(&left,token,req,8)==HAL_OK);
        CHECK(ServoTransport_TransmitDMA(&left,token,req,8)==HAL_ERROR);
        CHECK(left.sent==1);break;
    }
    case 14: provide_lift=false;advance(60000);CHECK(supervisor.have_feedback&&!supervisor.feedback.lift_homed);
        CHECK(!MobileServoOutput_Configure(&left,&supervisor,&output_config,50,true));break;
    case 15: clock_valid=false;advance(60000);CHECK(reads==0&&!supervisor.have_feedback);break;
    case 16: bind_output();provide_lift=false;advance(230000);
        CHECK(supervisor.state==ACTUATOR_MOBILE_STOP_LATCHED&&!supervisor.feedback.hardware_ok);break;
    case 17: /* Wrong UART callback ignored; duplicate owned callback faults. */
        advance(1100);MobileServoFeedback_OnUartError(&right);poll();
        CHECK(MobileServoFeedback_State()->fault==MOBILE_FEEDBACK_OK);
        complete();complete();poll();CHECK(MobileServoFeedback_State()->fault!=MOBILE_FEEDBACK_OK);break;
    case 18: /* A legacy owner postpones the read without stealing the UART. */
        CHECK(ServoTransport_Begin(&left,ACTUATOR_BUS_WORK_ARM,&token));advance(10000);CHECK(reads==0);
        CHECK(ServoTransport_End(&left,token,false)==HAL_OK);advance(70000);CHECK(supervisor.have_feedback);break;
    case 19: /* AS bytes from decoded device frames, for the actual Python bridge. */
    {
        advance(60000);actuator_mobile_endpoint_t e;CHECK(actuator_mobile_endpoint_init(&e,&supervisor,77));
        uint8_t query[36]={'A','M',1,5,1},out[64];query[8]=7;
        uint32_t crc=actuator_crc32c(query,32);for(unsigned i=0;i<4;i++)query[32+i]=(uint8_t)(crc>>(8*i));
        bool ready=false;for(unsigned i=0;i<36;i++)ready=actuator_mobile_endpoint_feed(&e,query[i],HAL_GetTick(),out);
        CHECK(ready);for(unsigned i=0;i<64;i++)printf("%02x",out[i]);puts("");break;
    }
    case 20: /* The same measured motor sign governs command and feedback. */
    {
        bind_output();int32_t logical[4]={100,200,300,20};
        CHECK(actuator_mobile_command(&supervisor,1,1,logical,HAL_GetTick()));
        device_speed[0]=100;device_speed[1]=-200;device_speed[2]=300;device_speed[3]=20;
        check_direction=true;advance(110000);
        CHECK(supervisor.state==ACTUATOR_MOBILE_ACTIVE);
        CHECK(memcmp(supervisor.feedback.velocity_raw,logical,sizeof(logical))==0);break;
    }
    case 21: /* A read-only lease rejects malformed and broadcast packets. */
    {
        CHECK(ServoTransport_BeginReadOnly(&left,&token));uint8_t req[8],bad[8];
        CHECK(actuator_sts3215_build_read(8,33,1,req)==ACTUATOR_STS3215_PACKET_OK);
        for(unsigned i=0;i<8;i++){memcpy(bad,req,8);bad[i]^=1;
            CHECK(ServoTransport_TransmitDMA(&left,token,bad,8)==HAL_ERROR);}
        memcpy(bad,req,8);bad[2]=254;bad[7]=actuator_sts3215_checksum(bad+2,5);
        CHECK(ServoTransport_TransmitDMA(&left,token,bad,8)==HAL_ERROR);
        CHECK(left.sent==0);CHECK(ServoTransport_TransmitDMA(&left,token,req,8)==HAL_OK);break;
    }
    case 22: advance(1000);clock_valid=false;advance(1200);
        CHECK(reads==0&&MobileServoFeedback_State()->fault!=MOBILE_FEEDBACK_OK);break;
    case 23: /* Never launch into a continuously noisy preflight window. */
        advance(1000);for(;elapsed<2500;elapsed+=50){ring[producer++%256]=3;poll();}
        CHECK(reads==0&&MobileServoFeedback_State()->fault!=MOBILE_FEEDBACK_OK);break;
    case 24:
    {
        bind_output();provide_lift=false;
        actuator_mobile_lift_evidence_t old={HAL_GetTick(),12000,false,false};
        MobileServoFeedback_SetLiftEvidence(&old);poll();
        CHECK(supervisor.state==ACTUATOR_MOBILE_STOP_LATCHED);break;
    }
    default:CHECK(false);
    }
    return 0;
}
