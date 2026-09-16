#include "mobile_board.h"
#include "mobile_boot_identity.h"
#include "actuator_core/robot_evidence.h"
#include "binary_control.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include "mobile_hold_output.h"
#include "mobile_arm_observer.h"
#include "single_arm_config.h"
#include "timebase.h"
#define MOBILE_BOARD_SUPPORTED (HOST_BIMANUAL_TRACKING_FEEDBACK_BUILD && \
    HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD && HOST_BIMANUAL_DMA_DISPATCH_BUILD)
#include "actuator_core/arm_hold_monitor.h"
#if MOBILE_BOARD_SUPPORTED
#include "bimanual_feedback_snapshot.h"
#include "bimanual_operational_limits.h"
#include "bimanual_tracking_feedback.h"
#include "bimanual_servo_dispatch.h"
#endif
#include <string.h>
#include "actuator_core/crc32c.h"
static MobileBoardProfile profile;
static actuator_robot_evidence_t host_evidence;
static bool host_evidence_bound;
static actuator_mobile_supervisor_t supervisor;
static actuator_mobile_endpoint_t mobile_endpoint;
static actuator_lift_t lift;
static actuator_lift_endpoint_t lift_endpoint;
static bool prepared,runtime_started,output_ready,have_interlocks,stop_requested;
static bool have_load,load_retained,monitor_fault;
#if MOBILE_BOARD_SUPPORTED
static bool monitor_started;
#endif
static uint32_t load_ms;
#if MOBILE_BOARD_SUPPORTED
static uint32_t last_arm_read_ms;
#endif
static actuator_stop_proof_t last_stop_proof;
#if MOBILE_BOARD_SUPPORTED
static uint8_t monitor_joint;
#endif
static actuator_system_stop_t system_stop;
static actuator_arm_hold_monitor_t hold_monitor;
static uint32_t interlock_ms,last_lift_sample;
static bool arms_safe,lift_hold,have_interlock_stamp,have_load_stamp;
static UART_HandleTypeDef *left_uart,*right_uart;
actuator_lift_endpoint_t *MobileBoard_LiftEndpoint(void){return prepared?&lift_endpoint:NULL;}
actuator_mobile_supervisor_t *MobileBoard_Supervisor(void){return prepared?&supervisor:NULL;}
bool MobileBoard_Prepare(UART_HandleTypeDef *uart,UART_HandleTypeDef *right,const MobileBoardProfile *p,bool secured){
#if !MOBILE_BOARD_SUPPORTED
    (void)uart;(void)right;(void)p;(void)secured;return false;
#else
    if(prepared||uart==NULL||right==NULL||uart==right||p==NULL||!secured||!ServoTransport_ServiceAllowed()||
       !p->current_microamps_per_raw||p->current_microamps_per_raw>1000000)return false;
    /* Reject impossible STOP/hold budgets before any maintenance I/O. */
    if(!uart->Init.BaudRate||!right->Init.BaudRate||!p->quiet_us||
       !p->stop.timeout_ms||p->stop.timeout_ms>=UINT32_C(0x80000000)||
       !p->stop.feedback_max_age_ms||p->stop.feedback_max_age_ms>=UINT32_C(0x80000000)||
       p->stop.zero_budget_us!=p->output.stop_budget_us||
       p->stop.job_lifetime_us<=p->stop.zero_budget_us||
       p->stop.job_lifetime_us<=p->stop.hold_budget_us||p->stop.job_lifetime_us>1000000||
       !p->arm_hold_tolerance_raw||p->arm_hold_tolerance_raw>=2048||
       !p->arm_hold_dwell_ms||p->arm_hold_dwell_ms>=p->stop.timeout_ms||
       (uint64_t)p->arm_read_period_ms*1000<2u*(uint64_t)p->read_period_us||
       (uint64_t)p->arm_read_period_ms*6>=p->stop.feedback_max_age_ms||
       (UINT64_C(26)*10000000+uart->Init.BaudRate-1)/uart->Init.BaudRate+p->quiet_us>=p->stop.hold_budget_us||
       (UINT64_C(26)*10000000+right->Init.BaudRate-1)/right->Init.BaudRate+p->quiet_us>=p->stop.hold_budget_us)return false;
    actuator_device_startup_t startup={0};
    if(!actuator_device_startup_begin(&startup,&p->devices,HAL_GetTick(),false,secured))return false;
    /* Read-only default boot checks; explicit mode provisioning is separate. */
    for(unsigned step=0;startup.state==DEVICE_STARTUP_CHECKING&&step<32;step++){
        actuator_device_operation_t op;uint8_t data[16];
        if(!actuator_device_startup_next(&startup,HAL_GetTick(),&op)||op.write)return false;
        HAL_StatusTypeDef status=Servo_ReadData(op.id,op.address,op.length,data);
        if(!actuator_device_startup_reply(&startup,op.token,data,op.length,status==HAL_OK,HAL_GetTick()))return false;
    }
    uint32_t boot;
    if(startup.state!=DEVICE_STARTUP_READY||!MobileBootIdentity_Advance(p->initialize_erased_boot_storage,&boot)||
       !actuator_mobile_init(&supervisor,&p->mobile)||!actuator_lift_init(&lift,&p->lift)||
       !actuator_lift_endpoint_init(&lift_endpoint,&lift,boot,200)||
       !actuator_mobile_endpoint_init(&mobile_endpoint,&supervisor,boot)||
       !BinaryControl_AttachMobileEndpoint(&mobile_endpoint))return false;
    profile=*p;left_uart=uart;right_uart=right;prepared=true;return true;
#endif
}
bool MobileBoard_ProvisionModes(const actuator_device_profile_t *p,bool secured){
    if(prepared||!secured||!ServoTransport_ServiceAllowed())return false;
    static actuator_device_startup_t startup;
    if(startup.state!=DEVICE_STARTUP_IDLE)return false;
    if(!actuator_device_startup_begin(&startup,p,HAL_GetTick(),true,secured))return false;
    /* One attempt per transaction, no retry after uncertain EEPROM WRITE. */
    for(unsigned step=0;startup.state==DEVICE_STARTUP_CHECKING&&step<32;step++){
        actuator_device_operation_t op;uint8_t data[16];
        if(!actuator_device_startup_next(&startup,HAL_GetTick(),&op))return false;
        HAL_StatusTypeDef result=op.write?Servo_WriteData(op.id,op.address,op.data,op.length):
            Servo_ReadData(op.id,op.address,op.length,data);
        /* Servo_WriteData proves TX+quiet, not a servo ACK. Read back EVERY
         * written register before advancing the maintenance state machine. */
        if(op.write&&result==HAL_OK){
            result=Servo_ReadData(op.id,op.address,op.length,data);
            if(result==HAL_OK&&memcmp(data,op.data,op.length))result=HAL_ERROR;
        }
        if(!actuator_device_startup_reply(&startup,op.token,op.write?NULL:data,
                op.write?0:op.length,result==HAL_OK,HAL_GetTick()))return false;
    }
    return startup.state==DEVICE_STARTUP_READY;
}
bool MobileBoard_Evidence(const uint8_t q[36],uint8_t out[64]){
    if(!prepared)return false;
    uint32_t age=profile.stop.feedback_max_age_ms;
    if(profile.lift.feedback_timeout_ms<age)age=profile.lift.feedback_timeout_ms;
    if(!actuator_robot_evidence_accept(&host_evidence,mobile_endpoint.boot_id,age,HAL_GetTick(),q,out))return false;
    host_evidence_bound=true;
    MobileBoard_ObserveInterlocks(host_evidence.observed_ms,(host_evidence.flags&1)!=0,(host_evidence.flags&2)!=0);
    MobileBoard_ObserveLoad(host_evidence.observed_ms,(host_evidence.flags&4)!=0);
    return true;
}
bool MobileBoard_Boot(UART_HandleTypeDef *uart,UART_HandleTypeDef *right){
#ifdef MOBILE_BOARD_PROFILE_HEADER
#include MOBILE_BOARD_PROFILE_HEADER
    return MobileBoard_Prepare(uart,right,&commissioned_mobile_profile,true);
#else
    (void)uart;(void)right;return false;
#endif
}
bool MobileBoard_IsConfigured(void){return prepared;}
bool MobileBoard_StopActive(void){return stop_requested;}
const actuator_system_stop_t *MobileBoard_StopState(void){return &system_stop;}
bool MobileBoard_ArmsPrepared(void){
    if(!prepared)return true; /* Unconfigured legacy image. */
    if(stop_requested)return false;
    if(runtime_started)return true;
#if MOBILE_BOARD_SUPPORTED
    if(RightServoBus_PreparePeriodicReader()!=HAL_OK||
       !MobileHoldOutput_Configure(right_uart,profile.stop.hold_budget_us,profile.quiet_us)||
       !MobileServoFeedback_Configure(left_uart,&supervisor,&profile.feedback,&profile.output.schedule,
            profile.read_period_us,profile.read_budget_us,profile.quiet_us,true)||
       !MobileArmObserver_Configure(left_uart,right_uart,profile.arm_read_period_ms,profile.stop.feedback_max_age_ms)){
        MobileBoard_RequestStop();return false;
    }
    runtime_started=true;return true;
#else
    return false;
#endif
}
bool MobileBoard_PrepareArmMotion(int32_t anchor[12]){
    if(!prepared)return true;
    if(!runtime_started||stop_requested||!anchor||!MobileArmObserver_Suspend())return false;
#if MOBILE_BOARD_SUPPORTED
    /* The first trajectory uses the existing freshly prepared torque-off
     * anchor. Background observation starts after its finite completion. */
    if(MobileArmObserver_HasStarted()){
        BimanualFeedbackSnapshot snap;BimanualFeedbackSnapshot_Copy(HAL_GetTick(),&snap);
        if(snap.present_mask!=BIMANUAL_FEEDBACK_COMPLETE_MASK)return false;
        for(unsigned i=0;i<12;i++)if(snap.sample_age_ms[i]>=profile.stop.feedback_max_age_ms)return false;
        uint16_t lp[6],rp[6];uint8_t failed;
        if(BimanualOperationalLimits_MapExecutorOutput(snap.positions_urad,lp,rp,&failed)!=ACTUATOR_BIMANUAL_GOAL_MAP_OK)return false;
        memcpy(anchor,snap.positions_urad,sizeof(snap.positions_urad));
    }
    return true;
#else
    return false;
#endif
}
void MobileBoard_RequestStop(void){
    if(!prepared||stop_requested)return;
    stop_requested=true;
    MobileServoOutput_Stop();actuator_lift_endpoint_stop(&lift_endpoint);
    ServoTransport_RequestStop(left_uart);ServoTransport_RequestStop(right_uart);
    uint16_t lp[6]={0},rp[6]={0};bool valid=false;
#if MOBILE_BOARD_SUPPORTED
    BimanualFeedbackSnapshot snap;BimanualFeedbackSnapshot_Copy(HAL_GetTick(),&snap);
    valid=snap.present_mask==BIMANUAL_FEEDBACK_COMPLETE_MASK;
    for(unsigned i=0;i<12;i++)if(snap.sample_age_ms[i]>=profile.stop.feedback_max_age_ms)valid=false;
    uint8_t failed;
    valid=valid&&BimanualOperationalLimits_MapExecutorOutput(snap.positions_urad,lp,rp,&failed)==ACTUATOR_BIMANUAL_GOAL_MAP_OK;
    BimanualServoDispatch_LatchFault();
    (void)MobileArmObserver_Suspend();BimanualTrackingFeedback_End();
#endif
    actuator_bus_router_t *l=MobileServoOutput_StopRouter(),*r=MobileHoldOutput_Router();
    if(!l||!r){system_stop.state=SYSTEM_STOP_UNCONFIRMED;return;}
    (void)actuator_system_stop_begin_timed(&system_stop,l,r,lp,rp,valid,true,
        Timebase_NowUs(),HAL_GetTick(),&profile.stop);
}
void MobileBoard_ObserveLoad(uint32_t stamp,bool retained){
    if(HAL_GetTick()-stamp>=profile.stop.feedback_max_age_ms||
       (have_load_stamp&&(stamp-load_ms==0||stamp-load_ms>=UINT32_C(0x80000000)))){have_load=false;return;}
    load_ms=stamp;load_retained=retained;have_load=true;have_load_stamp=true;
}
static void poll_stop(uint32_t now,const actuator_mobile_feedback_reader_t *r,bool base_stopped,bool modes){
    if(!stop_requested)return;
    actuator_bus_router_t *left=MobileServoOutput_StopRouter(),*right=MobileHoldOutput_Router();
    if(!left||!right)return;
    actuator_stop_proof_t proof={0};
#if MOBILE_BOARD_SUPPORTED
    if(system_stop.actions_completed&&!monitor_started&&!monitor_fault&&
       ServoTransport_ReadReady(left_uart)&&ServoTransport_ReadReady(right_uart)){
        monitor_started=actuator_arm_hold_monitor_init(&hold_monitor,system_stop.left_positions,
            system_stop.right_positions,system_stop.actions_completed_ms,profile.arm_hold_tolerance_raw,
            profile.arm_hold_dwell_ms,profile.stop.feedback_max_age_ms)&&BimanualTrackingFeedback_Begin()==HAL_OK;
        monitor_fault=!monitor_started;
    }
    if(monitor_started&&!monitor_fault){
        BimanualTrackingFeedbackSample sample;
        BimanualTrackingFeedbackResult result=BimanualTrackingFeedback_Poll(now,&sample);
        if(result==BIMANUAL_TRACKING_SAMPLE_READY){
            int32_t unwrapped;
            if(!BimanualOperationalLimits_UnwrapModuloRaw(BIMANUAL_ARM_LEFT,sample.joint_index,
                    sample.left_position_raw,&unwrapped)||
               !BimanualOperationalLimits_UnwrapModuloRaw(BIMANUAL_ARM_RIGHT,sample.joint_index,
                    sample.right_position_raw,&unwrapped))monitor_fault=true;
            (void)actuator_arm_hold_monitor_observe(&hold_monitor,sample.joint_index,
                sample.left_position_raw,sample.right_position_raw,sample.observed_ms,now);
            monitor_joint=(sample.joint_index+1u)%6u;
        }else if(result==BIMANUAL_TRACKING_FAULT){monitor_fault=true;}
        if(!monitor_fault&&!BimanualTrackingFeedback_Pending()&&
           now-last_arm_read_ms>=profile.arm_read_period_ms&&
           ServoTransport_ReadReady(left_uart)&&ServoTransport_ReadReady(right_uart)&&
           !left->jobs[0].pending&&!right->jobs[0].pending){
            last_arm_read_ms=now;
            if(BimanualTrackingFeedback_Start(monitor_joint,now,system_stop.left_positions,
                system_stop.right_positions,0,0)!=HAL_OK)monitor_fault=true;
        }
    }
#endif
    uint32_t oldest=0,arms_ms=now;
    proof.arms_holding=!monitor_fault&&actuator_arm_hold_monitor_proof(&hold_monitor,now,&arms_ms);
    oldest=now-arms_ms;
    for(unsigned i=0;i<4;i++)if(now-r->axes[i].observed_ms>oldest)oldest=now-r->axes[i].observed_ms;
    if(now-interlock_ms>oldest)oldest=now-interlock_ms;
    if(now-load_ms>oldest)oldest=now-load_ms;
    proof.observed_ms=now-oldest;proof.base_stopped=base_stopped&&modes&&r->fault==MOBILE_FEEDBACK_OK;
    proof.lift_holding=lift.state==LIFT_HOLDING&&have_interlocks&&lift_hold&&
        now-interlock_ms<profile.lift.feedback_timeout_ms;
    proof.load_retained=have_load&&load_retained&&now-load_ms<profile.stop.feedback_max_age_ms;
    actuator_system_stop_poll_timed(&system_stop,left,right,&proof,Timebase_NowUs(),now);
    last_stop_proof=proof;
}
void MobileBoard_ObserveInterlocks(uint32_t stamp,bool safe,bool held){
    if(HAL_GetTick()-stamp>=profile.lift.feedback_timeout_ms||
       (have_interlock_stamp&&(stamp-interlock_ms==0||stamp-interlock_ms>=UINT32_C(0x80000000)))){have_interlocks=false;return;}
    interlock_ms=stamp;arms_safe=safe;lift_hold=held;have_interlocks=true;have_interlock_stamp=true;
}
void MobileBoard_Poll(void){
    if(!prepared||!runtime_started)return;
    uint32_t now=HAL_GetTick();
    if(host_evidence_bound&&!actuator_robot_evidence_fresh(&host_evidence,now,profile.stop.feedback_max_age_ms)){
        have_interlocks=false;have_load=false;
        if(output_ready&&!stop_requested)BinaryControl_LatchStop();
    }
    const actuator_mobile_feedback_reader_t *r=MobileServoFeedback_State();
    const actuator_mobile_axis_sample_t *sample=&r->axes[3];
    bool interlock_fresh=have_interlocks&&now-interlock_ms<profile.lift.feedback_timeout_ms;
    if(sample->valid&&(!lift.have_sample||sample->observed_ms!=last_lift_sample)){
        int32_t pos=sample->position_raw%4096;if(pos<0)pos+=4096;
        uint32_t current=(uint32_t)(sample->current_raw<0?-sample->current_raw:sample->current_raw);
        actuator_lift_feedback_t f={.observed_ms=sample->observed_ms,.position_raw=(uint16_t)pos,
            .velocity_raw=sample->velocity_raw,
            .current_ma=(int32_t)(((uint64_t)current*profile.current_microamps_per_raw+999)/1000),
            .healthy=sample->mode_verified&&r->fault==MOBILE_FEEDBACK_OK,
            .hold_verified=interlock_fresh&&lift_hold};
        (void)actuator_lift_observe(&lift,&f,now);last_lift_sample=sample->observed_ms;
        actuator_mobile_lift_evidence_t evidence={sample->observed_ms,lift.height_um,lift.homed,
            f.healthy&&lift.state!=LIFT_FAULT};
        MobileServoFeedback_SetLiftEvidence(&evidence);
    }
    if(r->fault!=MOBILE_FEEDBACK_OK)actuator_lift_inhibit(&lift,LIFT_BAD_FEEDBACK);
    actuator_lift_poll(&lift,now);
    bool base_stopped=true,modes=true;
    uint32_t oldest=interlock_fresh?now-interlock_ms:UINT32_MAX;
    for(unsigned i=0;i<4;i++){
        if(!r->axes[i].valid||!r->axes[i].mode_verified||now-r->axes[i].observed_ms>=profile.feedback.sample_timeout_ms)modes=false;
        uint32_t age=now-r->axes[i].observed_ms;if(age>oldest)oldest=age;
        if(i<3&&(r->axes[i].velocity_raw>profile.mobile.stopped_velocity_raw||
                 r->axes[i].velocity_raw<-profile.mobile.stopped_velocity_raw))base_stopped=false;
    }
    lift_endpoint.interlock=(actuator_lift_interlock_t){now-oldest,base_stopped,interlock_fresh&&arms_safe,modes,r->fault==MOBILE_FEEDBACK_OK};
    lift_endpoint.have_interlock=interlock_fresh&&modes;
    if(!stop_requested&&!output_ready&&actuator_mobile_axes_stopped(&supervisor,now)&&interlock_fresh&&arms_safe){
        output_ready=MobileServoOutput_Configure(left_uart,&supervisor,&profile.output,profile.quiet_us,true);
        if(output_ready&&!MobileServoOutput_BindLift(&lift_endpoint)){MobileServoOutput_Stop();output_ready=false;}
    }
    const actuator_mobile_output_t *o=MobileServoOutput_State();
    lift_endpoint.transport_ready=output_ready&&!o->stop_requested&&
        supervisor.state!=ACTUATOR_MOBILE_ACTIVE;
    actuator_lift_endpoint_poll(&lift_endpoint,now);
    if(o->configured&&o->stop_requested&&!stop_requested)BinaryControl_LatchStop();
    if(!stop_requested&&!MobileArmObserver_Poll())BinaryControl_LatchStop();
    poll_stop(now,r,base_stopped,modes);
}

static uint32_t read32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void write32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
bool MobileBoard_StopQuery(const uint8_t q[36],uint8_t out[64]){
    if(!prepared||!q||!out||memcmp(q,"AQ\x01\x01",4)||!read32(q+4)||
       read32(q+32)!=actuator_crc32c(q,32))return false;
    for(unsigned i=8;i<32;i++)if(q[i])return false;
    uint32_t now=HAL_GetTick(),age=now-last_stop_proof.observed_ms;
    uint8_t state=(uint8_t)system_stop.state;
    if(state==SYSTEM_STOP_CONFIRMED&&age>=profile.stop.feedback_max_age_ms)state=SYSTEM_STOP_UNCONFIRMED;
    memset(out,0,64);memcpy(out,"AT\x01\x01",4);
    write32(out+4,read32(q+4));write32(out+8,now);write32(out+12,mobile_endpoint.boot_id);
    write32(out+16,system_stop.started_ms);write32(out+20,system_stop.actions_completed_ms);
    out[24]=state;out[25]=1u|(runtime_started?2u:0u)|(output_ready?4u:0u)|
        (system_stop.actions_completed?8u:0u)|(last_stop_proof.base_stopped?16u:0u)|
        (last_stop_proof.lift_holding?32u:0u)|(last_stop_proof.arms_holding?64u:0u)|
        (last_stop_proof.load_retained?128u:0u);
    write32(out+28,last_stop_proof.observed_ms);write32(out+32,age);
    write32(out+36,profile.stop.feedback_max_age_ms);
    write32(out+60,actuator_crc32c(out,60));return true;
}
