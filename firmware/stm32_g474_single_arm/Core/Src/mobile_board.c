#include "mobile_board.h"
#include "mobile_boot_identity.h"
#include "binary_control.h"
#include "servo_bus.h"
#include <string.h>
static MobileBoardProfile profile;
static actuator_mobile_supervisor_t supervisor;
static actuator_mobile_endpoint_t mobile_endpoint;
static actuator_lift_t lift;
static actuator_lift_endpoint_t lift_endpoint;
static bool prepared,output_ready,have_interlocks;
static uint32_t interlock_ms,last_lift_sample;
static bool arms_safe,lift_hold;
static UART_HandleTypeDef *left_uart;
actuator_lift_endpoint_t *MobileBoard_LiftEndpoint(void){return prepared?&lift_endpoint:NULL;}
actuator_mobile_supervisor_t *MobileBoard_Supervisor(void){return prepared?&supervisor:NULL;}
bool MobileBoard_Prepare(UART_HandleTypeDef *uart,const MobileBoardProfile *p,bool secured){
    if(prepared||uart==NULL||p==NULL||!secured||!ServoTransport_ServiceAllowed()||
       !p->current_microamps_per_raw||p->current_microamps_per_raw>1000000)return false;
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
       !MobileServoFeedback_Configure(uart,&supervisor,&p->feedback,&p->output.schedule,
            p->read_period_us,p->read_budget_us,p->quiet_us,secured)||
       !BinaryControl_AttachMobileEndpoint(&mobile_endpoint))return false;
    profile=*p;left_uart=uart;prepared=true;return true;
}
bool MobileBoard_Boot(UART_HandleTypeDef *uart){
#ifdef MOBILE_BOARD_PROFILE_HEADER
#include MOBILE_BOARD_PROFILE_HEADER
    return MobileBoard_Prepare(uart,&commissioned_mobile_profile,true);
#else
    (void)uart;return false;
#endif
}
void MobileBoard_ObserveInterlocks(uint32_t stamp,bool safe,bool held){
    if(have_interlocks&&(stamp-interlock_ms==0||stamp-interlock_ms>=UINT32_C(0x80000000))){have_interlocks=false;return;}
    interlock_ms=stamp;arms_safe=safe;lift_hold=held;have_interlocks=true;
}
void MobileBoard_Poll(void){
    if(!prepared)return;
    uint32_t now=HAL_GetTick();
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
    if(!output_ready&&actuator_mobile_axes_stopped(&supervisor,now)&&interlock_fresh&&arms_safe){
        output_ready=MobileServoOutput_Configure(left_uart,&supervisor,&profile.output,profile.quiet_us,true);
        if(output_ready&&!MobileServoOutput_BindLift(&lift_endpoint)){MobileServoOutput_Stop();output_ready=false;}
    }
    const actuator_mobile_output_t *o=MobileServoOutput_State();
    lift_endpoint.transport_ready=output_ready&&!o->stop_requested&&
        supervisor.state!=ACTUATOR_MOBILE_ACTIVE;
    actuator_lift_endpoint_poll(&lift_endpoint,now);
}
