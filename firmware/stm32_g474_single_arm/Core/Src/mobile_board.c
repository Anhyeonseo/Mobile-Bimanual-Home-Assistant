#include "mobile_board.h"
#include "mobile_boot_identity.h"
#include "actuator_core/robot_evidence.h"
#include "actuator_core/mobile_ids.h"
#include "binary_control.h"
#include "servo_bus.h"
#include "right_servo_bus.h"
#include "mobile_hold_output.h"
#include "mobile_arm_observer.h"
#include "single_arm_config.h"
#if HOST_S4_UNIFIED_CANDIDATE
#include "device_inventory.h"
#include MOBILE_BOARD_PROFILE_HEADER
static UART_HandleTypeDef *commission_left,*commission_right;
static uint32_t commission_ticket,commission_issued,commission_sequence;
static bool commission_attempted;
static bool power_attempted,power_enabled;
static bool recovery_attempted;
static uint8_t commission_reason;
static actuator_lift_hold_monitor_t lift_hold_monitor;
#endif
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
#include "actuator_core/wire_bytes.h"
static MobileBoardProfile profile;
static actuator_robot_evidence_t host_evidence;
static bool host_evidence_bound;
#if HOST_S4_UNIFIED_CANDIDATE
static uint32_t transport_admission_sequence;
#endif
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
static uint32_t first_stop_diagnostics[24];
#if HOST_S4_UNIFIED_CANDIDATE
static actuator_mobile_stationary_window_t wheel_stop_windows[3],wheel_idle_windows[3];
static uint32_t runtime_started_ms, park_zero_ms;
static bool park_tracking, park_wheels_still;
static actuator_mobile_stationary_window_t park_windows[3];

static bool park_proven(uint32_t now){
    return !stop_requested && park_tracking && park_wheels_still &&
        MobileServoOutput_CommandAllowed(&supervisor) && MobileArmObserver_OwnsFeedback() &&
        host_evidence_bound && actuator_robot_evidence_fresh(&host_evidence,now,profile.stop.feedback_max_age_ms) &&
        arms_safe && lift_hold && lift.homed && lift.state==LIFT_HOLDING && !lift_endpoint.active &&
        lift.zero_sent && !lift.command_raw && lift.have_sample && now-lift.observed_ms<profile.lift.feedback_timeout_ms &&
        actuator_mobile_wheels_stop_observe(park_windows,MobileServoFeedback_State(),park_zero_ms,now);
}

#endif
static void record_board_stop(uint32_t source){
    if(first_stop_diagnostics[0])return;
    const actuator_mobile_feedback_reader_t *r=MobileServoFeedback_State();
    const actuator_mobile_axis_sample_t *a=&r->axes[3];
    uint32_t *d=first_stop_diagnostics;
    d[0]=source;d[1]=HAL_GetTick();d[2]=r->fault;d[3]=lift.state;d[4]=lift.fault;
    d[5]=a->observed_ms;d[6]=a->mode_ms;d[7]=(uint32_t)a->position_raw;
    d[8]=(uint32_t)a->velocity_raw;d[9]=(uint32_t)a->current_raw;d[10]=a->moving;
    d[11]=(a->valid?1u:0u)|(a->mode_verified?2u:0u)|(a->drive_valid?4u:0u);
    d[12]=a->torque_enabled;d[13]=(uint32_t)a->goal_velocity_raw;
    d[14]=lift.feedback.healthy;d[15]=lift.feedback.hold_verified;
#if HOST_S4_UNIFIED_CANDIDATE
    d[16]=lift_hold_monitor.first_ms;d[17]=lift_hold_monitor.last_ms;
    d[18]=lift_hold_monitor.anchor_raw;
    d[19]=(lift_hold_monitor.tracking?1u:0u)|(lift_hold_monitor.proven?2u:0u);
#endif
    d[20]=interlock_ms;d[21]=have_interlocks;d[22]=lift.previous_raw;d[23]=(uint32_t)lift.height_um;
}
void MobileBoard_CopyPositions(uint32_t now,uint8_t out[44]){
    const actuator_mobile_feedback_reader_t *r=MobileServoFeedback_State();
    memset(out,0,44);
    out[0]=1;
    out[1]=r && r->configured;
    actuator_write_u32_le(out+4,now);
    for(unsigned i=0;i<4;i++){
        bool valid=out[1] && r->axes[i].valid;
        if(valid){
            out[2]|=(uint8_t)(1u<<i);
            actuator_write_u32_le(out+12+i*4,(uint32_t)r->axes[i].position_raw & 4095u);
        }
        actuator_write_u32_le(out+28+i*4,valid?now-r->axes[i].observed_ms:UINT32_MAX);
    }
    if(r)actuator_write_u32_le(out+8,r->axes[0].observed_ms);
}

void MobileBoard_Diagnostics(uint32_t out[24]){if(out)memcpy(out,first_stop_diagnostics,sizeof(first_stop_diagnostics));}
static void board_stop(uint32_t source){record_board_stop(source);BinaryControl_LatchStop();}
actuator_lift_endpoint_t *MobileBoard_LiftEndpoint(void){return prepared&&!supervisor.transport_only?&lift_endpoint:NULL;}
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
#if HOST_S4_UNIFIED_CANDIDATE
    mobile_endpoint.park_proven=park_proven;
    mobile_endpoint.cached_platform_positions=true;
#endif
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
static uint32_t read32(const uint8_t *p);
static void write32(uint8_t *p,uint32_t v);
static bool enter_transport(const uint8_t q[36],uint8_t out[64]){
#if HOST_S4_UNIFIED_CANDIDATE && MOBILE_BOARD_SUPPORTED
    uint32_t now=HAL_GetTick();
    if(!prepared||!q||!out||memcmp(q,"AE\x01\x02",4)||
       read32(q+32)!=actuator_crc32c(q,32))return false;
    for(unsigned i=21;i<32;i++)if(q[i])return false;
    /* The request references the last accepted initial measurement verbatim.
     * It cannot extend that measurement's life or create new arm evidence. */
    if(!host_evidence_bound||read32(q+4)!=host_evidence.sequence||
       read32(q+8)!=mobile_endpoint.boot_id||read32(q+12)!=host_evidence.observed_ms||
       read32(q+16)!=host_evidence.expires_ms||q[20]!=host_evidence.flags)return false;
    bool accepted=supervisor.transport_only&&transport_admission_sequence==read32(q+4);
    if(!accepted){
        if(stop_requested||!runtime_started||!output_ready||!power_enabled||
           !MobileArmObserver_OwnsFeedback()||
           supervisor.state!=ACTUATOR_MOBILE_DISABLED||supervisor.session||
           !actuator_robot_evidence_fresh(&host_evidence,now,profile.stop.feedback_max_age_ms)||
           (host_evidence.flags&5u)!=5u||!arms_safe||!lift_hold||
           lift.state!=LIFT_HOLDING||lift_endpoint.active||!lift.zero_sent||lift.command_raw||
           !lift.homed||!lift.have_sample||now-lift.observed_ms>=profile.lift.feedback_timeout_ms)
            return false;
        BimanualFeedbackSnapshot snap;BimanualFeedbackSnapshot_Copy(now,&snap);
        if(snap.present_mask!=BIMANUAL_FEEDBACK_COMPLETE_MASK)return false;
        for(unsigned i=0;i<12;i++)if(snap.sample_age_ms[i]>=profile.stop.feedback_max_age_ms)return false;
        /* Busy is an explicit non-admission, not an abort of a live read. */
        if(!BimanualTrackingFeedback_Pending() && MobileServoFeedback_EnterTransport()){
            if(!MobileArmObserver_Suspend()){board_stop(4);return false;}
            transport_admission_sequence=read32(q+4);accepted=true;
        }
    }
    memset(out,0,64);memcpy(out,"AF\x01\x02",4);if(!accepted)out[3]|=128u;
    write32(out+4,read32(q+4));write32(out+8,mobile_endpoint.boot_id);write32(out+12,now);
    write32(out+16,host_evidence.observed_ms);out[20]=host_evidence.flags;
    write32(out+60,actuator_crc32c(out,60));return true;
#else
    (void)q;(void)out;return false;
#endif
}
bool MobileBoard_Evidence(const uint8_t q[36],uint8_t out[64]){
    if(q&&q[3]==2)return enter_transport(q,out);
    if(!prepared||supervisor.transport_only)return false;
    uint32_t age=profile.stop.feedback_max_age_ms;
    if(profile.lift.feedback_timeout_ms<age)age=profile.lift.feedback_timeout_ms;
    if(!actuator_robot_evidence_accept(&host_evidence,mobile_endpoint.boot_id,age,HAL_GetTick(),q,out))return false;
    host_evidence_bound=true;
    MobileBoard_ObserveInterlocks(host_evidence.observed_ms,(host_evidence.flags&1)!=0,(host_evidence.flags&2)!=0);
    MobileBoard_ObserveLoad(host_evidence.observed_ms,(host_evidence.flags&4)!=0);
    return true;
}
bool MobileBoard_Boot(UART_HandleTypeDef *uart,UART_HandleTypeDef *right){
#if HOST_S4_UNIFIED_CANDIDATE
    commission_left=uart;commission_right=right;
    return false; /* No bus/flash writes or boot-time provisioning. */
#elif defined(MOBILE_BOARD_PROFILE_HEADER)
#include MOBILE_BOARD_PROFILE_HEADER
    return MobileBoard_Prepare(uart,right,&commissioned_mobile_profile,true);
#else
    (void)uart;(void)right;return false;
#endif
}
bool MobileBoard_IsConfigured(void){return prepared;}
bool MobileBoard_TransportOnly(void){return supervisor.transport_only;}
bool MobileBoard_PowerReady(void){
#if HOST_S4_UNIFIED_CANDIDATE
    return prepared&&power_enabled&&!commission_reason&&!stop_requested;
#else
    return true;
#endif
}
bool MobileBoard_MaintenanceAllowed(void){return !runtime_started&&!output_ready&&!stop_requested;}
bool MobileBoard_StopActive(void){return stop_requested;}
const actuator_system_stop_t *MobileBoard_StopState(void){return &system_stop;}
bool MobileBoard_ArmsPrepared(void){
    if(!prepared)return true; /* Unconfigured legacy image. */
    if(!MobileBoard_PowerReady())return false;
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
#if HOST_S4_UNIFIED_CANDIDATE
    runtime_started_ms=HAL_GetTick();
#endif
    runtime_started=true;return true;
#else
    return false;
#endif
}
bool MobileBoard_PrepareArmMotion(int32_t anchor[12]){
    if(!prepared)return true;
    if(!runtime_started||stop_requested||supervisor.transport_only||supervisor.state!=ACTUATOR_MOBILE_DISABLED||supervisor.session||!anchor||!MobileArmObserver_Suspend())return false;
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
    record_board_stop(5);stop_requested=true;
    MobileServoOutput_Stop();actuator_lift_endpoint_stop(&lift_endpoint);
    ServoTransport_RequestStop(left_uart);
    if(supervisor.transport_only){
        /* Arm goals and torque are already resident. Stop wheels without
         * requesting arm writes/observations over an unrelated UART. */
        system_stop.started_ms=HAL_GetTick();system_stop.state=SYSTEM_STOP_PENDING;
        return;
    }
    ServoTransport_RequestStop(right_uart);
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
    if(supervisor.transport_only){
        const actuator_mobile_output_t *o=MobileServoOutput_State();
        uint32_t age=0;bool newer=o->stop_written;
        for(unsigned i=0;i<3;i++){
            uint32_t a=now-r->axes[i].observed_ms;if(a>age)age=a;
            uint32_t delta=r->axes[i].observed_ms-o->stop_written_ms;
            newer=newer&&delta>0&&delta<UINT32_C(0x80000000);
        }
        last_stop_proof=(actuator_stop_proof_t){.observed_ms=now-age,
            .base_stopped=newer&&base_stopped&&modes&&r->fault==MOBILE_FEEDBACK_OK};
        system_stop.actions_completed=o->stop_written;system_stop.actions_completed_ms=o->stop_written_ms;
        /* Whole-system proof stays unconfirmed; AQ scope=1 separately proves wheels. */
        system_stop.state=SYSTEM_STOP_UNCONFIRMED;return;
    }
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
            int32_t unwrapped[2],position[2];
            bool valid=true;
            for(unsigned side=0;side<2;side++){
                BimanualArm arm=side?BIMANUAL_ARM_RIGHT:BIMANUAL_ARM_LEFT;
                const BimanualOperationalLimit *limit=BimanualOperationalLimits_Get(arm,sample.joint_index);
                uint16_t raw=side?sample.right_position_raw:sample.left_position_raw;
                if(!limit||!BimanualOperationalLimits_UnwrapModuloRaw(arm,sample.joint_index,raw,&unwrapped[side])||
                   actuator_unwrapped_raw_to_urad(limit->zero_raw,limit->positive_raw_direction,
                       unwrapped[side],&position[side])!=ACTUATOR_UNWRAP_OK||
                   position[side]<limit->urad.minimum_urad||position[side]>limit->urad.maximum_urad)valid=false;
            }
            if(!valid){
                monitor_fault=true;BimanualFeedbackSnapshot_InvalidatePair(sample.joint_index);
            }else{
                /* STOP owns the UART now; keep the public measured snapshot
                 * advancing from these actual reads, never from hold goals. */
                BimanualFeedbackSnapshot_UpdatePair(sample.joint_index,position[0],position[1],sample.observed_ms);
            }
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
    bool bottom_after_stop=actuator_lift_bottom_transport_ready(&lift,now)&&system_stop.actions_completed&&
        lift.observed_ms-system_stop.actions_completed_ms>0&&
        lift.observed_ms-system_stop.actions_completed_ms<UINT32_C(0x80000000)&&
        lift.feedback.bottom_drive_observed_ms-system_stop.actions_completed_ms>0&&
        lift.feedback.bottom_drive_observed_ms-system_stop.actions_completed_ms<UINT32_C(0x80000000);
    proof.lift_holding=lift.state==LIFT_HOLDING&&have_interlocks&&lift_hold&&
        (bottom_after_stop||(!lift.feedback.bottom_transport_verified&&!lift.feedback.hold_report_remaining_ms))&&
        now-interlock_ms<profile.lift.feedback_timeout_ms;
    proof.load_retained=have_load&&load_retained&&now-load_ms<profile.stop.feedback_max_age_ms;
    actuator_system_stop_poll_timed(&system_stop,left,right,&proof,Timebase_NowUs(),now);
    last_stop_proof=proof;
}
void MobileBoard_ObserveInterlocks(uint32_t stamp,bool safe,bool held){
    if(HAL_GetTick()-stamp>=profile.lift.feedback_timeout_ms||
       (have_interlock_stamp&&(stamp-interlock_ms==0||stamp-interlock_ms>=UINT32_C(0x80000000)))){have_interlocks=false;return;}
    interlock_ms=stamp;arms_safe=safe;
#if HOST_S4_UNIFIED_CANDIDATE
    (void)held; /* Actual drive/position readback owns lift holding. */
#else
    lift_hold=held;
#endif
    have_interlocks=true;have_interlock_stamp=true;
}
void MobileBoard_Poll(void){
    if(!prepared||!runtime_started)return;
    uint32_t now=HAL_GetTick();
    if(!supervisor.transport_only&&host_evidence_bound&&!actuator_robot_evidence_fresh(&host_evidence,now,profile.stop.feedback_max_age_ms)){
        have_interlocks=false;have_load=false;
        if(output_ready&&!stop_requested)board_stop(1);
    }
    const actuator_mobile_feedback_reader_t *r=MobileServoFeedback_State();
    const actuator_mobile_axis_sample_t *sample=&r->axes[3];
    bool interlock_fresh=have_interlocks&&now-interlock_ms<profile.lift.feedback_timeout_ms;
    if(!supervisor.transport_only&&sample->valid&&(!lift.have_sample||sample->observed_ms!=last_lift_sample)){
        int32_t pos=sample->position_raw%4096;if(pos<0)pos+=4096;
        uint32_t current=(uint32_t)(sample->current_raw<0?-sample->current_raw:sample->current_raw);
        actuator_lift_feedback_t f={.observed_ms=sample->observed_ms,.position_raw=(uint16_t)pos,
            .velocity_raw=sample->velocity_raw,
            .current_ma=(int32_t)(((uint64_t)current*profile.current_microamps_per_raw+999)/1000),
            .healthy=sample->mode_verified&&r->fault==MOBILE_FEEDBACK_OK,
            .hold_verified=interlock_fresh&&lift_hold};
#if HOST_S4_UNIFIED_CANDIDATE
        bool drive=sample->drive_valid&&sample->torque_enabled==1&&sample->goal_velocity_raw==0&&
            now-sample->mode_ms<profile.lift.feedback_timeout_ms&&
            f.current_ma<=profile.lift.maximum_current_ma;
        f.bottom_transport_verified=actuator_lift_bottom_transport_observe(&lift,&f,drive,sample->mode_ms,now);
        f.bottom_drive_observed_ms=sample->mode_ms;
        if(f.bottom_transport_verified){
            lift_hold=true;
        }else{
            /* Holding accuracy uses the same commissioned millimetre envelope
             * as arrival. The initial zero-speed dwell remains unchanged. */
            uint64_t counts=(uint64_t)profile.lift.tolerance_um*4096u/profile.lift.um_per_turn;
            uint16_t hold_tolerance=(uint16_t)(counts<2?2:counts>2047?2047:counts);
            lift_hold=actuator_lift_hold_observe_report(&lift_hold_monitor,&f,drive,sample->moving,
                lift.state==LIFT_HOLDING,now,
                profile.lift.feedback_timeout_ms,profile.arm_hold_dwell_ms,2,hold_tolerance);
        }
        f.hold_verified=lift_hold;
#endif
        (void)actuator_lift_observe(&lift,&f,now);last_lift_sample=sample->observed_ms;
        actuator_mobile_lift_evidence_t evidence={sample->observed_ms,actuator_lift_base_height(&lift,now),lift.homed,
            f.healthy&&lift.state!=LIFT_FAULT};
        MobileServoFeedback_SetLiftEvidence(&evidence);
    }
#if HOST_S4_UNIFIED_CANDIDATE
    if(r->fault!=MOBILE_FEEDBACK_OK){
        /* A wheel observation failure stops motion, but is not evidence that
         * the independently held lift has physically failed. Fresh lift age
         * and its own position/current/hold checks remain mandatory. */
        if(!stop_requested)board_stop(6);
        if(r->axis==3)actuator_lift_inhibit(&lift,LIFT_BAD_FEEDBACK);
    }
#else
    if(r->fault!=MOBILE_FEEDBACK_OK)actuator_lift_inhibit(&lift,LIFT_BAD_FEEDBACK);
#endif
    if(!supervisor.transport_only)actuator_lift_poll(&lift,now);
#if HOST_S4_UNIFIED_CANDIDATE
    bool top_hold = !supervisor.transport_only && !stop_requested && !lift_endpoint.active &&
        sample->drive_valid && sample->torque_enabled == 1 && sample->goal_velocity_raw == 0 &&
        now - sample->mode_ms < profile.lift.feedback_timeout_ms &&
        profile.lift.reference_top_um == supervisor.config.lift_max_um &&
        actuator_lift_top_hold_verified(&lift, now);
    supervisor.top_hold_tolerance_um = top_hold ? profile.lift.tolerance_um : 0;
    if(!supervisor.transport_only&&lift.state==LIFT_FAULT&&!stop_requested)board_stop(2);
#endif
    bool base_stopped=true,modes=true;
    uint32_t oldest=interlock_fresh?now-interlock_ms:UINT32_MAX;
    for(unsigned i=0;i<(supervisor.transport_only?3u:4u);i++){
        if(!r->axes[i].valid||!r->axes[i].mode_verified||now-r->axes[i].observed_ms>=profile.feedback.sample_timeout_ms)modes=false;
        uint32_t age=now-r->axes[i].observed_ms;if(age>oldest)oldest=age;
        if(i<3&&(r->axes[i].velocity_raw>profile.mobile.stopped_velocity_raw||
                 r->axes[i].velocity_raw<-profile.mobile.stopped_velocity_raw))base_stopped=false;
    }
#if HOST_S4_UNIFIED_CANDIDATE
    bool park_zero=!stop_requested && !supervisor.transport_only && supervisor.state==ACTUATOR_MOBILE_ACTIVE &&
        !supervisor.target_raw[0] && !supervisor.target_raw[1] && !supervisor.target_raw[2] && !supervisor.target_raw[3];
    if(park_zero){
        if(!park_tracking){park_tracking=true;park_zero_ms=now;memset(park_windows,0,sizeof(park_windows));}
        park_wheels_still=actuator_mobile_wheels_stop_observe(park_windows,r,park_zero_ms,now);
    }else{park_tracking=false;park_wheels_still=false;}
    /* Supported power activation has already written/read back wheel-zero.
     * Before the first base ARM only, corroborate noisy stationary speed bytes
     * with the same complete position window used for STOP. Never use this
     * pre-ARM anchor after a motion session or to hide missing/changed feedback. */
    if(power_enabled&&!stop_requested&&supervisor.state==ACTUATOR_MOBILE_DISABLED&&!supervisor.session){
        bool idle_proof=actuator_mobile_wheels_stop_observe(wheel_idle_windows,r,runtime_started_ms,now);
        base_stopped=base_stopped||idle_proof;
    }else memset(wheel_idle_windows,0,sizeof(wheel_idle_windows));
#endif
    lift_endpoint.interlock=(actuator_lift_interlock_t){now-oldest,base_stopped,interlock_fresh&&arms_safe,modes,r->fault==MOBILE_FEEDBACK_OK};
    lift_endpoint.have_interlock=interlock_fresh&&modes;
    bool configure_allowed=interlock_fresh&&arms_safe;
#if HOST_S4_UNIFIED_CANDIDATE
    /* Bind the zero/STOP writer before homing/interlocks; this does not arm
     * a motion session. AL/AM admission still requires fresh interlocks. */
    configure_allowed=MobileBoard_PowerReady();
#endif
    if(!stop_requested&&!output_ready&&actuator_mobile_axes_stopped(&supervisor,now)&&configure_allowed){
        output_ready=MobileServoOutput_Configure(left_uart,&supervisor,&profile.output,profile.quiet_us,true);
        if(output_ready&&!MobileServoOutput_BindLift(&lift_endpoint)){MobileServoOutput_Stop();output_ready=false;}
    }
    const actuator_mobile_output_t *o=MobileServoOutput_State();
    lift_endpoint.transport_ready=output_ready&&!o->stop_requested&&
        supervisor.state!=ACTUATOR_MOBILE_ACTIVE;
    if(!supervisor.transport_only)actuator_lift_endpoint_poll(&lift_endpoint,now);
    if(o->configured&&o->stop_requested&&!stop_requested)board_stop(3);
    if(!supervisor.transport_only&&!stop_requested&&!MobileArmObserver_Poll())board_stop(4);
#if HOST_S4_UNIFIED_CANDIDATE
    if(stop_requested && o->stop_written){
        bool position_proof=actuator_mobile_wheels_stop_observe(wheel_stop_windows,r,o->stop_written_ms,now);
        base_stopped=base_stopped||position_proof;
    }
#endif
    poll_stop(now,r,base_stopped,modes);
}

static uint32_t read32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void write32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
#if HOST_S4_UNIFIED_CANDIDATE
static bool wheel_snapshot(const uint8_t d[56],uint8_t expected_torque){
    uint16_t speed=(uint16_t)(d[39]|((uint16_t)d[40]<<8));
    return !d[1]&&d[4]==31&&d[26]==expected_torque&&d[36]==1&&
        d[25]==1&&!d[32]&&!d[33]&&(speed&0x7fff)<=50&&d[47]<=1;
}
static bool off_snapshot(const uint8_t d[56],bool wheel){
    if(wheel)return wheel_snapshot(d,0);
    return !d[1]&&d[4]==31&&!d[26]&&d[36]==1&&!d[39]&&!d[40]&&!d[47];
}
static bool maintenance_axes_quiet(uint8_t torque_mask,unsigned maximum_span){
    /* Supported maintenance only, all four speed-mode axes: zero goal and six independent position
     * reads over >=400 ms. Powered admission uses <=2 encoder counts;
     * torque-OFF wheels retain <=8 counts, lift always <=2. Runtime motion
     * and STOP admission are unchanged. Never infer rest from one speed byte. */
    int first[4]={0},low[4]={0},high[4]={0};uint32_t start=HAL_GetTick();
    for(unsigned pass=0;pass<6;pass++){
        if(pass)HAL_Delay(80);
        for(unsigned axis=0;axis<4;axis++){
            uint8_t q[4]={1,0,ACTUATOR_MOBILE_FIRST_ID+axis,0},d[56];
            DeviceInventory_Read(q,4,true,d);
            if(d[17]!=q[2]||((axis==3&&d[25]==0)?
                (((torque_mask>>axis)&1)||!off_snapshot(d,false)):
                !wheel_snapshot(d,(torque_mask>>axis)&1)))return false;
            int pos=d[37]|((int)d[38]<<8);if(pos>4095)return false;
            if(!pass)first[axis]=pos;
            int delta=pos-first[axis];if(delta>2048)delta-=4096;if(delta< -2048)delta+=4096;
            if(delta<low[axis])low[axis]=delta;
            if(delta>high[axis])high[axis]=delta;
            if((unsigned)(high[axis]-low[axis])>(axis==3?2:maximum_span))return false;
        }
    }
    return HAL_GetTick()-start>=400;
}
static bool wheels_off_quiet(void){return maintenance_axes_quiet(0,8);}
static bool write_mobile_checked(uint8_t id,uint8_t address,const uint8_t *value,uint8_t size){
    uint8_t got[2];
    return Servo_WriteData(id,address,value,size)==HAL_OK&&
        Servo_ReadData(id,address,size,got)==HAL_OK&&!memcmp(value,got,size);
}
static bool secured_mobile_power(bool enable){
    /* Maintenance-only transaction, before either periodic reader or arms
     * are enabled. No arm, position-goal, mode, calibration or nonzero speed writes. */
    uint8_t before[16][56],after[56],torque_mask=0;
    for(unsigned i=0;i<16;i++){
        uint8_t req[4]={1,i<10?0:1,i<10?i+1:i-9,0};
        DeviceInventory_Read(req,4,true,before[i]);
        bool mobile=i>=6&&i<10;const uint8_t *d=before[i];
        bool wheel=mobile;
        if(d[17]!=req[2]||(enable?!off_snapshot(d,wheel):
           (d[26]>(mobile?1:0)||(wheel?!wheel_snapshot(d,d[26]):
             (d[1]||d[4]!=31||d[36]!=1||d[47]||d[39]||d[40]))))||
           (mobile&&(d[25]!=1||d[32]||d[33])))return false;
        if(wheel&&d[26])torque_mask|=(uint8_t)(1u<<(i-6));
    }
    if(!maintenance_axes_quiet(torque_mask,enable?8:2))return false;
    const uint8_t zero[2]={0,0};bool ok=true;
    for(uint8_t id=ACTUATOR_MOBILE_FIRST_ID;id<=ACTUATOR_LIFT_ID;id++){
        uint8_t mode=0;
        if(Servo_ReadData(id,33,1,&mode)!=HAL_OK||mode!=1||!write_mobile_checked(id,46,zero,2)){ok=false;break;}
    }
    if(ok)for(uint8_t id=ACTUATOR_MOBILE_FIRST_ID;id<=ACTUATOR_LIFT_ID;id++){
        uint8_t value=enable?1:0;
        if(!write_mobile_checked(id,40,&value,1)){ok=false;break;}
    }
    if(ok)for(unsigned i=0;i<16;i++){
        uint8_t req[4]={1,i<10?0:1,i<10?i+1:i-9,0};
        DeviceInventory_Read(req,4,true,after);bool mobile=i>=6&&i<10;
        if(after[1]||after[4]!=31||memcmp(before[i]+12,after+12,14)||after[36]!=1||
           after[26]!=(mobile&&enable?1:0)||
           ((i>=6&&i<10)?!wheel_snapshot(after,enable?1:0):(after[47]||after[39]||after[40]))||
           (mobile&&(after[32]||after[33]))){ok=false;break;}
    }
    if(ok)ok=maintenance_axes_quiet(enable?15:0,enable?2:8);
    if(!ok){
        /* A failed activation is not proof of torque OFF. Keep support and
         * best-effort zero only in verified speed mode, never disable a load. */
        for(uint8_t id=ACTUATOR_MOBILE_FIRST_ID;id<=ACTUATOR_LIFT_ID;id++){
            uint8_t mode=0;
            if(Servo_ReadData(id,33,1,&mode)==HAL_OK&&mode==1)(void)write_mobile_checked(id,46,zero,2);
        }
    }
    return ok;
}
static bool secured_boot_release(void){
    /* Only before registration after an explicit reset. A supported empty
     * mechanism can retain servo torque across MCU reset. Never run this
     * while a runtime writer/reader owns the bus, or enable any torque. */
    uint8_t before[16][56],d[56];int first[16]={0},low[16]={0},high[16]={0};
    uint32_t start=HAL_GetTick();
    for(unsigned pass=0;pass<6;pass++){
        if(pass)HAL_Delay(80);
        for(unsigned i=0;i<16;i++){
            uint8_t q[4]={1,i<10?0:1,i<10?i+1:i-9,0};bool mobile=i>=6&&i<10;
            DeviceInventory_Read(q,4,true,d);
            uint16_t speed=(uint16_t)(d[39]|((uint16_t)d[40]<<8));
            /* Supported boot release only: include the speed-mode lift in
             * the existing six-sample, >=400 ms, <=2-count rest proof.
             * Its retained torque can report +/-50 while holding position.
             * Zero target, full feedback and torque-OFF position proof still
             * apply; this does not authorize lift or base motion. */
            if(d[1]||d[4]!=31||d[17]!=q[2]||d[36]!=1||d[26]>1||
               d[25]!=(mobile?1:0)||
               (mobile?((speed&0x7fff)>50||d[47]>1):(speed||d[47]))||
               (mobile&&(d[32]||d[33])))return false;
            int pos=d[37]|((int)d[38]<<8);if(pos>4095)return false;
            if(!pass){memcpy(before[i],d,56);first[i]=pos;}
            else if(memcmp(before[i]+12,d+12,15))return false;
            int delta=pos-first[i];if(delta>2048)delta-=4096;if(delta< -2048)delta+=4096;
            if(delta<low[i])low[i]=delta;
            if(delta>high[i])high[i]=delta;
            if(high[i]-low[i]>2)return false;
        }
    }
    if(HAL_GetTick()-start<400)return false;
    recovery_attempted=true; /* uncertain writes cannot be repeated this boot */
    const uint8_t zero[2]={0,0};
    for(uint8_t id=ACTUATOR_MOBILE_FIRST_ID;id<=ACTUATOR_LIFT_ID;id++)
        if(!write_mobile_checked(id,46,zero,2))return false;
    for(uint8_t id=ACTUATOR_MOBILE_FIRST_ID;id<=ACTUATOR_LIFT_ID;id++)
        if(!write_mobile_checked(id,40,zero,1))return false;
    if(Servo_DisableTorqueAll()!=HAL_OK||RightServoBus_DisableTorqueAll()!=HAL_OK)return false;
    for(unsigned i=0;i<16;i++){
        uint8_t q[4]={1,i<10?0:1,i<10?i+1:i-9,0};
        DeviceInventory_Read(q,4,true,d);
        if(!off_snapshot(d,i>=6&&i<10)||memcmp(before[i]+12,d+12,14)||
           (i>=6&&i<10&&(d[32]||d[33])))return false;
    }
    return maintenance_axes_quiet(0,8);
}
#endif
bool MobileBoard_Commission(const uint8_t q[36],uint8_t out[64],bool host_idle){
#if HOST_S4_UNIFIED_CANDIDATE
    if(!q||!out||memcmp(q,"AC\x01",3)||q[3]<1||q[3]>5||!read32(q+4)||
       read32(q+32)!=actuator_crc32c(q,32))return false;
    for(unsigned i=16;i<32;i++)if(q[i])return false;
    uint32_t now=HAL_GetTick();bool ok=false;
    if(q[3]==1){
        if(read32(q+8)||read32(q+12))return false;
        commission_ticket=0;
        if(host_idle&&MobileBoard_MaintenanceAllowed()&&
           ((!prepared&&!commission_attempted)||prepared)){
            if(commission_sequence==UINT32_MAX)return false;
            commission_ticket=++commission_sequence;commission_issued=now;
        }
        ok=true;
    }else if(q[3]==5&&host_idle&&MobileBoard_MaintenanceAllowed()&&!prepared&&!commission_attempted&&
             !recovery_attempted&&commission_ticket&&read32(q+8)==commission_ticket&&
             now-commission_issued<2000&&read32(q+12)==UINT32_C(0x53454355)){
        commission_ticket=0;commission_reason=7;
        ok=secured_boot_release();if(ok)commission_reason=0;
    }else if(q[3]==2&&host_idle&&MobileBoard_MaintenanceAllowed()&&!prepared&&!commission_attempted&&
             commission_reason!=7&&
             commission_ticket&&read32(q+8)==commission_ticket&&now-commission_issued<2000&&
             read32(q+12)==UINT32_C(0x53454355)){
        commission_ticket=0;commission_attempted=true;commission_reason=1;
        uint8_t before[16][56],after[56];bool valid=true;
        for(unsigned i=0;i<16&&valid;i++){
            uint8_t req[4]={1,i<10?0:1,i<10?i+1:i-9,0};
            DeviceInventory_Read(req,4,true,before[i]);
            valid=before[i][17]==req[2]&&off_snapshot(before[i],i>=6&&i<10&&before[i][25]==1);
        }
        if(valid)valid=wheels_off_quiet();
        /* A rejected READ-only preflight has made no persistent or motor
         * write. Permit a new explicit ticket, never repeat an uncertain write. */
        if(!valid)commission_attempted=false;
        if(valid){
            commission_reason=2;
            valid=MobileBoard_ProvisionModes(&commissioned_mobile_profile.devices,true);
        }
        if(valid){
            commission_reason=3;
            for(unsigned i=0;i<16&&valid;i++){
                uint8_t req[4]={1,i<10?0:1,i<10?i+1:i-9,0};
                DeviceInventory_Read(req,4,true,after);
                valid=after[1]==0&&after[4]==31&&after[26]==0&&after[36]==1&&
                    !memcmp(before[i]+12,after+12,13); /* ID, baud, limits, offset. */
                if(i<6||i>=10)valid=valid&&before[i][25]==after[25];
                else valid=valid&&after[25]==1;
            }
        }
        if(valid){
            commission_reason=4;
            MobileBoardProfile candidate=commissioned_mobile_profile;
            /* Explicit transaction only. Corrupt/nonempty journals cannot reset. */
            candidate.initialize_erased_boot_storage=true;
            valid=MobileBoard_Prepare(commission_left,commission_right,&candidate,true);
        }
        ok=valid;if(ok)commission_reason=0;
    }else if((q[3]==3||q[3]==4)&&host_idle&&MobileBoard_MaintenanceAllowed()&&prepared&&
             commission_ticket&&read32(q+8)==commission_ticket&&
             now-commission_issued<2000&&read32(q+12)==UINT32_C(0x53454355)&&
             (q[3]==3?!power_attempted:power_attempted)){
        commission_ticket=0;power_attempted=true;commission_reason=q[3]==3?5:6;
        ok=secured_mobile_power(q[3]==3);
        power_enabled=ok&&q[3]==3;
        if(ok){commission_reason=0;if(q[3]==4)power_attempted=false;}
    }
    if(q[3]==5)commission_ticket=0; /* rejected releases also consume their ticket */
    memset(out,0,64);memcpy(out,"CS\x01",3);out[3]=ok?q[3]:(uint8_t)(q[3]|128);
    write32(out+4,read32(q+4));write32(out+8,HAL_GetTick());
    write32(out+12,prepared?mobile_endpoint.boot_id:0);
    write32(out+16,commission_ticket);write32(out+20,commission_issued);
    out[24]=(prepared?1:0)|(runtime_started?2:0)|(output_ready?4:0)|
        (stop_requested?8:0)|(commission_attempted?16:0)|(power_enabled?32:0)|(power_attempted?64:0);
    out[25]=commission_reason;
    write32(out+28,actuator_crc32c((const uint8_t*)&commissioned_mobile_profile,sizeof(commissioned_mobile_profile)));
    write32(out+60,actuator_crc32c(out,60));return true;
#else
    (void)q;(void)out;(void)host_idle;return false;
#endif
}
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
    if(supervisor.transport_only){
        out[26]=1; /* explicit wheel-only STOP scope; no arm/lift proof claimed */
        out[24]=(system_stop.actions_completed&&last_stop_proof.base_stopped&&
            age<profile.stop.feedback_max_age_ms)?SYSTEM_STOP_CONFIRMED:SYSTEM_STOP_UNCONFIRMED;
    }
    write32(out+28,last_stop_proof.observed_ms);write32(out+32,age);
    write32(out+36,profile.stop.feedback_max_age_ms);
    write32(out+60,actuator_crc32c(out,60));return true;
}
