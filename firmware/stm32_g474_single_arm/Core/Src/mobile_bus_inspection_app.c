/* Temporary maintenance image, not the runtime motion firmware.
 * No binary/legacy arm command handler. Maintenance variants have distinct
 * commands and independent servo_transport register/value allowlists.
 */
#include "single_arm_app.h"
#include "servo_bus.h"
#include "actuator_core/mobile_ids.h"
#if HOST_MOBILE_WHEEL_PULSE_ONLY
#include "mobile_wheel_pulse.h"
#define INSPECTION_NAME "mobile-wheel-pulse-v4"
#elif HOST_MOBILE_WHEEL_SETUP_ONLY
#include "mobile_wheel_setup.h"
#define INSPECTION_NAME "mobile-wheel-setup-v1"
#else
#define INSPECTION_NAME "mobile-bus-inspection-v2"
#endif
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *host;
static char command[16];
static unsigned used;
static int discard;

static void send_line(const char *line) {
    (void)HAL_UART_Transmit(host, (const uint8_t *)line, (uint16_t)strlen(line), 100);
}
static void info(void) {
#if HOST_MOBILE_WHEEL_PULSE_ONLY
    char line[320];
    (void)snprintf(line,sizeof(line),
        "{\"firmware\":\"%s\",\"host_baud\":921600,\"servo_baud\":1000000,"
        "\"motion_enabled\":false,\"pulse_supported\":true,\"pulse_consumed\":%s,"
        "\"base_pulse_supported\":true,\"speed_raw\":200,\"duration_ms\":3000,\"ids\":[%u,%u,%u,%u]}\r\n",
        INSPECTION_NAME,MobileWheelPulse_Consumed()?"true":"false",
        (unsigned)ACTUATOR_WHEEL_0_ID,(unsigned)ACTUATOR_WHEEL_1_ID,
        (unsigned)ACTUATOR_WHEEL_2_ID,(unsigned)ACTUATOR_LIFT_ID);
#else
    char line[256];
    (void)snprintf(line, sizeof(line),
        "{\"firmware\":\"%s\",\"host_baud\":921600,"
        "\"servo_baud\":1000000,\"reference_id\":1,\"motion_enabled\":false,\"ids\":[%u,%u,%u,%u]}\r\n",
        INSPECTION_NAME, (unsigned)ACTUATOR_WHEEL_0_ID, (unsigned)ACTUATOR_WHEEL_1_ID,
        (unsigned)ACTUATOR_WHEEL_2_ID, (unsigned)ACTUATOR_LIFT_ID);
#endif
    send_line(line);
}
static int read_byte(uint8_t id, uint8_t address) {
    uint8_t value;
    return Servo_ReadData(id, address, 1, &value) == HAL_OK ? value : -1;
}
static const char *failure_name(ServoBusFailureReason reason) {
    switch (reason) {
    case SERVO_BUS_FAILURE_NONE: return "NONE";
    case SERVO_BUS_FAILURE_TX: return "TX";
    case SERVO_BUS_FAILURE_RX_TIMEOUT: return "RX_TIMEOUT";
    case SERVO_BUS_FAILURE_UART: return "UART";
    case SERVO_BUS_FAILURE_HEADER: return "HEADER";
    case SERVO_BUS_FAILURE_ID: return "ID";
    case SERVO_BUS_FAILURE_LENGTH: return "LENGTH";
    case SERVO_BUS_FAILURE_STATUS: return "SERVO_STATUS";
    case SERVO_BUS_FAILURE_CHECKSUM: return "CHECKSUM";
    case SERVO_BUS_FAILURE_RECOVERY: return "RECOVERY";
    case SERVO_BUS_FAILURE_RX_OVERFLOW: return "RX_OVERFLOW";
    case SERVO_BUS_FAILURE_DMA: return "DMA";
    default: return "UNKNOWN";
    }
}
static void report_motor(uint8_t id, int identity_only) {
    uint8_t identity[4] = {0}, position[2] = {0};
    uint32_t before = ServoBus_GetHealth()->transaction_count;
    HAL_StatusTypeDef status = Servo_ReadData(id, 3, 4, identity);
    int started = ServoBus_GetHealth()->transaction_count != before;
    /* Copy now: subsequent register reads replace the shared diagnostic state. */
    ServoBusDiagnostics diag = {0};
    if (started) diag = *ServoBus_GetDiagnostics();
    int ok = status == HAL_OK;
    int mode = -1, torque = -1, raw = -1;
    if (ok && !identity_only) {
        mode = read_byte(id, 33);
        torque = read_byte(id, 40);
        if (Servo_ReadData(id, 56, 2, position) == HAL_OK)
            raw = position[0] | (position[1] << 8);
    }
    char line[640];
    (void)snprintf(line, sizeof(line),
        "{\"id\":%u,\"identity_ok\":%s,\"model_raw\":%d,\"reported_id\":%d,"
        "\"baud_code\":%d,\"mode_raw\":%d,\"torque_raw\":%d,\"position_raw\":%d,"
        "\"identity_hal_status\":%u,\"identity_transaction_started\":%s,"
        "\"identity_failure\":\"%s\",\"identity_rx_bytes\":%u,"
        "\"identity_uart_error\":%lu,\"identity_uart_isr\":%lu,"
        "\"identity_dma_error\":%lu,\"identity_servo_status\":%u}\r\n",
        (unsigned)id, ok ? "true" : "false",
        ok ? identity[0] | (identity[1] << 8) : -1,
        ok ? identity[2] : -1, ok ? identity[3] : -1, mode, torque, raw,
        (unsigned)status, started ? "true" : "false",
        ok ? "NONE" : started ? failure_name(diag.reason) : "SERVICE_NOT_STARTED",
        (unsigned)diag.received_bytes, (unsigned long)diag.uart_error_code,
        (unsigned long)diag.uart_isr, (unsigned long)diag.dma_error_code,
        (unsigned)diag.servo_status);
    send_line(line);
}
static void scan(void) {
    send_line("{\"scan\":\"begin\"}\r\n");
    /* Known left-arm ID: identity READ only, to compare the common bus path. */
    report_motor(1, 1);
    for (unsigned axis = 0; axis < 4; ++axis)
        report_motor(actuator_mobile_axis_id(axis), 0);
    send_line("{\"scan\":\"end\",\"motor_write_commands\":0}\r\n");
}
#if HOST_MOBILE_WHEEL_SETUP_ONLY
static void prepare_wheels(void) {
    MobileWheelSetupResult wheels[3];
    bool ok=MobileWheelSetup_Run(wheels);
    char line[768];
    int offset=snprintf(line,sizeof(line),"{\"setup_ok\":%s,\"motion_enabled\":false,\"wheels\":[",ok?"true":"false");
    for(unsigned i=0;i<3;i++) {
        MobileWheelSetupResult *x=&wheels[i];
        offset+=snprintf(line+offset,sizeof(line)-(unsigned)offset,
            "%s{\"id\":%u,\"ok\":%s,\"stage\":\"%s\",\"mode\":%d,\"torque\":%d,\"lock\":%d}",
            i?",":"",(unsigned)x->id,x->ok?"true":"false",x->stage,x->mode,x->torque,x->lock);
    }
    (void)snprintf(line+offset,sizeof(line)-(unsigned)offset,"]}\r\n");
    send_line(line);
}
#endif
#if HOST_MOBILE_WHEEL_PULSE_ONLY
static void pulse_base(char motion) {
    MobileBasePulseResult x;
    MobileWheelPulse_RunBase(motion,&x);
    char line[384];
    (void)snprintf(line,sizeof(line),
        "{\"base_result\":\"begin\",\"motion\":\"%c\",\"pulse_ok\":%s,"
        "\"stage\":\"%s\",\"failed_id\":%u,\"stop_confirmed\":%s,"
        "\"cleanup_attempted\":%s,\"stop_requested_ms\":%lu}\r\n",
        motion,x.ok?"true":"false",x.stage,(unsigned)x.failed_id,
        x.stop_confirmed?"true":"false",x.cleanup_attempted?"true":"false",
        (unsigned long)x.stop_requested_ms);
    send_line(line);
    for (unsigned i=0;i<3;++i) {
        MobileWheelPulseResult *w=&x.wheels[i];
        (void)snprintf(line,sizeof(line),
            "{\"id\":%u,\"command_raw\":%d,\"position_before\":%d,\"position_after\":%d,"
            "\"goal_after\":%d,\"torque_after\":%d,\"speed_after\":%d,\"stop_confirmed\":%s,"
            "\"speed_check_raw\":%d,\"speed_check_hal_status\":%d}\r\n",
            (unsigned)w->id,w->speed_raw,w->position_before,w->position_after,w->goal_after,
            w->torque_after,w->speed_after,w->stop_confirmed?"true":"false",
            w->speed_check_raw,w->speed_check_hal_status);
        send_line(line);
    }
    send_line("{\"base_result\":\"end\"}\r\n");
}
static void pulse_wheel(uint8_t id, int direction) {
    MobileWheelPulseResult x;
    MobileWheelPulse_Run(id,direction,&x);
    char line[768];
    (void)snprintf(line,sizeof(line),
        "{\"pulse_ok\":%s,\"id\":%u,\"failed_id\":%u,\"stage\":\"%s\","
        "\"motion_attempted\":%s,\"cleanup_attempted\":%s,\"stop_confirmed\":%s,"
        "\"speed_check_raw\":%d,\"speed_check_hal_status\":%d,"
        "\"speed_raw\":%d,\"stop_requested_ms\":%lu,\"position_before\":%d,\"position_after\":%d,"
        "\"goal_after\":%d,\"torque_after\":%d,\"speed_after\":%d}\r\n",
        x.ok?"true":"false",(unsigned)x.id,(unsigned)x.failed_id,x.stage,
        x.motion_attempted?"true":"false",x.cleanup_attempted?"true":"false",x.stop_confirmed?"true":"false",
        x.speed_check_raw,x.speed_check_hal_status,
        x.speed_raw,(unsigned long)x.stop_requested_ms,x.position_before,x.position_after,
        x.goal_after,x.torque_after,x.speed_after);
    send_line(line);
}
#endif
void SingleArmApp_Init(UART_HandleTypeDef *host_uart, UART_HandleTypeDef *servo_uart) {
    host = host_uart;
    used = 0; discard = 0;
    ServoBus_Init(servo_uart, NULL, NULL);
    /* Boot only initializes UART/DMA bookkeeping. No servo transactions. */
    info();
}
void SingleArmApp_Process(void) {
    uint8_t byte;
    HAL_StatusTypeDef status = HAL_UART_Receive(host, &byte, 1, 10);
    if (status == HAL_TIMEOUT) return;
    if (status != HAL_OK) { used = 0; discard = 1; return; }
    if (byte == '\r') return;
    if (byte == '\n') {
        command[used] = 0;
        if (!discard && !strcmp(command, "INFO")) info();
        else if (!discard && !strcmp(command, "SCAN")) scan();
#if HOST_MOBILE_WHEEL_SETUP_ONLY
        else if (!discard && !strcmp(command, "PREPARE_WHEELS")) prepare_wheels();
#endif
#if HOST_MOBILE_WHEEL_PULSE_ONLY
        else if (!discard && used==6 && !memcmp(command,"BASE ",5) &&
                 strchr("FBLRAD",command[5])) pulse_base(command[5]);
        else if (!discard && used==9 && !memcmp(command,"PULSE ",6) &&
                 command[6]>='7' && command[6]<='9' && command[7]==' ' &&
                 (command[8]=='+' || command[8]=='-'))
            pulse_wheel((uint8_t)(command[6]-'0'),command[8]=='+'?1:-1);
#endif
        else send_line("{\"error\":\"unsupported maintenance command\"}\r\n");
        used = 0; discard = 0;
    } else if (byte < 32 || byte > 126 || used + 1 >= sizeof(command)) {
        used = 0; discard = 1;
    } else if (!discard) command[used++] = (char)byte;
}
