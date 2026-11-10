/* Isolated left jaw commissioning. No motor transaction at boot. Each sequenced
 * STEP permits at most one small closing setpoint. Loss of the host cuts torque. */
#include "single_arm_app.h"
#include "servo_bus.h"
#include "gripper_bench_limits.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *host;
static bool active, bound, off_verified, locked, sample_ok, discard;
static bool watching_load, preloading;
static uint32_t load_since;
static uint32_t ticket, sequence, bound_at, began, heartbeat, last_step, progress_at, writes;
static uint32_t hold_duration, hold_at, hold_elapsed;
static int hold_goal, hold_position;
static const char *hold_cause;
static int lower_bound, upper_bound;
static int position, speed, load, current, motor_status, goal, anchor, last_progress, cap, torque;
static int failed_reg, expected_value, returned_value, config[4], seed_torque;
static const char *reason;
static char command[52];
static unsigned used;

static int word(const uint8_t *b) { return b[0] | b[1]<<8; }

static int reg(uint8_t address, uint8_t n) {
    uint8_t data[2] = {0};
    return Servo_ReadData(GRIP_ID, address, n, data) == HAL_OK ? word(data) : -1;
}
static bool write_verified(uint8_t address, uint8_t *data, uint8_t n) {
    uint8_t readback[8];
    ++writes;
    if (Servo_WriteData(GRIP_ID,address,data,n)!=HAL_OK ||
        Servo_ReadData(GRIP_ID,address,n,readback)!=HAL_OK) {
        failed_reg=address; expected_value=(n==1 ? data[0] : word(data)); returned_value=-1; return false;
    }
    if (memcmp(data,readback,n)) {
        failed_reg=address; expected_value=(n==1 ? data[0] : word(data));
        returned_value=(n==1 ? readback[0] : word(readback)); return false;
    }
    return true;
}
static bool write_word(uint8_t address, int value) {
    uint8_t b[2] = {(uint8_t)value, (uint8_t)(value>>8)};
    return write_verified(address,b,2);
}
static bool read_runtime(void) {
    uint8_t b[8];
    if (Servo_ReadData(GRIP_ID,42,8,b)!=HAL_OK) return false;
    for (unsigned i=0; i<4; ++i) config[i]=word(b+2*i);
    return true;
}
static bool sample(void) {
    uint8_t b[15];
    sample_ok = false;
    if (Servo_ReadData(GRIP_ID, 56, sizeof(b), b) != HAL_OK) return false;
    position = word(b); speed = word(b+2); load = word(b+4);
    current = word(b+13); motor_status = b[9];
    torque = reg(40, 1);
    sample_ok = position <= 4095 && load <= 2047 && (torque == 0 || torque == 1);
    return sample_ok;
}
static bool identified(void) {
    uint8_t b[4], calibration[3];
    return Servo_ReadData(GRIP_ID, 3, 4, b) == HAL_OK &&
        b[0] == 9 && b[1] == 3 && b[2] == GRIP_ID && b[3] == 0 &&
        Servo_ReadData(GRIP_ID, 31, 3, calibration) == HAL_OK &&
        word(calibration) == 0x0253 && calibration[2] == 0;
}
static void finish(const char *why) {
    /* Even a write reporting an error may have arrived. Confirm OFF independently. */
    uint8_t off = 0;
    bound = active = false;
    off_verified = false;
    for (unsigned i = 0; i < 2 && !off_verified; ++i) {
        ++writes;
        (void)Servo_WriteData(GRIP_ID, 40, &off, 1);
        torque = reg(40,1);
        off_verified = torque == 0;
    }
    locked = !off_verified;
    reason = why;
}
static void reply(void) {
    char out[900];
    int n = snprintf(out, sizeof(out),
        "{\"firmware\":\"left-grip-bench-v5\",\"ticket\":%lu,\"sequence\":%lu,\"bound\":%d,\"active\":%d,"
        "\"reason\":\"%s\",\"off_verified\":%d,\"locked\":%d,\"sample_valid\":%d,"
        "\"position\":%d,\"speed_raw\":%d,\"load_raw\":%d,\"current_raw\":%d,\"status_raw\":%d,"
        "\"torque\":%d,\"goal\":%d,\"torque_limit_raw\":%d,\"writes\":%lu,\"tick_ms\":%lu,\"config\":[%d,%d,%d,%d],\"failed_reg\":%d,\"expected\":%d,\"returned\":%d,\"seed_torque\":%d,\"floor_raw\":%d,\"entry_raw\":%d,"
        "\"hold_ms\":%lu,\"hold_elapsed_ms\":%lu,\"hold_goal\":%d,\"hold_cause\":\"%s\"}\r\n",
        (unsigned long)ticket,(unsigned long)sequence,bound,active,reason,off_verified,locked,sample_ok,
        position,speed,load,current,motor_status,torque,goal,cap,(unsigned long)writes,(unsigned long)HAL_GetTick(),config[0],config[1],config[2],config[3],failed_reg,expected_value,returned_value,seed_torque,lower_bound,upper_bound,
        (unsigned long)hold_duration,(unsigned long)hold_elapsed,hold_goal,hold_cause);
    if (n>0 && (unsigned)n<sizeof(out)) (void)HAL_UART_Transmit(host,(uint8_t*)out,(uint16_t)n,20);
}
static void bind(uint32_t nonce) {
    bound = off_verified = sample_ok = false;
    failed_reg=expected_value=returned_value=seed_torque=-1;
    ticket = nonce; sequence = 0;
    if (!identified() || !sample() || !read_runtime()) { reason="READ_OR_IDENTITY_FAILED"; return; }
    off_verified = torque == 0;
    if (!off_verified || motor_status || (speed & 32767)>3 ||
        position < GRIP_LOW+GRIP_STEP || position > GRIP_HIGH) { reason="START_POSE_OR_TORQUE"; return; }
    anchor = position; goal = anchor;
    bound = true; bound_at = HAL_GetTick(); reason="READY";
}
static void begin(uint32_t nonce, unsigned long floor, unsigned long entry, unsigned long hold_ms) {
    bool fresh = bound && ticket == nonce && HAL_GetTick()-bound_at < 3000;
    bound = false;
    if (!fresh) { reason="STALE_BIND"; return; }
    if (hold_ms && (hold_ms < GRIP_HOLD_MIN_MS || hold_ms > GRIP_HOLD_MAX_MS)) {
        reason="HOLD_DURATION_REJECTED"; return;
    }
    if (floor < GRIP_LOW || floor > GRIP_HIGH-GRIP_STEP || entry > GRIP_HIGH || entry < floor + GRIP_STEP ||
        anchor < (int)floor + GRIP_STEP || anchor > (int)entry) { reason="PLAN_BOUNDS_REJECTED"; return; }
    lower_bound=(int)floor; upper_bound=(int)entry;
    hold_duration=(uint32_t)hold_ms; hold_at=hold_elapsed=0; hold_goal=-1; hold_cause="NONE"; preloading=false;
    if (!identified() || !sample() || torque != 0 || motor_status || (speed & 32767)>3 ||
        abs(position-anchor)>3 || position < lower_bound + GRIP_STEP || position > upper_bound) { reason="POSE_CHANGED"; return; }
    int prior = reg(48,2);
    if (prior < 1 || prior > 1000) { reason="INVALID_TORQUE_LIMIT"; return; }
    cap = prior < GRIP_CAP ? prior : GRIP_CAP;
    goal = anchor = last_progress = position;
    /* Program and verify the output cap and speed before writing a position, including
     * the stationary seed. Some servos energize on receipt of a position command. */
    uint8_t enable = 1;
    off_verified = false;
    if (!write_word(48,cap) || !write_word(46,GRIP_SPEED) ||
        !write_word(44,0) || !read_runtime() || config[1]!=0 ||
        config[2]!=GRIP_SPEED || config[3]!=cap) { finish("CAP_OR_SPEED_READBACK"); return; }
    if (!sample() || torque!=0 || motor_status || abs(position-anchor)>3 || position < lower_bound || position > upper_bound || (speed&32767)>3) {
        finish("PRESEED_STATE_CHANGED"); return;
    }
    if (!write_word(42,anchor) || !read_runtime() || config[0]!=anchor || config[1]!=0 ||
        config[2]!=GRIP_SPEED || config[3]!=cap) { finish("SEED_READBACK"); return; }
    if (!sample() || motor_status || abs(position-anchor)>3 || position < lower_bound || position > upper_bound || (speed&32767)>3) {
        finish("SEED_FEEDBACK"); return;
    }
    seed_torque = torque;
    if (!torque && !write_verified(40,&enable,1)) { finish("ENABLE_READBACK"); return; }
    watching_load = false;
    active = true; torque = 1;
    heartbeat = began = last_step = progress_at = HAL_GetTick();
    reason="ANCHORED";
}
static bool runtime_intact(void) {
    return read_runtime() && config[0]==goal && config[1]==0 && config[2]==GRIP_SPEED && config[3]==cap;
}
static bool checked_feedback(void) {
    if (!sample() || torque!=1 || motor_status || !runtime_intact() || !identified()) {
        finish("FEEDBACK_OR_CONFIG"); return false;
    }
    if (position < lower_bound || position > upper_bound || position > anchor+4 ||
        (!preloading && hold_goal<0 && abs(position-goal)>16) ||
        (speed & 32767)>150) { finish("TRACKING_LIMIT"); return false; }
    if ((current & 32767) >= 30 || (load & 1023) > cap) { finish("EFFORT_BOUND"); return false; }
    if (hold_goal >= 0 && (goal != hold_goal || abs(position-hold_position)>16)) {
        finish("HOLD_POSITION_CHANGED"); return false;
    }
    return true;
}
static void observe_resistance(const char *why, uint32_t now) {
    (void)now;
    if (!hold_duration) { finish(why); return; }
    /* A rigid object may stop the encoder before the preselected target.
     * Continue only the bounded setpoints under the verified output cap. */
    preloading=true; hold_cause=why; reason="PRELOADING_TARGET";
}
static bool close_increment(uint32_t now) {
    int next=goal-GRIP_STEP;
    if (next<lower_bound) next=lower_bound;
    if (next!=goal && !write_word(42,next)) { finish("GOAL_READBACK"); return false; }
    goal=next; last_step=now;
    return true;
}
static void step(uint32_t nonce, uint32_t seq) {
    if (!active) {
        /* A timed completion can precede the host's next STEP. Acknowledge that
         * sequence once without energizing, preserving its terminal sample. */
        if (!strcmp(reason,"HOLD_COMPLETE") && nonce==ticket && seq==sequence+1) ++sequence;
        return;
    }
    if (nonce != ticket || seq != sequence+1) { finish("SEQUENCE_MISMATCH"); return; }
    ++sequence;
    uint32_t now = HAL_GetTick();
    if (!checked_feedback()) return;
    /* Last energized sample is preserved in the final reply for comparison.
     * These caps stop the trial; they never assert that an object was grasped. */
    heartbeat = now;
    if (hold_goal >= 0) {
        hold_elapsed=now-hold_at;
        if (hold_elapsed >= hold_duration) { finish("HOLD_COMPLETE"); return; }
        reason="HOLDING_TARGET"; return;
    }
    if (preloading) {
        if (now-last_step>=75 && close_increment(now) && goal==lower_bound) {
            preloading=false; hold_goal=goal; hold_position=position;
            hold_at=now; hold_elapsed=0; reason="HOLDING_TARGET";
        }
        return;
    }
    if ((load & 1023) >= 55) {
        if (!watching_load) { load_since = now; watching_load = true; }
        if (now-load_since >= 240) { observe_resistance("EFFORT_PERSISTED",now); return; }
        reason = "OBSERVING_SAME_GOAL";
        return;
    }
    watching_load = false;
    if (last_progress-position >= 3) { last_progress=position; progress_at=now; }
    if (goal==lower_bound && abs(goal-position)<=3) { finish("LOWER_GUARD"); return; }
    if (now-progress_at >= 450) { observe_resistance("NO_PROGRESS",now); return; }
    if (now-last_step < 75 || abs(position-goal)>3) { reason="SAMPLING"; return; }
    if (close_increment(now)) reason="CLOSING";
}
static void watchdog(void) {
    uint32_t now=HAL_GetTick();
    if (active && now-heartbeat >= GRIP_LEASE_MS) finish("HOST_LEASE");
    else if (active && now-began >= GRIP_TRIAL_MS) finish("TRIAL_DEADLINE");
    else if (active && hold_goal >= 0 && now-hold_at >= hold_duration) {
        hold_elapsed=now-hold_at;
        if (checked_feedback()) finish("HOLD_COMPLETE");
    }
}
static void dispatch(void) {
    unsigned long nonce=0, seq=0, floor=0, entry=0, hold_ms=0;
    int end=0;
    if (!strcmp(command,"INFO")) { reply(); return; }
    if (!strcmp(command,"STOP")) {
        if (active || locked) finish("OPERATOR_STOP");
        else bound=false;
    } else if (!active && !locked && !strncmp(command,"BIND ",5) &&
        command[5]>='1' && command[5]<='9' && sscanf(command+5,"%lu%n",&nonce,&end)==1 &&
        !command[5+end] && nonce<=UINT32_MAX) {
        bind((uint32_t)nonce);
    } else if (!active && !locked && !strncmp(command,"BEGIN ",6) &&
        command[6]>='1' && command[6]<='9' && sscanf(command+6,"%lu %lu %lu%n",&nonce,&floor,&entry,&end)==3 &&
        !command[6+end] && nonce<=UINT32_MAX) {
        begin((uint32_t)nonce,floor,entry,0);
    } else if (!active && !locked && !strncmp(command,"BEGIN_RIGID ",12) &&
        command[12]>='1' && command[12]<='9' &&
        sscanf(command+12,"%lu %lu %lu %lu%n",&nonce,&floor,&entry,&hold_ms,&end)==4 &&
        !command[12+end] && nonce<=UINT32_MAX) {
        if (!hold_ms) { bound=false; reason="HOLD_DURATION_REJECTED"; }
        else begin((uint32_t)nonce,floor,entry,hold_ms);
    } else if (!strncmp(command,"STEP ",5) && command[5]>='1' && command[5]<='9' &&
        sscanf(command+5,"%lu %lu%n",&nonce,&seq,&end)==2 && !command[5+end] &&
        nonce<=UINT32_MAX && seq<=UINT32_MAX) {
        step((uint32_t)nonce,(uint32_t)seq);
    } else if (active) finish("INVALID_COMMAND");
    else if (!locked) { reason="INVALID_COMMAND"; bound=false; }
    reply();
}
void SingleArmApp_Init(UART_HandleTypeDef *h, UART_HandleTypeDef *servo) {
    host=h; active=bound=locked=off_verified=sample_ok=discard=false;
    used=ticket=sequence=writes=0; goal=cap=torque=lower_bound=upper_bound=-1; reason="IDLE";
    hold_duration=hold_at=hold_elapsed=0; hold_goal=-1; hold_cause="NONE";
    preloading=false;
    ServoBus_Init(servo,NULL,NULL);
}
void SingleArmApp_Process(void) {
    uint8_t c;
    watchdog();
    HAL_StatusTypeDef status=HAL_UART_Receive(host,&c,1,2);
    watchdog();
    if (status==HAL_TIMEOUT) return;
    if (status!=HAL_OK) { if(active) finish("HOST_RX_ERROR"); discard=true; used=0; return; }
    if (c=='\r') return;
    if (c=='\n') {
        command[used]=0;
        if (!discard) dispatch();
        else { if(active) finish("INVALID_FRAME"); else bound=false; reply(); }
        discard=false; used=0;
    } else if (c<32 || c>126 || used+1>=sizeof(command)) { discard=true; used=0; }
    else if (!discard) command[used++]=(char)c;
}
