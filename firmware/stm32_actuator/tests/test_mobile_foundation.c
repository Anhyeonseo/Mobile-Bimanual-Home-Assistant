#include "actuator_core/shared_bus.h"
#include "actuator_core/mobile_supervisor.h"
#include "actuator_core/motor_groups.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); return 1; } } while (0)

static int bus_tests(void) {
    actuator_shared_bus_t b;
    uint32_t token = 123u, next = 0u;
    actuator_shared_bus_init(&b);
    CHECK(!actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_FEEDBACK, 0, 4000, 3000, &token));
    CHECK(token == 123u && !b.active);
    CHECK(actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_ARM, 0, 100, 100, &token));
    actuator_shared_bus_request_stop(&b);
    CHECK(!actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_STOP, 1, 100, 0, &next));
    CHECK(!actuator_shared_bus_complete(&b, token + 1, 50, true));
    CHECK(b.active);
    CHECK(actuator_shared_bus_complete(&b, token, 99, true));
    CHECK(!actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_LIFT, 100, 100, 1000, &next));
    CHECK(actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_STOP, 100, 100, 0, &next));
    CHECK(actuator_shared_bus_complete(&b, next, 199, true));
    CHECK(!b.stop_pending);
    CHECK(actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_WHEELS, UINT32_MAX - 20, 30, 100, &token));
    actuator_shared_bus_poll(&b, 9); /* exactly 30 us across timer wrap */
    CHECK(b.faulted && b.active && b.stop_pending);
    CHECK(!actuator_shared_bus_complete(&b, token, 9, true));
    CHECK(!actuator_shared_bus_recover(&b, false));
    CHECK(actuator_shared_bus_recover(&b, true));
    CHECK(actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_STOP, 10, 100, 0, &next));
    CHECK(!actuator_shared_bus_complete(&b, token, 11, true));
    CHECK(!actuator_shared_bus_complete(&b, next, 11, false));
    CHECK(b.faulted && b.active);
    CHECK(actuator_shared_bus_recover(&b, true));
    CHECK(!actuator_shared_bus_acquire(&b, (actuator_bus_work_t)-1, 12, 1, 1, &next));
    CHECK(!actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_STOP, 12, 0, 1, &next));
    CHECK(!actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_STOP, 12, UINT32_MAX, 1, &next));
    b.token = UINT32_MAX;
    CHECK(!actuator_shared_bus_acquire(&b, ACTUATOR_BUS_WORK_STOP, 12, 1, 1, &next));
    CHECK(b.faulted);
    return 0;
}
static int mobile_tests(void) {
    actuator_mobile_supervisor_t s;
    actuator_mobile_config_t c = {{100,100,100,50}, 2, 100, 200, 0, 100000};
    actuator_mobile_feedback_t f = {0, {0,0,0,0}, 50000, true, true, true};
    int32_t goal[4] = {10,-10,10,20};
    uint8_t packet[26];
    size_t length = 0;
    CHECK(actuator_mobile_init(&s, &c));
    CHECK(!actuator_mobile_arm(&s, 1, 0));
    CHECK(actuator_mobile_feedback(&s, &f, 0));
    CHECK(!actuator_mobile_feedback(&s, &f, 0));
    CHECK(actuator_mobile_arm(&s, 1, 0));
    CHECK(!actuator_mobile_arm(&s, 2, 0));
    CHECK(!actuator_mobile_command(&s, 2, 0, goal, 1));
    CHECK(actuator_mobile_command(&s, 1, 0, goal, 1));
    CHECK(!actuator_mobile_command(&s, 1, 0, goal, 99));
    CHECK(s.command_ms == 1);
    goal[0] = 101;
    CHECK(!actuator_mobile_command(&s, 1, 1, goal, 99));
    CHECK(s.sequence == 0 && s.target_raw[0] == 10);
    goal[0] = 10;
    actuator_mobile_poll(&s, 101);
    CHECK(s.state == ACTUATOR_MOBILE_STOP_LATCHED);
    CHECK(s.reason == ACTUATOR_MOBILE_REASON_COMMAND_TIMEOUT);
    CHECK(s.target_raw[0] == 0 && s.target_raw[3] == 0);
    CHECK(actuator_motor_group_velocities(ACTUATOR_GROUP_BASE_WHEELS, s.target_raw, 3, packet, &length) == ACTUATOR_GROUP_OK);
    CHECK(length == 17 && packet[5] == 46 && packet[7] == 8);
    CHECK(!actuator_mobile_command(&s, 1, 1, goal, 101));
    f.observed_ms = 101; f.velocity_raw[0] = 10;
    CHECK(actuator_mobile_feedback(&s, &f, 101));
    CHECK(!actuator_mobile_measured_stopped(&s, 101));
    CHECK(!actuator_mobile_arm(&s, 2, 101));
    f.observed_ms = 102; f.velocity_raw[0] = 0;
    CHECK(actuator_mobile_feedback(&s, &f, 102));
    CHECK(!actuator_mobile_arm(&s, 1, 102));
    CHECK(actuator_mobile_arm(&s, 2, 102));
    f.observed_ms = 103; f.lift_position_um = 100000;
    CHECK(actuator_mobile_feedback(&s, &f, 103));
    CHECK(!actuator_mobile_command(&s, 2, 1, goal, 103));
    goal[3] = -20;
    CHECK(actuator_mobile_command(&s, 2, 1, goal, 103));
    f.observed_ms = 104; f.lift_position_um = 0;
    CHECK(actuator_mobile_feedback(&s, &f, 104));
    CHECK(s.reason == ACTUATOR_MOBILE_REASON_LIFT_LIMIT && s.target_raw[3] == 0);
    CHECK(actuator_mobile_arm(&s, 3, 104));
    f.observed_ms = 105; f.velocity_modes_verified = false;
    CHECK(actuator_mobile_feedback(&s, &f, 105));
    CHECK(s.reason == ACTUATOR_MOBILE_REASON_FEEDBACK);
    CHECK(!actuator_mobile_arm(&s, 4, 105));
    f.observed_ms = 106; f.velocity_modes_verified = true; f.lift_homed = false;
    CHECK(actuator_mobile_feedback(&s, &f, 106));
    CHECK(!actuator_mobile_arm(&s, 4, 106));
    f.observed_ms = 107; f.lift_homed = true;
    CHECK(actuator_mobile_feedback(&s, &f, 107));
    CHECK(actuator_mobile_arm(&s, 4, 107));
    f.observed_ms = 108;
    CHECK(!actuator_mobile_feedback(&s, &f, 107)); /* future evidence */
    actuator_mobile_poll(&s, 307);
    CHECK(s.state == ACTUATOR_MOBILE_STOP_LATCHED);
    CHECK(!actuator_mobile_measured_stopped(&s, 307));
    /* A late command cannot refresh a watchdog that already expired. */
    CHECK(actuator_mobile_init(&s, &c));
    f.observed_ms = UINT32_MAX - 50; f.lift_position_um = 50000;
    CHECK(actuator_mobile_feedback(&s, &f, f.observed_ms));
    CHECK(actuator_mobile_arm(&s, 1, f.observed_ms));
    CHECK(actuator_mobile_command(&s, 1, 1, goal, f.observed_ms));
    CHECK(!actuator_mobile_command(&s, 1, 2, goal, 49));
    CHECK(s.reason == ACTUATOR_MOBILE_REASON_COMMAND_TIMEOUT);
    /* A fresh packet after a feedback gap must not silently restore motion. */
    CHECK(actuator_mobile_init(&s, &c));
    f.observed_ms = 0;
    CHECK(actuator_mobile_feedback(&s, &f, 0));
    CHECK(actuator_mobile_arm(&s, 1, 0));
    f.observed_ms = 200;
    CHECK(actuator_mobile_feedback(&s, &f, 200));
    CHECK(s.state == ACTUATOR_MOBILE_STOP_LATCHED);
    CHECK(actuator_mobile_arm(&s, 2, 200));
    actuator_mobile_stop(&s);
    CHECK(s.reason == ACTUATOR_MOBILE_REASON_REQUEST);
    c.max_velocity_raw[0] = 32768;
    CHECK(!actuator_mobile_init(&s, &c));
    CHECK(!s.configured && s.state == ACTUATOR_MOBILE_DISABLED);
    CHECK(!actuator_mobile_command(&s, 0, 0, goal, 0));
    return 0;
}
int main(void) { CHECK(bus_tests() == 0); CHECK(mobile_tests() == 0); return 0; }
