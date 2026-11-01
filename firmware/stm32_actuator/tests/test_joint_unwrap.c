#include "actuator_core/joint_unwrap.h"

#include <stdint.h>
#include <stdio.h>

static int failures = 0;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", \
                __FILE__, __LINE__, #condition); \
        failures++; \
    } \
} while (0)

static void test_tracks_positive_and_negative_wraps(void) {
    actuator_joint_unwrapper_t state;
    int32_t value = 0;

    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 3919u, 3919, 512u) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(actuator_joint_unwrapper_update(&state, 4059u, &value) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(value == 4059);
    CHECK(actuator_joint_unwrapper_update(&state, 65u, &value) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(value == 4161);
    CHECK(actuator_joint_unwrapper_update(&state, 4078u, &value) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(value == 4078);
}

static void test_binding_requires_a_close_verified_reference(void) {
    actuator_joint_unwrapper_t state;

    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 24u, 4120, 256u) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(state.unwrapped_raw == 4120);

    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 24u, 2048, 1024u) ==
          ACTUATOR_UNWRAP_REFERENCE_TOO_FAR);
    CHECK(!state.bound);
    CHECK(actuator_joint_unwrapper_bind(&state, 0u, 2048, 2047u) ==
          ACTUATOR_UNWRAP_AMBIGUOUS_DELTA);
}

static void test_half_turn_update_fails_closed(void) {
    actuator_joint_unwrapper_t state;
    int32_t value = 0;

    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 1000u, 1000, 64u) ==
          ACTUATOR_UNWRAP_OK);
    CHECK(actuator_joint_unwrapper_update(&state, 3048u, &value) ==
          ACTUATOR_UNWRAP_AMBIGUOUS_DELTA);
    CHECK(state.unwrapped_raw == 1000);
    CHECK(state.previous_raw == 1000u);
}

static void test_unwrapped_command_validates_before_modulo(void) {
    const actuator_unwrapped_joint_limit_t limit = {1859, 4188};
    int32_t unwrapped_raw = 0;
    int32_t position_urad = 0;
    uint16_t modulo_raw = 0u;

    CHECK(actuator_unwrapped_raw_to_urad(
              2048u, 1, 4188, &position_urad) == ACTUATOR_UNWRAP_OK);
    CHECK(actuator_unwrapped_urad_to_raw(
              2048u, 1, position_urad, &limit,
              &unwrapped_raw, &modulo_raw) == ACTUATOR_UNWRAP_OK);
    CHECK(unwrapped_raw == 4188);
    CHECK(modulo_raw == 92u);

    CHECK(actuator_unwrapped_raw_to_urad(
              2048u, 1, 4189, &position_urad) == ACTUATOR_UNWRAP_OK);
    CHECK(actuator_unwrapped_urad_to_raw(
              2048u, 1, position_urad, &limit,
              &unwrapped_raw, &modulo_raw) ==
          ACTUATOR_UNWRAP_LIMIT_VIOLATION);
}

/* Nearest-branch arithmetic is necessary, but not permission to move a servo.
 * This does not establish single-turn servo behavior across its raw seam.
 */
static void test_nearest_branch_does_not_choose_a_long_turn(void) {
    static const struct { uint16_t from, to; int32_t delta; } cases[] = {
        {200u, 500u, 300}, {500u, 200u, -300},
        {4090u, 10u, 16}, {10u, 4090u, -16},
        {4095u, 0u, 1}, {0u, 4095u, -1}, {2048u, 2048u, 0}
    };
    for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
        actuator_joint_unwrapper_t state;
        actuator_joint_unwrapper_reset(&state);
        CHECK(actuator_joint_unwrapper_bind(&state, cases[i].to,
              cases[i].from, 2047u) == ACTUATOR_UNWRAP_OK);
        CHECK(state.unwrapped_raw - cases[i].from == cases[i].delta);
    }
    actuator_joint_unwrapper_t state;
    actuator_joint_unwrapper_reset(&state);
    CHECK(actuator_joint_unwrapper_bind(&state, 2248u, 200, 2047u) ==
          ACTUATOR_UNWRAP_AMBIGUOUS_DELTA);
    CHECK(!state.bound);
}

static void test_invalid_short_branch_never_falls_back_to_long_branch(void) {
    /* raw 4106 (=10 mod 4096) is the nearby target from 4090. If outside
     * the permitted branch, reject it; do not silently substitute raw 10. */
    const actuator_unwrapped_joint_limit_t limit = {0, 4095};
    int32_t angle = 0, unwrapped = 123;
    uint16_t modulo = 123u;
    CHECK(actuator_unwrapped_raw_to_urad(2048u, 1, 4106, &angle) == ACTUATOR_UNWRAP_OK);
    CHECK(actuator_unwrapped_urad_to_raw(2048u, 1, angle, &limit,
          &unwrapped, &modulo) == ACTUATOR_UNWRAP_LIMIT_VIOLATION);
    CHECK(unwrapped == 123 && modulo == 123u);
}

int main(void) {
    test_tracks_positive_and_negative_wraps();
    test_binding_requires_a_close_verified_reference();
    test_half_turn_update_fails_closed();
    test_unwrapped_command_validates_before_modulo();
    test_nearest_branch_does_not_choose_a_long_turn();
    test_invalid_short_branch_never_falls_back_to_long_branch();
    if (failures != 0) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    puts("joint unwrap tests passed");
    return 0;
}
