#ifndef ACTUATOR_CORE_ROBOT_EVIDENCE_H
#define ACTUATOR_CORE_ROBOT_EVIDENCE_H
#include <stdint.h>
#include <stdbool.h>
/* Trusted host observations, not a safety-rated sensor channel. Flags are
 * independently measured arms-safe/lift-hold/payload-safe, not command echoes.
 * Expiry and source time are synchronized MCU milliseconds. Boot/sequence
 * prevent a restarted PC from silently reviving an old observation stream. */
typedef struct { uint32_t sequence,observed_ms,expires_ms; uint8_t flags; bool valid; } actuator_robot_evidence_t;
bool actuator_robot_evidence_accept(actuator_robot_evidence_t *s,uint32_t boot,
    uint32_t maximum_age_ms,uint32_t now,const uint8_t request[36],uint8_t response[64]);
bool actuator_robot_evidence_fresh(const actuator_robot_evidence_t *s,uint32_t now,uint32_t maximum_age_ms);
#endif
