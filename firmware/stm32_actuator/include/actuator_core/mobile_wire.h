#ifndef ACTUATOR_CORE_MOBILE_WIRE_H
#define ACTUATOR_CORE_MOBILE_WIRE_H
#include "actuator_core/mobile_supervisor.h"
#include <stddef.h>
#define ACTUATOR_MOBILE_WIRE_SIZE 36u
/* Experimental fixed command frame, NOT registered in the live arm protocol.
 * AM / version 1 / opcode; u32 session/sequence/deadline; 4 i32 velocities;
 * CRC32C over bytes 0..31. All fields little endian. No native struct casting.
 * MCU-time synchronization and a capability handshake are required before
 * connecting this codec to a host UART parser. No implicit enable on decode. */
typedef enum { ACTUATOR_MOBILE_WIRE_ARM = 1, ACTUATOR_MOBILE_WIRE_VELOCITY = 2,
               ACTUATOR_MOBILE_WIRE_STOP = 3 } actuator_mobile_opcode_t;
typedef struct {
    actuator_mobile_opcode_t opcode;
    uint32_t session, sequence, valid_until_ms;
    int32_t velocity_raw[4];
} actuator_mobile_message_t;
bool actuator_mobile_wire_encode(const actuator_mobile_message_t *message,
    uint8_t packet[ACTUATOR_MOBILE_WIRE_SIZE]);
bool actuator_mobile_wire_decode(const uint8_t *packet, size_t length,
    actuator_mobile_message_t *message);
/* Checks expiry, session and replay before mutating targets. Invalid packets
 * still poll the watchdog. STOP is session-scoped normal stop, not E-stop. */
bool actuator_mobile_wire_apply(actuator_mobile_supervisor_t *s,
    const uint8_t *packet, size_t length, uint32_t now_ms);
#endif
