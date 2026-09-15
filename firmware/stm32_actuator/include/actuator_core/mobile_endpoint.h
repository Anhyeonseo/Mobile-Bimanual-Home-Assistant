#ifndef ACTUATOR_CORE_MOBILE_ENDPOINT_H
#define ACTUATOR_CORE_MOBILE_ENDPOINT_H
#include "actuator_core/mobile_wire.h"
#define ACTUATOR_MOBILE_STATUS_SIZE 64u
/* HELLO=4 STATUS=5 TIME_SYNC=6 use AM v1 command shape, nonzero request
 * nonce in session, zero velocities. Reply is AS v1, request opcode, sequence,
 * MCU tick, boot ID, capabilities, active session/sequence, state/reason/flags,
 * feedback tick, four velocities, lift um, rejected counter, CRC32C (LE).
 * Command replies use their opcode on success, opcode|0x80 on rejection.
 * Boot ID must change after MCU restart; supplied by board initialization. */
typedef struct {
    actuator_mobile_supervisor_t *supervisor;
    uint32_t boot_id, rejected_frames;
    uint8_t buffer[ACTUATOR_MOBILE_WIRE_SIZE];
    uint8_t used;
    uint32_t last_byte_ms;
} actuator_mobile_endpoint_t;
bool actuator_mobile_endpoint_init(actuator_mobile_endpoint_t *endpoint,
    actuator_mobile_supervisor_t *supervisor,uint32_t boot_id);
/* Feed one byte in main-loop context. True yields exactly one reply.
 * Garbage/CRC errors resync by sliding one byte; partial frame expires in
 * 100 ms. No I/O, dynamic memory or silent motion on reconnect. */
bool actuator_mobile_endpoint_feed(actuator_mobile_endpoint_t *endpoint,
    uint8_t byte,uint32_t now_ms,uint8_t reply[ACTUATOR_MOBILE_STATUS_SIZE]);
#endif
