#ifndef ACTUATOR_CORE_MOBILE_FRAMED_H
#define ACTUATOR_CORE_MOBILE_FRAMED_H
#include "actuator_core/mobile_endpoint.h"
#include "actuator_core/protocol.h"
#define ACTUATOR_V2_MSG_MOBILE_REQUEST UINT8_C(64)
#define ACTUATOR_V2_MSG_MOBILE_RESPONSE UINT8_C(65)
/* One v2 COBS envelope owns host UART framing. Never interleave raw AM/AS with
 * arm packets. Reply payload: status byte (0 OK, 1 unbound, 2 malformed, 3 host fault inhibited) then
 * exactly 64 AS bytes only for status=0. Inner rejection stays in AS opcode.
 * NULL endpoint is the board default until real limits/feedback are attached.
 * now_ms is the processing clock, not old host/receive time. */
bool actuator_mobile_framed_reply(actuator_mobile_endpoint_t *endpoint,
    const actuator_frame_t *request, uint32_t now_ms, actuator_frame_t *response);
#endif
