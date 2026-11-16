#include "actuator_core/wire_bytes.h"
#include "actuator_core/mobile_wire.h"
#include "actuator_core/crc32c.h"
#include <string.h>
static bool valid(const actuator_mobile_message_t *m) {
    unsigned i;
    if (m == NULL || m->session == 0u || m->opcode < ACTUATOR_MOBILE_WIRE_ARM ||
        m->opcode > ACTUATOR_MOBILE_WIRE_STOP) return false;
    if (m->opcode == ACTUATOR_MOBILE_WIRE_ARM && m->sequence != 0u) return false;
    for (i=0u;i<4u;i++) {
        if (m->velocity_raw[i] < -32767 || m->velocity_raw[i] > 32767 ||
            (m->opcode != ACTUATOR_MOBILE_WIRE_VELOCITY && m->velocity_raw[i] != 0)) return false;
    }
    return true;
}
bool actuator_mobile_wire_encode(const actuator_mobile_message_t *message,
    uint8_t packet[ACTUATOR_MOBILE_WIRE_SIZE]) {
    uint8_t candidate[ACTUATOR_MOBILE_WIRE_SIZE] = {0};
    unsigned i;
    if (packet == NULL || !valid(message)) return false;
    candidate[0]='A'; candidate[1]='M'; candidate[2]=1u; candidate[3]=(uint8_t)message->opcode;
    actuator_write_u32_le(candidate+4,message->session); actuator_write_u32_le(candidate+8,message->sequence);
    actuator_write_u32_le(candidate+12,message->valid_until_ms);
    for (i=0u;i<4u;i++) actuator_write_u32_le(candidate+16+4*i,(uint32_t)message->velocity_raw[i]);
    actuator_write_u32_le(candidate+32,actuator_crc32c(candidate,32));
    memcpy(packet,candidate,sizeof(candidate));
    return true;
}
bool actuator_mobile_wire_decode(const uint8_t *packet, size_t length,
    actuator_mobile_message_t *message) {
    actuator_mobile_message_t candidate = {0};
    unsigned i;
    if (packet == NULL || message == NULL || length != ACTUATOR_MOBILE_WIRE_SIZE ||
        packet[0]!='A' || packet[1]!='M' || packet[2]!=1u ||
        actuator_read_u32_le(packet+32) != actuator_crc32c(packet,32)) return false;
    candidate.opcode=(actuator_mobile_opcode_t)packet[3];
    candidate.session=actuator_read_u32_le(packet+4); candidate.sequence=actuator_read_u32_le(packet+8);
    candidate.valid_until_ms=actuator_read_u32_le(packet+12);
    for (i=0u;i<4u;i++) {
        uint32_t word=actuator_read_u32_le(packet+16+4*i);
        candidate.velocity_raw[i] = word <= INT32_MAX ? (int32_t)word :
            (int32_t)((int64_t)word - INT64_C(4294967296));
    }
    if (!valid(&candidate)) return false;
    *message=candidate;
    return true;
}
bool actuator_mobile_wire_apply(actuator_mobile_supervisor_t *s,
    const uint8_t *packet, size_t length, uint32_t now_ms) {
    actuator_mobile_message_t message;
    uint32_t remaining;
    if (s == NULL) return false;
    actuator_mobile_poll(s,now_ms);
    if (!s->configured || !actuator_mobile_wire_decode(packet,length,&message)) return false;
    remaining=message.valid_until_ms-now_ms;
    if (remaining == 0u || remaining > s->config.command_timeout_ms) return false;
    if (message.opcode == ACTUATOR_MOBILE_WIRE_ARM)
        return actuator_mobile_arm(s,message.session,now_ms);
    if (message.opcode == ACTUATOR_MOBILE_WIRE_VELOCITY)
        return actuator_mobile_command_until(s,message.session,message.sequence,
            message.velocity_raw,message.valid_until_ms,now_ms);
    if (message.session != s->session || s->state == ACTUATOR_MOBILE_DISABLED ||
        (s->have_sequence && message.sequence <= s->sequence)) return false;
    actuator_mobile_stop(s);
    s->sequence=message.sequence;
    s->have_sequence=true;
    return true;
}
