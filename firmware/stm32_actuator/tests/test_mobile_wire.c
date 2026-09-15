#include "actuator_core/mobile_wire.h"
#include "actuator_core/crc32c.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do {if (!(x)) {fprintf(stderr,"line %d: %s\n",__LINE__,#x);return 1;}} while(0)
int main(void) {
    actuator_mobile_config_t config={{100,100,100,50},2,100,200,0,100000};
    actuator_mobile_supervisor_t s;
    actuator_mobile_feedback_t f={0,{0,0,0,0},50000,true,true,true};
    actuator_mobile_message_t m={ACTUATOR_MOBILE_WIRE_ARM,1,0,100,{0,0,0,0}}, decoded={0};
    uint8_t packet[36], saved[36];
    CHECK(actuator_crc32c((const uint8_t *)"123456789",9)==UINT32_C(0xe3069283));
    CHECK(actuator_mobile_init(&s,&config));
    CHECK(actuator_mobile_feedback(&s,&f,0));
    CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(actuator_mobile_wire_decode(packet,36,&decoded));
    CHECK(decoded.session==1 && decoded.opcode==ACTUATOR_MOBILE_WIRE_ARM);
    CHECK(actuator_mobile_wire_apply(&s,packet,36,0));
    CHECK(!actuator_mobile_wire_apply(&s,packet,36,0));
    m.opcode=ACTUATOR_MOBILE_WIRE_VELOCITY; m.sequence=1; m.valid_until_ms=20;
    m.velocity_raw[0]=-25; m.velocity_raw[3]=10;
    CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(actuator_mobile_wire_apply(&s,packet,36,10));
    CHECK(s.target_raw[0]==-25 && s.command_valid_for_ms==10);
    CHECK(!actuator_mobile_wire_apply(&s,packet,36,11));
    actuator_mobile_poll(&s,20);
    CHECK(s.state==ACTUATOR_MOBILE_STOP_LATCHED && s.target_raw[0]==0);
    m.opcode=ACTUATOR_MOBILE_WIRE_ARM;m.session=2;m.sequence=0;m.valid_until_ms=120;
    memset(m.velocity_raw,0,sizeof(m.velocity_raw));
    CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(actuator_mobile_wire_apply(&s,packet,36,20));
    m.opcode=ACTUATOR_MOBILE_WIRE_VELOCITY;m.sequence=1;m.valid_until_ms=20;
    CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(!actuator_mobile_wire_apply(&s,packet,36,20)); /* boundary expired */
    m.valid_until_ms=121;CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(!actuator_mobile_wire_apply(&s,packet,36,20)); /* too far future */
    m.valid_until_ms=50;CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(actuator_mobile_wire_apply(&s,packet,36,20));
    m.opcode=ACTUATOR_MOBILE_WIRE_STOP;m.sequence=2;m.session=1;
    CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(!actuator_mobile_wire_apply(&s,packet,36,21));
    m.session=2;CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(actuator_mobile_wire_apply(&s,packet,36,21));
    CHECK(s.reason==ACTUATOR_MOBILE_REASON_REQUEST);
    memcpy(saved,packet,sizeof(saved));
    m.velocity_raw[0]=1;
    CHECK(!actuator_mobile_wire_encode(&m,packet));
    CHECK(memcmp(packet,saved,sizeof(saved))==0);
    decoded.session=123;
    packet[16]^=1;
    CHECK(!actuator_mobile_wire_decode(packet,36,&decoded));
    CHECK(decoded.session==123);
    CHECK(!actuator_mobile_wire_decode(saved,35,&decoded));
    CHECK(actuator_mobile_init(&s,&config));
    f.observed_ms=UINT32_MAX-5;
    CHECK(actuator_mobile_feedback(&s,&f,f.observed_ms));
    CHECK(actuator_mobile_arm(&s,1,f.observed_ms));
    m.opcode=ACTUATOR_MOBILE_WIRE_VELOCITY;m.session=1;m.sequence=1;m.valid_until_ms=4;
    CHECK(actuator_mobile_wire_encode(&m,packet));
    CHECK(actuator_mobile_wire_apply(&s,packet,36,UINT32_MAX-5));
    CHECK(s.command_valid_for_ms==10);
    actuator_mobile_poll(&s,4);
    CHECK(s.state==ACTUATOR_MOBILE_STOP_LATCHED);
    return 0;
}
