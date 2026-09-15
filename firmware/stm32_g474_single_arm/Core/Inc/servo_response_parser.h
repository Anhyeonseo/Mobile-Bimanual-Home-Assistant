#ifndef SERVO_RESPONSE_PARSER_H
#define SERVO_RESPONSE_PARSER_H

#include "actuator_core/sts3215_response.h"
/* Existing arm API delegates to the shared mobile/arm response parser. */
#define SERVO_RESPONSE_MAX_BODY_LENGTH ACTUATOR_STS_RESPONSE_MAX_BODY_LENGTH
#define SERVO_RESPONSE_REJECT_NONE ACTUATOR_STS_RESPONSE_REJECT_NONE
#define SERVO_RESPONSE_REJECT_HEADER ACTUATOR_STS_RESPONSE_REJECT_HEADER
#define SERVO_RESPONSE_REJECT_ID ACTUATOR_STS_RESPONSE_REJECT_ID
#define SERVO_RESPONSE_REJECT_LENGTH ACTUATOR_STS_RESPONSE_REJECT_LENGTH
#define SERVO_RESPONSE_REJECT_CHECKSUM ACTUATOR_STS_RESPONSE_REJECT_CHECKSUM
#define SERVO_RESPONSE_NEED_MORE ACTUATOR_STS_RESPONSE_NEED_MORE
#define SERVO_RESPONSE_FRAME_READY ACTUATOR_STS_RESPONSE_FRAME_READY
#define SERVO_RESPONSE_FRAME_REJECTED ACTUATOR_STS_RESPONSE_FRAME_REJECTED
#define SERVO_RESPONSE_STATUS_ERROR ACTUATOR_STS_RESPONSE_STATUS_ERROR
typedef actuator_sts_response_reject_t ServoResponseRejectReason;
typedef actuator_sts_response_result_t ServoResponseParseResult;
typedef actuator_sts_response_t ServoResponseParser;
void ServoResponseParser_Init(
    ServoResponseParser *parser,
    uint8_t expected_id,
    uint8_t expected_data_length
);

ServoResponseParseResult ServoResponseParser_Push(
    ServoResponseParser *parser,
    uint8_t byte
);

const uint8_t *ServoResponseParser_Data(
    const ServoResponseParser *parser
);

#endif
