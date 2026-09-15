#ifndef ACTUATOR_CORE_STS3215_RESPONSE_H
#define ACTUATOR_CORE_STS3215_RESPONSE_H

#include <stdint.h>

#define ACTUATOR_STS_RESPONSE_MAX_BODY_LENGTH UINT8_C(18)

typedef enum
{
    ACTUATOR_STS_RESPONSE_REJECT_NONE = 0,
    ACTUATOR_STS_RESPONSE_REJECT_HEADER = 1,
    ACTUATOR_STS_RESPONSE_REJECT_ID = 2,
    ACTUATOR_STS_RESPONSE_REJECT_LENGTH = 3,
    ACTUATOR_STS_RESPONSE_REJECT_CHECKSUM = 4
} actuator_sts_response_reject_t;

typedef enum
{
    ACTUATOR_STS_RESPONSE_NEED_MORE = 0,
    ACTUATOR_STS_RESPONSE_FRAME_READY = 1,
    ACTUATOR_STS_RESPONSE_FRAME_REJECTED = 2,
    ACTUATOR_STS_RESPONSE_STATUS_ERROR = 3
} actuator_sts_response_result_t;

typedef struct
{
    uint8_t valid_configuration;
    actuator_sts_response_result_t terminal;
    uint8_t expected_id;
    uint8_t expected_data_length;
    uint8_t sync_count;
    uint8_t frame_id;
    uint8_t frame_length;
    uint8_t body_index;
    uint8_t body[ACTUATOR_STS_RESPONSE_MAX_BODY_LENGTH];
    uint8_t servo_status;
    actuator_sts_response_reject_t last_reject;
    uint16_t discarded_bytes;
} actuator_sts_response_t;

/* Transaction-scoped, HAL-free parser. READY/STATUS_ERROR are latched until
 * init; extra bytes cannot write past the terminal frame. Invalid ID/size
 * stays rejected. Data is exposed only after a valid, error-free response. */
void actuator_sts_response_init(
    actuator_sts_response_t *parser,
    uint8_t expected_id,
    uint8_t expected_data_length
);

actuator_sts_response_result_t actuator_sts_response_push(
    actuator_sts_response_t *parser,
    uint8_t byte
);

const uint8_t *actuator_sts_response_data(
    const actuator_sts_response_t *parser
);

#endif /* ACTUATOR_CORE_STS3215_RESPONSE_H */
