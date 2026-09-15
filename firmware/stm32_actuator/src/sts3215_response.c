#include "actuator_core/sts3215_response.h"

#include <stddef.h>
#include <string.h>

enum
{
    ACTUATOR_STS_RESPONSE_READ_ID = 2,
    ACTUATOR_STS_RESPONSE_READ_LENGTH = 3,
    ACTUATOR_STS_RESPONSE_READ_BODY = 4
};

static void reset_frame(
    actuator_sts_response_t *parser
)
{
    parser->sync_count = 0U;
    parser->frame_id = 0U;
    parser->frame_length = 0U;
    parser->body_index = 0U;
}

static actuator_sts_response_result_t reject_frame(
    actuator_sts_response_t *parser,
    actuator_sts_response_reject_t reason,
    uint16_t frame_bytes
)
{
    parser->last_reject = reason;
    if ((uint16_t)(UINT16_MAX - parser->discarded_bytes) < frame_bytes)
    {
        parser->discarded_bytes = UINT16_MAX;
    }
    else
    {
        parser->discarded_bytes =
            (uint16_t)(parser->discarded_bytes + frame_bytes);
    }
    reset_frame(parser);
    return ACTUATOR_STS_RESPONSE_FRAME_REJECTED;
}

void actuator_sts_response_init(
    actuator_sts_response_t *parser,
    uint8_t expected_id,
    uint8_t expected_data_length
)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->valid_configuration = expected_id > 0U && expected_id < 254U &&
        expected_data_length <= ACTUATOR_STS_RESPONSE_MAX_BODY_LENGTH - 2U;
    parser->expected_id = expected_id;
    parser->expected_data_length = expected_data_length;
}

actuator_sts_response_result_t actuator_sts_response_push(
    actuator_sts_response_t *parser,
    uint8_t byte
)
{
    if (parser == NULL)
    {
        return ACTUATOR_STS_RESPONSE_FRAME_REJECTED;
    }

    if (!parser->valid_configuration) return ACTUATOR_STS_RESPONSE_FRAME_REJECTED;
    if (parser->terminal != ACTUATOR_STS_RESPONSE_NEED_MORE) return parser->terminal;

    if (parser->sync_count < 2U)
    {
        if (byte == 0xFFU)
        {
            parser->sync_count++;
        }
        else
        {
            parser->sync_count = 0U;
            if (parser->discarded_bytes < UINT16_MAX)
            {
                parser->discarded_bytes++;
            }
            parser->last_reject = ACTUATOR_STS_RESPONSE_REJECT_HEADER;
        }
        return ACTUATOR_STS_RESPONSE_NEED_MORE;
    }

    if (parser->sync_count == ACTUATOR_STS_RESPONSE_READ_ID)
    {
        if (byte == 0xFFU)
        {
            /*
             * Keep the last two bytes as a possible header. This handles an
             * arbitrary run of 0xFF bytes and the overlap between a stale
             * trailing sync byte and the next valid frame's FF FF header.
             * A unicast servo status response can never use ID 0xFF.
             */
            return ACTUATOR_STS_RESPONSE_NEED_MORE;
        }
        parser->frame_id = byte;
        parser->sync_count = ACTUATOR_STS_RESPONSE_READ_LENGTH;
        return ACTUATOR_STS_RESPONSE_NEED_MORE;
    }

    if (parser->sync_count == ACTUATOR_STS_RESPONSE_READ_LENGTH)
    {
        parser->frame_length = byte;
        parser->body_index = 0U;
        if ((byte < 2U) || (byte > ACTUATOR_STS_RESPONSE_MAX_BODY_LENGTH))
        {
            return reject_frame(
                parser,
                ACTUATOR_STS_RESPONSE_REJECT_LENGTH,
                4U
            );
        }
        parser->sync_count = ACTUATOR_STS_RESPONSE_READ_BODY;
        return ACTUATOR_STS_RESPONSE_NEED_MORE;
    }

    parser->body[parser->body_index++] = byte;
    if (parser->body_index < parser->frame_length)
    {
        return ACTUATOR_STS_RESPONSE_NEED_MORE;
    }

    uint8_t sum = (uint8_t)(
        parser->frame_id + parser->frame_length
    );
    for (uint8_t index = 0U;
         index < (uint8_t)(parser->frame_length - 1U);
         index++)
    {
        sum = (uint8_t)(sum + parser->body[index]);
    }

    uint16_t frame_bytes = (uint16_t)parser->frame_length + 4U;
    if (parser->body[parser->frame_length - 1U] != (uint8_t)(~sum))
    {
        return reject_frame(
            parser,
            ACTUATOR_STS_RESPONSE_REJECT_CHECKSUM,
            frame_bytes
        );
    }
    if (parser->frame_id != parser->expected_id)
    {
        return reject_frame(
            parser,
            ACTUATOR_STS_RESPONSE_REJECT_ID,
            frame_bytes
        );
    }
    /* Some devices return only status+checksum on error. Do not confuse a
     * local READ echo (length 4, instruction 2) with such an error reply. */
    if (parser->frame_length == 2U && parser->body[0] != 0U)
    {
        parser->servo_status = parser->body[0];
        parser->terminal = ACTUATOR_STS_RESPONSE_STATUS_ERROR;
        return parser->terminal;
    }
    if (parser->frame_length !=
        (uint8_t)(parser->expected_data_length + 2U))
    {
        return reject_frame(
            parser,
            ACTUATOR_STS_RESPONSE_REJECT_LENGTH,
            frame_bytes
        );
    }

    parser->servo_status = parser->body[0];
    if (parser->servo_status != 0U)
    {
        parser->terminal = ACTUATOR_STS_RESPONSE_STATUS_ERROR;
        return parser->terminal;
    }
    parser->terminal = ACTUATOR_STS_RESPONSE_FRAME_READY;
    return parser->terminal;
}

const uint8_t *actuator_sts_response_data(
    const actuator_sts_response_t *parser
)
{
    if (parser == NULL)
    {
        return NULL;
    }
    return parser->terminal == ACTUATOR_STS_RESPONSE_FRAME_READY ? &parser->body[1] : NULL;
}
