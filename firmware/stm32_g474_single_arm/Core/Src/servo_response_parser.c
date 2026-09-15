#include "servo_response_parser.h"
void ServoResponseParser_Init(ServoResponseParser *parser, uint8_t id, uint8_t length)
{
    actuator_sts_response_init(parser, id, length);
}
ServoResponseParseResult ServoResponseParser_Push(ServoResponseParser *parser, uint8_t byte)
{
    return actuator_sts_response_push(parser, byte);
}
const uint8_t *ServoResponseParser_Data(const ServoResponseParser *parser)
{
    return actuator_sts_response_data(parser);
}
