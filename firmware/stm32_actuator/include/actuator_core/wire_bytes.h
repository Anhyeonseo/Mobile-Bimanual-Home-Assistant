#ifndef ACTUATOR_CORE_WIRE_BYTES_H
#define ACTUATOR_CORE_WIRE_BYTES_H

#include <stdint.h>

/* Byte-wise access also works on unaligned packet fields. Callers validate
 * packet lengths before access; these helpers do not define a wire protocol. */
static inline uint16_t actuator_read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

static inline uint32_t actuator_read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
        ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

static inline int32_t actuator_read_i32_le(const uint8_t *p)
{
    uint32_t word = actuator_read_u32_le(p);
    return word <= INT32_MAX ? (int32_t)word :
        (int32_t)((int64_t)word - INT64_C(4294967296));
}

static inline void actuator_write_u16_le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
}

static inline void actuator_write_u32_le(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
    p[2] = (uint8_t)(value >> 16U);
    p[3] = (uint8_t)(value >> 24U);
}

#endif
