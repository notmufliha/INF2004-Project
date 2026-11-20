/** @file encoder_numeric.h
 * @brief Exact signed 24-bit encoder storage and bounded speed publication.
 */
#ifndef ENCODER_NUMERIC_H
#define ENCODER_NUMERIC_H

#include <stdint.h>
#include <stdbool.h>

#define ENC_COUNT_MIN (-INT32_C(8388608))
#define ENC_COUNT_MAX INT32_C(8388607)
#define ENC_SPEED_MIN (-INT32_C(8192))
#define ENC_SPEED_MAX INT32_C(8191)

typedef struct
{
    uint8_t bytes[3];
} encoder_count_t;

static inline int32_t encoder_count_read(const volatile encoder_count_t *value)
{
    uint32_t raw = (uint32_t)value->bytes[0] |
        ((uint32_t)value->bytes[1] << 8) |
        ((uint32_t)value->bytes[2] << 16);
    return (raw & UINT32_C(0x800000)) != 0u ?
        (int32_t)raw - INT32_C(16777216) : (int32_t)raw;
}

/* Caller guarantees the value is in the declared signed 24-bit range. */
static inline void encoder_count_write(volatile encoder_count_t *value, int32_t count)
{
    uint32_t raw = (uint32_t)count;
    value->bytes[0] = (uint8_t)raw;
    value->bytes[1] = (uint8_t)(raw >> 8);
    value->bytes[2] = (uint8_t)(raw >> 16);
}

/* Saturate instead of wrapping; false requires a latched encoder fault. */
static inline bool encoder_count_step(volatile encoder_count_t *value, int32_t delta)
{
    int64_t next = (int64_t)encoder_count_read(value) + delta;
    bool valid = next >= ENC_COUNT_MIN && next <= ENC_COUNT_MAX;
    if (next > ENC_COUNT_MAX)
    {
        next = ENC_COUNT_MAX;
    }
    if (next < ENC_COUNT_MIN)
    {
        next = ENC_COUNT_MIN;
    }
    encoder_count_write(value, (int32_t)next);
    return valid;
}

/* Fault detection uses the original float before this telemetry clamp. */
static inline int32_t encoder_speed_publish(float speed)
{
    if (speed > (float)ENC_SPEED_MAX)
    {
        return ENC_SPEED_MAX;
    }
    if (speed < (float)ENC_SPEED_MIN)
    {
        return ENC_SPEED_MIN;
    }
    return speed == speed ? (int32_t)speed : 0;
}

#endif
