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

/* Shortest signed displacement across rollover. Sample before either
 * wheel travels half the counter range (over 340 m between readings). */
static inline int32_t encoder_count_delta(int32_t now, int32_t previous)
{
    uint32_t raw = ((uint32_t)now - (uint32_t)previous) &
        UINT32_C(0xffffff);
    return raw >= UINT32_C(0x800000) ?
        (int32_t)raw - INT32_C(16777216) : (int32_t)raw;
}

/* Unsigned rollover is intentional; absolute travel is not a fault. */
static inline void encoder_count_step(volatile encoder_count_t *value,
    int32_t delta)
{
    uint32_t raw = ((uint32_t)encoder_count_read(value) + (uint32_t)delta) &
        UINT32_C(0xffffff);
    encoder_count_write(value, raw >= UINT32_C(0x800000) ?
        (int32_t)raw - INT32_C(16777216) : (int32_t)raw);
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
