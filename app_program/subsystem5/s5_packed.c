#include "s5_packed.h"

#include <stddef.h>

#define S5_DISTANCE_MAX_CM (400u)
#define S5_WIDTH_MAX_CM (100u)
#define S5_ANGLE_MAX_DEG (180u)
#define S5_DISTANCE_BITS (9u)
#define S5_WIDTH_BITS (7u)
#define S5_ANGLE_BITS (8u)
#define S5_DISTANCE_OFFSET (0u)
#define S5_WIDTH_OFFSET (9u)
#define S5_LEFT_OFFSET (16u)
#define S5_RIGHT_OFFSET (25u)
#define S5_ANGLE_OFFSET (34u)
#define S5_VERSION_OFFSET (42u)
#define S5_VERSION_BITS (4u)

/* Bitstream: bit zero is bit zero of byte zero (little-endian bit order).
 * Only the owning task writes a message; recipients get an immutable copy.
 * No read-modify-write of shared packed data is performed. */
static void set_bits(uint8_t *destination, uint8_t offset, uint8_t width,
                     uint16_t value)
{
    uint8_t index;

    for (index = 0u; index < width; ++index)
    {
        uint8_t bit_position = (uint8_t)(offset + index);
        uint8_t mask = (uint8_t)(1u << (bit_position % 8u));

        if ((value & (uint16_t)(1u << index)) != 0u)
        {
            destination[bit_position / 8u] |= mask;
        }
    }
}

static uint16_t get_bits(const uint8_t *source, uint8_t offset, uint8_t width)
{
    uint8_t index;
    uint16_t result = 0u;

    for (index = 0u; index < width; ++index)
    {
        uint8_t bit_position = (uint8_t)(offset + index);
        uint8_t mask = (uint8_t)(1u << (bit_position % 8u));

        if ((source[bit_position / 8u] & mask) != 0u)
        {
            result |= (uint16_t)(1u << index);
        }
    }

    return result;
}

static uint16_t encode_cm(float cm, uint16_t maximum, uint16_t invalid)
{
    /* Comparisons also reject NaN and infinity without libm. */
    if (!((cm >= 0.0f) && (cm <= (float)maximum)))
    {
        return invalid;
    }

    return (uint16_t)(cm + 0.5f);
}

bool s5_pack_profile(const S5Profile *profile, S5PackedProfile *message)
{
    uint16_t distance = S5_DISTANCE_INVALID;
    uint16_t width = S5_WIDTH_INVALID;
    uint16_t left = S5_DISTANCE_INVALID;
    uint16_t right = S5_DISTANCE_INVALID;
    uint16_t angle = S5_ANGLE_INVALID;
    uint8_t index;
    bool valid = false;

    if (message == NULL)
    {
        return false;
    }

    for (index = 0u; index < S5_PACKED_BYTES; ++index)
    {
        message->data[index] = 0u;
    }

    if ((profile != NULL) && profile->valid)
    {
        distance = encode_cm(profile->nearest_cm, S5_DISTANCE_MAX_CM,
                             S5_DISTANCE_INVALID);
        width = encode_cm(profile->estimated_width_cm, S5_WIDTH_MAX_CM,
                          S5_WIDTH_INVALID);
        left = encode_cm(profile->left_clearance_cm, S5_DISTANCE_MAX_CM,
                         S5_DISTANCE_INVALID);
        right = encode_cm(profile->right_clearance_cm, S5_DISTANCE_MAX_CM,
                          S5_DISTANCE_INVALID);

        if (profile->nearest_angle_deg <= S5_ANGLE_MAX_DEG)
        {
            angle = profile->nearest_angle_deg;
        }

        valid = (distance != S5_DISTANCE_INVALID) &&
                (width != S5_WIDTH_INVALID) &&
                (left != S5_DISTANCE_INVALID) &&
                (right != S5_DISTANCE_INVALID) &&
                (angle != S5_ANGLE_INVALID);
    }

    set_bits(message->data, S5_DISTANCE_OFFSET, S5_DISTANCE_BITS, distance);
    set_bits(message->data, S5_WIDTH_OFFSET, S5_WIDTH_BITS, width);
    set_bits(message->data, S5_LEFT_OFFSET, S5_DISTANCE_BITS, left);
    set_bits(message->data, S5_RIGHT_OFFSET, S5_DISTANCE_BITS, right);
    set_bits(message->data, S5_ANGLE_OFFSET, S5_ANGLE_BITS, angle);
    set_bits(message->data, S5_VERSION_OFFSET, S5_VERSION_BITS,
             S5_PACKED_VERSION);
    return valid;
}

bool s5_unpack_profile(const S5PackedProfile *message, S5DecodedProfile *profile)
{
    uint16_t distance;
    uint16_t width;
    uint16_t left;
    uint16_t right;
    uint16_t angle;

    if ((message == NULL) || (profile == NULL))
    {
        return false;
    }

    /* Bits 46..47 are padding; bits 42..45 are the format version. */
    if ((message->data[5] & 0xc0u) != 0u)
    {
        return false;
    }

    if (get_bits(message->data, S5_VERSION_OFFSET,
                 S5_VERSION_BITS) != S5_PACKED_VERSION)
    {
        return false;
    }

    distance = get_bits(message->data, S5_DISTANCE_OFFSET, S5_DISTANCE_BITS);
    width = get_bits(message->data, S5_WIDTH_OFFSET, S5_WIDTH_BITS);
    left = get_bits(message->data, S5_LEFT_OFFSET, S5_DISTANCE_BITS);
    right = get_bits(message->data, S5_RIGHT_OFFSET, S5_DISTANCE_BITS);
    angle = get_bits(message->data, S5_ANGLE_OFFSET, S5_ANGLE_BITS);

    if ((distance > S5_DISTANCE_MAX_CM) || (width > S5_WIDTH_MAX_CM) ||
        (left > S5_DISTANCE_MAX_CM) || (right > S5_DISTANCE_MAX_CM) ||
        (angle > S5_ANGLE_MAX_DEG))
    {
        return false;
    }

    profile->obstacle_distance_cm = distance;
    profile->obstacle_width_cm = (uint8_t)width;
    profile->left_clearance_cm = left;
    profile->right_clearance_cm = right;
    profile->servo_angle_deg = (uint8_t)angle;
    return true;
}
