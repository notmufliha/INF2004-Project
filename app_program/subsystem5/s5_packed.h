/* Buddy 5 compact wire format. Fixed order and little-endian bit numbering.
 * The six-byte message is independent of C bit-field layout and alignment.
 * It contains 42 B5 data bits, four version bits and two zero pad bits.
 * Never send a native C structure over MQTT or a message buffer.
 */
#ifndef S5_PACKED_H
#define S5_PACKED_H

#include <stdbool.h>
#include <stdint.h>
#include "s5_logic.h"

#define S5_PACKED_BYTES (6u)
#define S5_PACKED_BITS (42u)
#define S5_PACKED_VERSION (1u)
#define S5_DISTANCE_INVALID (511u)
#define S5_WIDTH_INVALID (127u)
#define S5_ANGLE_INVALID (255u)

typedef struct
{
    uint8_t data[S5_PACKED_BYTES];
} S5PackedProfile;

typedef struct
{
    uint16_t obstacle_distance_cm;
    uint8_t obstacle_width_cm;
    uint16_t left_clearance_cm;
    uint16_t right_clearance_cm;
    uint8_t servo_angle_deg;
} S5DecodedProfile;

/* Returns false when a source field was invalid; still encodes a well-defined
 * sentinel for every invalid field. The payload is always initialized. */
bool s5_pack_profile(const S5Profile *profile, S5PackedProfile *message);
/* Rejects reserved codes, corrupt padding and invalid/sentinel fields. */
bool s5_unpack_profile(const S5PackedProfile *message, S5DecodedProfile *profile);

#endif /* S5_PACKED_H */
