/* Host-only test: no Pico SDK or RTOS required. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "s5_packed.h"

static void roundtrip(float distance, float width, float left,
                      float right, uint16_t angle)
{
    S5Profile source = {0};
    S5PackedProfile packet;
    S5DecodedProfile result;
    bool packed;

    source.valid = true;
    source.nearest_cm = distance;
    source.estimated_width_cm = width;
    source.left_clearance_cm = left;
    source.right_clearance_cm = right;
    source.nearest_angle_deg = angle;
    packed = s5_pack_profile(&source, &packet);
    assert(packed);
    assert(s5_unpack_profile(&packet, &result));
    assert(result.obstacle_distance_cm == (uint16_t)(distance + 0.5f));
    assert(result.obstacle_width_cm == (uint8_t)(width + 0.5f));
    assert(result.left_clearance_cm == (uint16_t)(left + 0.5f));
    assert(result.right_clearance_cm == (uint16_t)(right + 0.5f));
    assert(result.servo_angle_deg == angle);
    assert((packet.data[5] & 0xc0u) == 0u);
}

int main(void)
{
    S5Profile source = {0};
    S5PackedProfile packet;
    S5DecodedProfile result;
    uint16_t angle;

    assert(sizeof(packet) == 6u);
    roundtrip(0.0f, 0.0f, 0.0f, 0.0f, 0u);
    roundtrip(400.0f, 100.0f, 400.0f, 400.0f, 180u);
    roundtrip(35.0f, 20.0f, 45.0f, 80.0f, 90u);
    roundtrip(34.5f, 99.5f, 44.5f, 399.5f, 1u);

    for (angle = 0u; angle <= 180u; ++angle)
    {
        roundtrip(23.0f, 12.0f, 45.0f, 66.0f, angle);
    }

    assert(!s5_pack_profile(&source, &packet));
    assert(!s5_unpack_profile(&packet, &result));

    source.valid = true;
    source.nearest_cm = 401.0f;
    source.estimated_width_cm = 50.0f;
    source.left_clearance_cm = 80.0f;
    source.right_clearance_cm = 80.0f;
    source.nearest_angle_deg = 90u;
    assert(!s5_pack_profile(&source, &packet));
    assert(!s5_unpack_profile(&packet, &result));

    source.nearest_cm = 20.0f;
    assert(s5_pack_profile(&source, &packet));
    packet.data[5] ^= 0x04u; /* Unsupported protocol version. */
    assert(!s5_unpack_profile(&packet, &result));

    assert(s5_pack_profile(&source, &packet));
    packet.data[5] |= 0x80u; /* Nonzero padding is forbidden. */
    assert(!s5_unpack_profile(&packet, &result));

    assert(s5_pack_profile(&source, &packet));
    packet.data[0] = 0xffu;
    packet.data[1] |= 0x01u; /* 511: invalid distance. */
    assert(!s5_unpack_profile(&packet, &result));

    puts("[HOST TEST] B5 42-bit packed format + version: PASS");
    return 0;
}
