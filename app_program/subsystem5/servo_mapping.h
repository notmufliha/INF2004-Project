#ifndef S5_SERVO_MAPPING_H
#define S5_SERVO_MAPPING_H
#include <stdint.h>
#include "../robot_config.h"

/* Convert robot-relative scan bearings to the mounted servo's shaft angle. */
static inline uint16_t s5_servo_mount_angle(uint16_t bearing)
{
    if (bearing > 180u) bearing = 180u;
    return ROBOT_SCAN_SERVO_REVERSED ? (uint16_t)(180u - bearing) : bearing;
}
#endif
