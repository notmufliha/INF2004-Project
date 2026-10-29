/** @file motion_profile.h
 * @brief Hardware-independent wheel target calculation.
 */
#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdbool.h>

typedef struct
{
    float left_speed;
    float right_speed;
    bool left_reached;
    bool right_reached;
} motion_profile_t;

/* All distances are mm and speeds are mm/s. Progress is direction-normalized.
 * Positive steering makes the left wheel faster (a rightward curve).
 * Correction is applied only to straight motion, never intentional steering.
 */
motion_profile_t motion_profile_targets(float speed, float steering,
    float left_progress, float right_progress, float target,
    bool bounded, bool correct_straight);

#endif
