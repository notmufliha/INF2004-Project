#include "motion_profile.h"

/* Preliminary tuning values; validate on the loaded robot. */
#define PROFILE_CORRECTION_GAIN 2.0f
#define PROFILE_CORRECTION_LIMIT 0.25f
#define PROFILE_SLOW_DISTANCE_MM 30.0f
#define PROFILE_MIN_SPEED_MM_S 5.0f

static float profile_approach(float speed, float remaining)
{
    float limit = speed;
    if (remaining < PROFILE_SLOW_DISTANCE_MM)
    {
        limit = speed * remaining / PROFILE_SLOW_DISTANCE_MM;
        if (limit < PROFILE_MIN_SPEED_MM_S)
        {
            limit = PROFILE_MIN_SPEED_MM_S;
        }
    }
    return (limit < speed) ? limit : speed;
}

motion_profile_t motion_profile_targets(float speed, float steering,
    float left_progress, float right_progress, float target,
    bool bounded, bool correct_straight)
{
    motion_profile_t result = {0};
    float correction = 0.0f;
    float limit = speed * PROFILE_CORRECTION_LIMIT;

    if (correct_straight && steering == 0.0f)
    {
        correction = (left_progress - right_progress) *
            PROFILE_CORRECTION_GAIN;
        if (correction > limit)
        {
            correction = limit;
        }
        if (correction < -limit)
        {
            correction = -limit;
        }
    }
    result.left_speed = speed * (1.0f + steering) - correction;
    result.right_speed = speed * (1.0f - steering) + correction;
    if (bounded)
    {
        result.left_reached = left_progress >= target;
        result.right_reached = right_progress >= target;
        if (result.left_reached)
        {
            result.left_speed = 0.0f;
        }
        else
        {
            result.left_speed = profile_approach(result.left_speed,
                target - left_progress);
        }
        if (result.right_reached)
        {
            result.right_speed = 0.0f;
        }
        else
        {
            result.right_speed = profile_approach(result.right_speed,
                target - right_progress);
        }
    }
    return result;
}
