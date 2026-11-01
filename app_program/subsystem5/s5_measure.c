/* Buddy 5: bounded ultrasonic consensus. Invalid readings never grant drive. */
#include "subsystem5.h"
#include "s5_measure.h"
#include "drv_ultrasonic.h"
#include "../robot_config.h"

bool s5_echo_suppressed_cm(float cm)
{
    if (!(cm > ROBOT_IGNORE_ECHO_AT_OR_BELOW_CM))
    {
        return true;
    }
#if ROBOT_SUPPRESS_REPEATING_ECHO_BANDS
    if (((cm >= ROBOT_FALSE_ECHO_NEAR_MIN_CM) &&
         (cm <= ROBOT_FALSE_ECHO_NEAR_MAX_CM)) ||
        ((cm >= ROBOT_FALSE_ECHO_FAR_MIN_CM) &&
         (cm <= ROBOT_FALSE_ECHO_FAR_MAX_CM)))
    {
        return true;
    }
#endif
    return false;
}

static float abs_difference(float first, float second)
{
    return (first > second) ? (first - second) : (second - first);
}

static bool agreeing(float first, float second)
{
    float average = (first + second) * 0.5f;
    float tolerance = average * ROBOT_ECHO_AGREEMENT_FRACTION;

    if (tolerance < ROBOT_ECHO_AGREEMENT_MIN_CM)
    {
        tolerance = ROBOT_ECHO_AGREEMENT_MIN_CM;
    }

    return abs_difference(first, second) <= tolerance;
}

/* Three pairwise-agreeing readings are needed within five separated pings.
 * Do not use a median of invalid or inconsistent measurements as clearance. */
static bool measure_one_batch(float *out_cm)
{
    float values[ROBOT_ECHO_SAMPLES];
    uint8_t count = 0u;
    uint8_t i;
    uint8_t j;
    uint8_t k;
    bool found = false;
    float best_spread = 401.0f;
    float best_cm = 0.0f;

    if (out_cm == NULL)
    {
        return false;
    }

    *out_cm = 0.0f;
    for (i = 0u; i < ROBOT_ECHO_SAMPLES; ++i)
    {
        S5EchoReading reading = s5_ultrasonic_read_detailed();

        if ((reading.status == S5_ECHO_OK) &&
            !s5_echo_suppressed_cm(reading.cm))
        {
            values[count] = reading.cm;
            ++count;
        }

        if ((uint8_t)(i + 1u) < ROBOT_ECHO_SAMPLES)
        {
            tk_dly_tsk((reading.status == S5_ECHO_OK) ?
                       S5_ECHO_GAP_MS : ROBOT_ECHO_GAP_AFTER_FAILURE_MS);
        }
    }

    if (count < ROBOT_ECHO_REQUIRED_AGREEING)
    {
        return false;
    }

    for (i = 0u; i < count; ++i)
    {
        for (j = (uint8_t)(i + 1u); j < count; ++j)
        {
            for (k = (uint8_t)(j + 1u); k < count; ++k)
            {
                if (agreeing(values[i], values[j]) &&
                    agreeing(values[i], values[k]) &&
                    agreeing(values[j], values[k]))
                {
                    float minimum = values[i];
                    float maximum = values[i];
                    float spread;

                    if (values[j] < minimum)
                    {
                        minimum = values[j];
                    }
                    if (values[k] < minimum)
                    {
                        minimum = values[k];
                    }
                    if (values[j] > maximum)
                    {
                        maximum = values[j];
                    }
                    if (values[k] > maximum)
                    {
                        maximum = values[k];
                    }

                    spread = maximum - minimum;
                    if (!found || (spread < best_spread))
                    {
                        found = true;
                        best_spread = spread;
                        best_cm = (values[i] + values[j] + values[k]) / 3.0f;
                    }
                }
            }
        }
    }

    if (!found)
    {
        return false;
    }

    *out_cm = best_cm;
    return true;
}

bool s5_measure_stable_cm(float *out_cm)
{
    uint8_t attempt;

    if (out_cm == NULL)
    {
        return false;
    }

    *out_cm = 0.0f;
    for (attempt = 0u; attempt < ROBOT_ECHO_BATCH_ATTEMPTS; ++attempt)
    {
        if (measure_one_batch(out_cm))
        {
            return true;
        }

        if ((uint8_t)(attempt + 1u) < ROBOT_ECHO_BATCH_ATTEMPTS)
        {
            tk_dly_tsk(ROBOT_ECHO_BATCH_RETRY_PAUSE_MS);
        }
    }

    return false;
}
