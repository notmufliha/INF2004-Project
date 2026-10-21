#include "subsystem5.h"
#include "s5_measure.h"
#include "drv_ultrasonic.h"
#include "../robot_config.h"

bool s5_echo_suppressed_cm(float cm)
{
    if (!(cm > ROBOT_IGNORE_ECHO_AT_OR_BELOW_CM)) return true;
#if ROBOT_SUPPRESS_REPEATING_ECHO_BANDS
    if ((cm >= ROBOT_FALSE_ECHO_NEAR_MIN_CM &&
         cm <= ROBOT_FALSE_ECHO_NEAR_MAX_CM) ||
        (cm >= ROBOT_FALSE_ECHO_FAR_MIN_CM &&
         cm <= ROBOT_FALSE_ECHO_FAR_MAX_CM)) return true;
#endif
    return false;
}

static float abs_difference(float a, float b) { return a > b ? a-b : b-a; }
static bool agreeing(float a, float b)
{
    float average = (a + b) * 0.5f;
    float tolerance = average * ROBOT_ECHO_AGREEMENT_FRACTION;
    if (tolerance < ROBOT_ECHO_AGREEMENT_MIN_CM)
        tolerance = ROBOT_ECHO_AGREEMENT_MIN_CM;
    return abs_difference(a,b) <= tolerance;
}

/* Cluster five physically separate pings, and require three readings
 * that agree with each other. One-off reflections must not drive motors. */
static bool measure_one_batch(float *out_cm)
{
    float values[ROBOT_ECHO_SAMPLES];
    unsigned count=0u, i, j, k;
    unsigned best=0u;
    float best_spread=401.0f, best_cm=0.0f;
    if (!out_cm) return false;
    *out_cm=0.0f;
    for (i=0u; i<ROBOT_ECHO_SAMPLES; ++i) {
        S5EchoReading r=s5_ultrasonic_read_detailed();
        if (r.status==S5_ECHO_OK) {
            if (!s5_echo_suppressed_cm(r.cm))
                values[count++]=r.cm;
        }
        if (i+1u<ROBOT_ECHO_SAMPLES)
            tk_dly_tsk(r.status==S5_ECHO_OK ? S5_ECHO_GAP_MS
                                           : ROBOT_ECHO_GAP_AFTER_FAILURE_MS);
    }
    if (count < ROBOT_ECHO_REQUIRED_AGREEING) return false;
    /* All three readings in a cluster must agree, not just one pair. */
    for (i=0u; i<count; ++i) {
        for (j=i+1u; j<count; ++j) {
            for (k=j+1u; k<count; ++k) {
                if (agreeing(values[i],values[j]) &&
                    agreeing(values[i],values[k]) &&
                    agreeing(values[j],values[k])) {
                    float min=values[i], max=values[i], spread;
                    if (values[j]<min) min=values[j];
                    if (values[k]<min) min=values[k];
                    if (values[j]>max) max=values[j];
                    if (values[k]>max) max=values[k];
                    spread=max-min;
                    if (!best || spread<best_spread) {
                        best=1u;
                        best_spread=spread;
                        best_cm=(values[i]+values[j]+values[k])/3.0f;
                    }
                }
            }
        }
    }
    if (!best) {
        return false;
    }
    *out_cm=best_cm;
    return true;
}

/* Recover from isolated missed echoes without a GPIO reset/reboot loop.
 * The result remains UNKNOWN if neither batch has three agreeing returns.
 * This routine is not permission to move on missing data. */
bool s5_measure_stable_cm(float *out_cm)
{
    unsigned attempt;
    if (!out_cm) return false;
    *out_cm = 0.0f;
    for (attempt = 0u; attempt < ROBOT_ECHO_BATCH_ATTEMPTS; ++attempt) {
        if (measure_one_batch(out_cm)) return true;
        if (attempt + 1u < ROBOT_ECHO_BATCH_ATTEMPTS)
            tk_dly_tsk(ROBOT_ECHO_BATCH_RETRY_PAUSE_MS);
    }
    return false;
}
