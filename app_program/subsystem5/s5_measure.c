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
static bool measure_one_batch(float *out_cm, bool forward)
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
        S5EchoReading reading = forward ? s5_ultrasonic_read_front() :
                                         s5_ultrasonic_read_detailed();
#if ROBOT_VERBOSE_SENSOR_LOG
        tm_printf((UB*)"[S5-ECHO] ping=%d/%d status=%s pulse_us=%d cm_x100=%d suppressed=%d\n",
                  (int)(i + 1u), (int)ROBOT_ECHO_SAMPLES,
                  s5_echo_status_name(reading.status), (int)reading.pulse_us,
                  (int)(reading.cm * 100.0f),
                  (int)s5_echo_suppressed_cm(reading.cm));
#endif

        /* A real close reading overrides the no-target assumption, even
         * if other pings in this batch miss the obstacle. */
        if (forward && reading.status == S5_ECHO_OK &&
            reading.cm > 0.0f && reading.cm <= ROBOT_STOP_CM) {
            *out_cm = reading.cm;
            return true;
        }
        if ((reading.status == S5_ECHO_OK ||
             (forward && reading.status == S5_ECHO_OPEN_SPACE)) &&
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
#if ROBOT_VERBOSE_SENSOR_LOG
        tm_printf((UB*)"[S5-BATCH] invalid: usable=%d/%d; need=%d\n",
                  (int)count, (int)ROBOT_ECHO_SAMPLES,
                  (int)ROBOT_ECHO_REQUIRED_AGREEING);
#endif
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
#if ROBOT_VERBOSE_SENSOR_LOG
        tm_printf((UB*)"[S5-BATCH] invalid: usable echoes disagree\n");
#endif
        return false;
    }

    *out_cm = best_cm;
    return true;
}

static bool measure_stable(float *out_cm, bool forward)
{
    uint8_t attempt;

    if (out_cm == NULL)
    {
        return false;
    }

    *out_cm = 0.0f;
    for (attempt = 0u; attempt < ROBOT_ECHO_BATCH_ATTEMPTS; ++attempt)
    {
        if (measure_one_batch(out_cm, forward))
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

bool s5_measure_stable_cm(float *out_cm) { return measure_stable(out_cm, false); }
bool s5_measure_front_cm(float *out_cm) { return measure_stable(out_cm, true); }

/* Chatter protection: an invalid or close moving echo still stops the
 * motors immediately. A cluster of such events imposes a timed hold,
 * but unlike V2 it can recover without a reboot after the sensor proves
 * it is healthy again with consecutive multi-echo clear batches. */
void s5_guard_stop_history_init(S5GuardStopHistory *history)
{
    if (history != NULL)
    {
        history->window_start_ms = 0u;
        history->hold_start_ms = 0u;
        history->last_clear_ms = 0u;
        history->stop_count = 0u;
        history->clear_batches = 0u;
        history->locked = false;
    }
}

bool s5_guard_stop_history_record(S5GuardStopHistory *history,
                                  uint32_t now_ms)
{
    if (history == NULL)
    {
        return true; /* Missing safety state must never grant motion. */
    }
    history->clear_batches = 0u;
    if (history->locked)
    {
        return true;
    }
    if ((history->stop_count == 0u) ||
        ((uint32_t)(now_ms - history->window_start_ms) >
         S5_GUARD_STOP_WINDOW_MS))
    {
        history->window_start_ms = now_ms;
        history->stop_count = 1u;
    }
    else if (history->stop_count < S5_GUARD_STOP_LIMIT)
    {
        ++history->stop_count;
    }
    if (history->stop_count >= S5_GUARD_STOP_LIMIT)
    {
        history->locked = true;
        history->hold_start_ms = now_ms;
        history->clear_batches = 0u;
    }
    return history->locked;
}

void s5_guard_stop_history_unclear(S5GuardStopHistory *history)
{
    if (history != NULL)
    {
        history->clear_batches = 0u;
    }
}

bool s5_guard_stop_history_clear(S5GuardStopHistory *history,
                                 uint32_t now_ms)
{
    if (history == NULL || !history->locked)
    {
        return false;
    }
    if (history->clear_batches != 0u &&
        (uint32_t)(now_ms - history->last_clear_ms) >
            S5_GUARD_REARM_MAX_GAP_MS)
    {
        history->clear_batches = 0u;
    }
    history->last_clear_ms = now_ms;
    if (history->clear_batches < S5_GUARD_REARM_CLEAR_BATCHES)
    {
        ++history->clear_batches;
    }
    if ((uint32_t)(now_ms - history->hold_start_ms) <
            S5_GUARD_COOLDOWN_MS ||
        history->clear_batches < S5_GUARD_REARM_CLEAR_BATCHES)
    {
        return false;
    }
    /* A full hold and fresh clear batches: re-arm once. The next failure
     * still forces an immediate STOP and starts a new observation window. */
    s5_guard_stop_history_init(history);
    return true;
}
