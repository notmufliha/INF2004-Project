/** @file imu_odom.c
 * @brief Snapshot initialized B2 encoders at IMU acquisition, not queue
 * receipt.
 */
#include "imu_odom.h"
#include "s4_runtime.h"
#include "../subsystem2/encoder_control.h"

/* The kernel counter ABI must match the exact-width queue representation. */
typedef uint8_t imu_odom_counter_check_t[((sizeof(INT) == sizeof(int32_t)) &&
                                          (sizeof(UW) == sizeof(uint32_t)))
                                             ? 1
                                             : -1];

typedef struct
{
    SYSTIM start;
    SYSTIM encoder_end;
    ER     start_error;
    ER     encoder_error;
} imu_odom_clocks_t;

static uint16_t elapsed_ms (uint32_t start, uint32_t end);
static void     reset_metadata (imu_data_t * p_data);
static void     capture_counts (imu_data_t * p_data);
static void     record_clocks (imu_data_t *              p_data,
                               const imu_odom_clocks_t * p_clocks,
                               const SYSTIM * p_now, ER final_error);

/** @brief Compatibility capture for an already validated acquisition frame. */
void
imu_odom_capture (imu_data_t * p_data)
{
    if (NULL == p_data)
    {
        return;
    }

    reset_metadata(p_data);

    if (!p_data->b_valid)
    {
        p_data->odometry_reasons = IMU_ODOM_REASON_INPUT;
        return;
    }

    capture_counts(p_data);
}

/** @brief Pair bytes with encoder counts before angle or magnetic processing.
 */
void
imu_odom_pair (imu_data_t * p_data, uint32_t begin_ms, uint32_t end_ms,
               bool b_time_valid)
{
    if (NULL == p_data)
    {
        return;
    }

    reset_metadata(p_data);

    if ((!b_time_valid) || ((end_ms - begin_ms) > UINT32_MAX / 2U))
    {
        p_data->odometry_reasons = IMU_ODOM_REASON_CLOCK;
        return;
    }

    p_data->timestamp_ms = begin_ms;
    capture_counts(p_data);
    /* Read-completion time estimates availability, not exact ADC conversion.
     * Validity still bounds the entire burst-plus-encoder window to 10 ms.
     */
    p_data->timestamp_ms = end_ms;
}

/** @brief Clear pairing metadata while retaining measured sensor-read timings.
 */
static void
reset_metadata (imu_data_t * p_data)
{
    p_data->b_odometry_valid = false;
    p_data->motion           = (imu_motion_t){0};
    p_data->odometry_reasons = 0U;
    p_data->odometry_time_valid &= IMU_READ_TIME_ACCEL | IMU_READ_TIME_MAG |
                                  IMU_READ_MAG_DEFERRED;
    p_data->odometry_age_ms     = 0U;
    p_data->odometry_encoder_ms = 0U;
    p_data->odometry_context_ms = 0U;
}

/** @brief Read bounded encoder/context snapshots within the acquisition span.
 */
static void
capture_counts (imu_data_t * p_data)
{
    INT               left_count   = 0;
    INT               right_count  = 0;
    UW                left_before  = 0U;
    UW                right_before = 0U;
    UW                left_after   = 0U;
    UW                right_after  = 0U;
    SYSTIM            now          = {0};
    imu_odom_clocks_t clocks       = {0};

    clocks.start_error = tk_get_tim(&clocks.start);
    encoder_get_invalid_counts(&left_before, &right_before);
    encoder_get_counts(&left_count, &right_count);
    encoder_get_invalid_counts(&left_after, &right_after);
    clocks.encoder_error = tk_get_tim(&clocks.encoder_end);

    if (left_before != left_after)
    {
        p_data->odometry_reasons |= IMU_ODOM_REASON_LEFT;
    }

    if (right_before != right_after)
    {
        p_data->odometry_reasons |= IMU_ODOM_REASON_RIGHT;
    }

    /* Optional B2 context must not suppress IMU readiness or raw count capture.
     * Its validity flag records a missing/stale snapshot independently. */
    (void)imu_motion_capture(&p_data->motion);

    ER error = tk_get_tim(&now);

    record_clocks(p_data, &clocks, &now, error);

    if (error < E_OK)
    {
        return;
    }

    /* Verified 32-bit kernel signed/unsigned counts fit these fields exactly.
     */
    p_data->encoder_left         = (int32_t)left_count;
    p_data->encoder_right        = (int32_t)right_count;
    p_data->encoder_left_errors  = (uint32_t)left_after;
    p_data->encoder_right_errors = (uint32_t)right_after;
    p_data->b_odometry_valid =
        (left_before == left_after) && (right_before == right_after) &&
        (now.lo - p_data->timestamp_ms <= S4_IMU_PERIOD_MS);
    p_data->timestamp_ms = now.lo;
}

/** @brief Preserve wrap-safe elapsed time in a bounded diagnostic field. */
static uint16_t
elapsed_ms (uint32_t start, uint32_t end)
{
    const uint32_t elapsed = end - start;

    /* The checked result fits uint16_t; larger values retain a lower bound. */
    return (elapsed > UINT16_MAX) ? UINT16_MAX : (uint16_t)elapsed;
}

/** @brief Distinguish full acquisition age from encoder/context overhead. */
static void
record_clocks (imu_data_t * p_data, const imu_odom_clocks_t * p_clocks,
               const SYSTIM * p_now, ER final_error)
{
    if ((E_OK != p_clocks->start_error) || (E_OK != p_clocks->encoder_error))
    {
        /* Failed diagnostic probes do not change the existing validity rule. */
        p_data->odometry_reasons |= IMU_ODOM_REASON_PROBE;
    }

    if ((E_OK == p_clocks->start_error) && (E_OK == p_clocks->encoder_error))
    {
        p_data->odometry_encoder_ms =
            elapsed_ms(p_clocks->start.lo, p_clocks->encoder_end.lo);
        p_data->odometry_time_valid |= IMU_ODOM_TIME_ENCODER;
    }

    if (E_OK != final_error)
    {
        p_data->odometry_reasons |= IMU_ODOM_REASON_CLOCK;
        return;
    }

    p_data->odometry_age_ms = elapsed_ms(p_data->timestamp_ms, p_now->lo);
    p_data->odometry_time_valid |= IMU_ODOM_TIME_AGE;

    if ((p_now->lo - p_data->timestamp_ms) > S4_IMU_PERIOD_MS)
    {
        p_data->odometry_reasons |= IMU_ODOM_REASON_AGE;
    }

    if (E_OK == p_clocks->encoder_error)
    {
        p_data->odometry_context_ms =
            elapsed_ms(p_clocks->encoder_end.lo, p_now->lo);
        p_data->odometry_time_valid |= IMU_ODOM_TIME_CONTEXT;
    }
}

/*** end of file ***/

