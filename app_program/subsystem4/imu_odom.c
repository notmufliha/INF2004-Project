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

/** @brief Use the single B2 geometry/calibration definition for wheel units. */
bool
imu_odom_init (imu_height_t * p_height)
{
    imu_height_config_t config;

    imu_height_default_config(&config);
    config.b_require_motion = true;

    if ((ENC_LEFT_COUNTS_PER_REV <= 0) || (ENC_RIGHT_COUNTS_PER_REV <= 0))
    {
        return false;
    }

    /* Current counts/revolution are positive small integers, exact in binary32.
     * These provisional B2 constants require measured wheel calibration.
     */
    config.left_cm_per_count  = (ENC_WHEEL_DIAMETER_CM * 3.14159265f) /
                                (float32_t)ENC_LEFT_COUNTS_PER_REV;
    config.right_cm_per_count = (ENC_WHEEL_DIAMETER_CM * 3.14159265f) /
                                (float32_t)ENC_RIGHT_COUNTS_PER_REV;
    return imu_height_init(p_height, &config);
}

/** @brief Pair real counts with the reading and reject slow/errorful capture.
 */
void
imu_odom_capture (imu_data_t * p_data)
{
    INT    left_count   = 0;
    INT    right_count  = 0;
    UW     left_before  = 0U;
    UW     right_before = 0U;
    UW     left_after   = 0U;
    UW     right_after  = 0U;
    SYSTIM now;

    if (NULL == p_data)
    {
        return;
    }

    p_data->b_odometry_valid = false;
    p_data->motion           = (imu_motion_t){0};

    if (!p_data->b_valid)
    {
        return;
    }

    encoder_get_invalid_counts(&left_before, &right_before);
    encoder_get_counts(&left_count, &right_count);
    encoder_get_invalid_counts(&left_after, &right_after);
    /* Optional B2 context must not suppress IMU readiness or raw count capture.
     * Its validity flag records a missing/stale snapshot independently. */
    (void)imu_motion_capture(&p_data->motion);

    if (tk_get_tim(&now) < E_OK)
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

/*** end of file ***/

