/** @file imu_motion.c
 * @brief Consume B2 motion snapshots without changing B2 state or ownership.
 */
#include "imu_motion.h"
#include "../subsystem2/motion_command.h"

#define MOTION_SPEED_LIMIT_CPS 100000

/* Kernel speeds use the same signed width as the B4 acquisition payload. */
typedef uint8_t
    imu_motion_abi_check_t[((sizeof(INT) == sizeof(int32_t)) &&
                            (sizeof(motion_cmd_type_t) <= sizeof(uint32_t)) &&
                            (sizeof(motion_status_t) <= sizeof(uint32_t)) &&
                            (sizeof(motion_direction_t) <= sizeof(uint32_t)) &&
                            (sizeof(motion_fault_t) <= sizeof(uint32_t)))
                               ? 1
                               : -1];

static bool snapshot_is_valid (const motion_status_snapshot_t * p_snapshot,
                               uint32_t                         now_ms);
static void map_snapshot (const motion_status_snapshot_t * p_snapshot,
                          imu_motion_t *                   p_motion);

/** @brief Read a coherent B2 snapshot and reject invalid or stale results. */
bool
imu_motion_capture (imu_motion_t * p_motion)
{
    motion_status_snapshot_t snapshot = {0};
    SYSTIM                   now;

    if (NULL == p_motion)
    {
        return false;
    }

    *p_motion = (imu_motion_t){0};
    if ((motion_get_status(&snapshot) != E_OK) || (tk_get_tim(&now) != E_OK))
    {
        return false;
    }

    if (!snapshot_is_valid(&snapshot, now.lo))
    {
        return false;
    }

    map_snapshot(&snapshot, p_motion);
    return p_motion->b_valid;
}

/** @brief Validate B2's timestamp, declared ranges and measurement fields. */
static bool
snapshot_is_valid (const motion_status_snapshot_t * p_snapshot, uint32_t now_ms)
{
    /* Unsigned conversion is defined modulo 2^32: negative enum values become
     * large and are rejected. Valid enum values and their maxima fit exactly.
     */
    if (((uint32_t)p_snapshot->command_type > (uint32_t)MOTION_CMD_MOVE) ||
        ((uint32_t)p_snapshot->status > (uint32_t)MOTION_STATUS_FAULT) ||
        ((uint32_t)p_snapshot->direction > (uint32_t)MOTION_DIR_REVERSE) ||
        ((uint32_t)p_snapshot->fault > (uint32_t)MOTION_FAULT_ENCODER) ||
        (0U == p_snapshot->control_interval_ms) ||
        (p_snapshot->control_interval_ms > IMU_MOTION_MAX_AGE_MS) ||
        ((now_ms - p_snapshot->sample_time_ms) > IMU_MOTION_MAX_AGE_MS) ||
        (p_snapshot->left_speed_cps < -MOTION_SPEED_LIMIT_CPS) ||
        (p_snapshot->left_speed_cps > MOTION_SPEED_LIMIT_CPS) ||
        (p_snapshot->right_speed_cps < -MOTION_SPEED_LIMIT_CPS) ||
        (p_snapshot->right_speed_cps > MOTION_SPEED_LIMIT_CPS) ||
        (!imu_numeric_is_finite(p_snapshot->left_distance_mm)) ||
        (!imu_numeric_is_finite(p_snapshot->right_distance_mm)) ||
        (!imu_numeric_is_finite(p_snapshot->total_distance_mm)) ||
        (p_snapshot->total_distance_mm < 0.0f))
    {
        return false;
    }

    return true;
}

/** @brief Preserve measured values and map movement context into B4 flags. */
static void
map_snapshot (const motion_status_snapshot_t * p_snapshot,
              imu_motion_t *                   p_motion)
{
    p_motion->timestamp_ms = p_snapshot->sample_time_ms;
    p_motion->command_id   = p_snapshot->command_id;
    /* The verified signed 32-bit kernel speeds fit int32_t exactly. */
    p_motion->left_speed_cps  = (int32_t)p_snapshot->left_speed_cps;
    p_motion->right_speed_cps = (int32_t)p_snapshot->right_speed_cps;
    p_motion->left_travel_cm  = p_snapshot->left_distance_mm / 10.0f;
    p_motion->right_travel_cm = p_snapshot->right_distance_mm / 10.0f;
    p_motion->total_travel_cm = p_snapshot->total_distance_mm / 10.0f;
    p_motion->b_running       = MOTION_STATUS_RUNNING == p_snapshot->status;
    p_motion->b_fault         = (MOTION_FAULT_NONE != p_snapshot->fault) ||
                                (MOTION_STATUS_FAULT == p_snapshot->status) ||
                                (MOTION_STATUS_TIMEOUT == p_snapshot->status);
    p_motion->b_stopped       = p_snapshot->physically_stopped;
    p_motion->b_turning =
        (p_motion->b_running) && (MOTION_CMD_TURN == p_snapshot->command_type);

    const bool b_translating = (p_motion->b_running) &&
                               (!p_motion->b_turning) &&
                               (!p_motion->b_stopped) && (!p_motion->b_fault) &&
                               ((MOTION_CMD_MOVE == p_snapshot->command_type) ||
                                (MOTION_CMD_DRIVE == p_snapshot->command_type));

    p_motion->b_forward =
        b_translating && (MOTION_DIR_FORWARD == p_snapshot->direction) &&
        (p_motion->left_speed_cps >= 0) && (p_motion->right_speed_cps >= 0);
    p_motion->b_reverse =
        b_translating && (MOTION_DIR_REVERSE == p_snapshot->direction);
    p_motion->b_valid = (imu_numeric_is_finite(p_motion->left_travel_cm)) &&
                        (imu_numeric_is_finite(p_motion->right_travel_cm)) &&
                        (imu_numeric_is_finite(p_motion->total_travel_cm));

    if (!p_motion->b_valid)
    {
        *p_motion = (imu_motion_t){0};
    }
}

/*** end of file ***/

