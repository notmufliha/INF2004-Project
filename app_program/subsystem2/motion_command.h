/** @file motion_command.h
 * @brief Subsystem 2 motion command interface.
 */

#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

/*
 * Buddy 2 - Motion Command Interface
 *
 * Initial command interface for integration
 * with Buddy 1 Vehicle Controller.
 */

typedef enum
{
    MOTION_CMD_STOP = 0,
    MOTION_CMD_DRIVE,
    MOTION_CMD_TURN,
    MOTION_CMD_MOVE
} motion_cmd_type_t;

typedef enum
{
    MOTION_DIR_STOP = 0,
    MOTION_DIR_FORWARD,
    MOTION_DIR_REVERSE
} motion_direction_t;

typedef enum
{
    MOTION_TURN_LEFT = 0,
    MOTION_TURN_RIGHT
} motion_turn_direction_t;
/*
 * Motion command sent by Vehicle Controller.
 *
 * Target speed is measured in encoder
 * counts per second.
 *
 * Speed and distance use left-wheel reference counts.
 * MOVE is distance limited; TURN is an in-place encoder-based turn.
 */

typedef struct
{
    motion_cmd_type_t type;
    motion_direction_t direction;
    INT target_speed_cps;
    INT target_distance_counts;
    motion_turn_direction_t turn_direction;
    INT target_angle_deg;
    /* DRIVE only: -1000 full left, 0 straight, +1000 full right.
     * The inner wheel stops at full steering; neither wheel reverses.
     * Positive steering increases left wheel speed, including in reverse
     * (therefore reverse yaw changes sign). Zero-initialize commands.
     */
    INT steering_permille;
    /* MOVE/TURN only: 0 derives a bounded deadline from distance/speed.
     * Explicit deadlines must be 100..60000 ms.
     */
    uint32_t timeout_ms;
    /*
     * Unique identifier assigned by the command interface.
     * The same identifier is returned in motion status.
     */
    uint32_t command_id;
    /*
     * Safety STOP generation observed by Buddy 1
     * when authorizing this motion command.
     *
     * Required for DRIVE, MOVE and TURN.
     * Ignored for STOP.
     */
    uint32_t safety_generation;
} motion_command_t;

/*
 * Public motion execution status.
 *
 * Allows the Vehicle Controller to distinguish
 * successful completion from cancellation or failure.
 */
typedef enum
{
    MOTION_STATUS_IDLE = 0,
    MOTION_STATUS_RUNNING,
    MOTION_STATUS_COMPLETED,
    MOTION_STATUS_CANCELLED,
    MOTION_STATUS_TIMEOUT,
    MOTION_STATUS_FAULT
} motion_status_t;

typedef enum
{
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_SAFETY,
    MOTION_FAULT_WATCHDOG,
    MOTION_FAULT_DURATION,
    MOTION_FAULT_STALL,
    MOTION_FAULT_STOPPING,
    MOTION_FAULT_TIMING,
    MOTION_FAULT_COMMAND,
    MOTION_FAULT_ENCODER
} motion_fault_t;

/* The most recent eight terminal results survive later submissions.
 * Older results are evicted FIFO; query returns E_OBJ if unavailable.
 */
#define MOTION_RESULT_HISTORY 8u

/*
 * Snapshot of Buddy 2's latest motion state.
 */
typedef struct
{
    motion_cmd_type_t command_type;
    motion_status_t status;
    motion_fault_t fault;
    motion_direction_t direction;
    motion_turn_direction_t turn_direction;

    /*
     * Identifies the command associated with this status.
     */
    uint32_t command_id;

    INT left_encoder_counts;
    INT right_encoder_counts;

    INT left_speed_cps;
    INT right_speed_cps;

    /* Signed travel since command start; total is absolute wheel travel. */
    float left_distance_mm;
    float right_distance_mm;
    float total_distance_mm;
    bool physically_stopped;
    uint32_t sample_time_ms;
    uint32_t control_interval_ms;
    uint32_t maximum_interval_ms;

    uint32_t safety_generation;
} motion_status_snapshot_t;

/*
 * Public command interface.
 *
 * Submissions poll the one-slot queue and return E_TMOUT/E_OBJ when busy.
 */

ER motion_send_command(const motion_command_t *p_command);
/* VC SPEED mapping; zero percent means submit STOP, not a zero-speed DRIVE. */
ER motion_speed_from_percent(INT percent, INT *speed_cps);

/* Task-context convenience APIs. All still require Buddy 5 clearance. */
ER motion_move_cm(float distance_cm, INT speed_cps,
    motion_direction_t direction, uint32_t *command_id);
ER motion_turn_degrees(INT angle, INT speed_cps,
    motion_turn_direction_t direction, uint32_t *command_id);

/*
 * Queue a motion command and return its assigned ID.
 *
 * On success, p_command_id receives the ID used
 * to correlate the command with its execution status.
 */
ER motion_send_command_with_id(
    const motion_command_t *p_command,
    uint32_t *p_command_id
);

ER motion_command_init(void);

ER motion_receive_command(motion_command_t *p_command);

/*
 * Read the latest motion status.
 *
 * Uses a short interrupt/SMP lock; no network or semaphore waits.
 */
ER motion_get_status(motion_status_snapshot_t *p_status);

ER motion_get_result(uint32_t command_id,
    motion_status_snapshot_t *p_status);

/*
 * Publish a new motion-status snapshot.
 * Called by Buddy 2's Motion Control Task.
 */
ER motion_update_status(const motion_status_snapshot_t *p_status);

#endif
