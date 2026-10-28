/** @file motion_command.h
 * @brief Subsystem 2 motion command interface.
 */

#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include <stdint.h>
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
 * Target distance and angle are reserved
 * for the next implementation stage.
 */

typedef struct
{
    motion_cmd_type_t type;
    motion_direction_t direction;
    INT target_speed_cps;
    INT target_distance_counts;
    motion_turn_direction_t turn_direction;
    INT target_angle_deg;
    /*
     * Unique identifier assigned by the command sender.
     * The same identifier is returned in motion status.
     */
    uint32_t command_id;
    /*
     * Safety STOP generation observed by Buddy 1
     * when authorizing this motion command.
     *
     * Required for DRIVE.
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

/*
 * Snapshot of Buddy 2's latest motion state.
 */
typedef struct
{
    motion_cmd_type_t command_type;
    motion_status_t status;

    /*
     * Identifies the command associated with this status.
     */
    uint32_t command_id;

    INT left_encoder_counts;
    INT right_encoder_counts;

    INT left_speed_cps;
    INT right_speed_cps;

    uint32_t safety_generation;
} motion_status_snapshot_t;

/*
 * Public command interface.
 *
 * This will eventually send commands to
 * the Motion Control Task.
 */

ER motion_send_command(const motion_command_t *p_command);

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
 * Implementation will provide a synchronized snapshot
 * for other tasks.
 */
ER motion_get_status(motion_status_snapshot_t *p_status);

/*
 * Publish a new motion-status snapshot.
 * Called by Buddy 2's Motion Control Task.
 */
ER motion_update_status(const motion_status_snapshot_t *p_status);

#endif
