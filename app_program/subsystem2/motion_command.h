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
    MOTION_CMD_TURN
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
     * Safety STOP generation observed by Buddy 1
     * when authorizing this motion command.
     *
     * Required for DRIVE.
     * Ignored for STOP.
     */
    uint32_t safety_generation;
} motion_command_t;
/*
 * Public command interface.
 *
 * This will eventually send commands to
 * the Motion Control Task.
 */

ER motion_send_command(const motion_command_t *p_command);

ER motion_command_init(void);

ER motion_receive_command(motion_command_t *p_command);
#endif
