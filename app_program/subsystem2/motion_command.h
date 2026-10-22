
#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include "../app.h"

/*
 * Buddy 2 - Motion Command Interface
 *
 * Initial command interface for integration
 * with Buddy 1 Vehicle Controller.
 */

typedef enum {
    MOTION_CMD_STOP = 0,
    MOTION_CMD_DRIVE,
    MOTION_CMD_TURN
} MOTION_CMD_TYPE;

typedef enum {
    MOTION_DIR_STOP = 0,
    MOTION_DIR_FORWARD,
    MOTION_DIR_REVERSE
} MOTION_DIRECTION;

typedef enum {
    MOTION_TURN_LEFT = 0,
    MOTION_TURN_RIGHT
} MOTION_TURN_DIRECTION;

/*
 * Motion command sent by Vehicle Controller.
 *
 * Target speed is measured in encoder
 * counts per second.
 *
 * Target distance and angle are reserved
 * for the next implementation stage.
 */
typedef struct {
    MOTION_CMD_TYPE type;

    MOTION_DIRECTION direction;

    INT target_speed_cps;

    INT target_distance_counts;

    MOTION_TURN_DIRECTION turn_direction;

    INT target_angle_deg;
} MOTION_COMMAND;

/*
 * Public command interface.
 *
 * This will eventually send commands to
 * the Motion Control Task.
 */
ER motion_send_command(const MOTION_COMMAND *command);

ER motion_command_init(void);

ER motion_receive_command(MOTION_COMMAND *command);

#endif
