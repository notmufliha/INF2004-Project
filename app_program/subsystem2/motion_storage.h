/** @file motion_storage.h
 * @brief Native packed storage for commands already validated by B2.
 */
#ifndef MOTION_STORAGE_H
#define MOTION_STORAGE_H

#include "motion_command.h"

/* Validated task-local/native storage, never a wire format. */
typedef struct
{
    UINT     type : 2;
    UINT     direction : 2;
    UINT     turn_direction : 1;
    UINT     target_speed_cps : 13;
    UINT     target_angle_deg : 9;
    INT      steering_permille : 11;
    UINT     timeout_ms : 16;
    UINT     run_ms : 16;
    UINT     target_distance_counts : 31;
    uint32_t command_id;
    uint32_t safety_generation;
} motion_stored_command_t;

static inline motion_stored_command_t
motion_pack_command (const motion_command_t * command)
{
    motion_stored_command_t packed     = {0};
    motion_command_t        normalized = *command;
    if (normalized.type == MOTION_CMD_STOP)
    {
        normalized                   = (motion_command_t){0};
        normalized.command_id        = command->command_id;
        normalized.safety_generation = command->safety_generation;
    }
    else
    {
        if (normalized.type != MOTION_CMD_TURN)
        {
            normalized.turn_direction   = MOTION_TURN_LEFT;
            normalized.target_angle_deg = 0;
        }
        else
        {
            normalized.direction = MOTION_DIR_STOP;
        }
        if (normalized.type != MOTION_CMD_MOVE)
        {
            normalized.target_distance_counts = 0;
        }
    }
    packed.type                   = normalized.type;
    packed.direction              = normalized.direction;
    packed.turn_direction         = normalized.turn_direction;
    packed.target_speed_cps       = normalized.target_speed_cps;
    packed.target_angle_deg       = normalized.target_angle_deg;
    packed.steering_permille      = normalized.steering_permille;
    packed.timeout_ms             = normalized.timeout_ms;
    packed.run_ms                 = normalized.run_ms;
    packed.target_distance_counts = normalized.target_distance_counts;
    packed.command_id             = normalized.command_id;
    packed.safety_generation      = normalized.safety_generation;
    return packed;
}

#endif

/*** end of file ***/

