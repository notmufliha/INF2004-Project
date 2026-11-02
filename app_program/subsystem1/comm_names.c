/** @file comm_names.c
 * @brief Printable names for Subsystem 1's enums (see comm_names.h).
 * The tables are const, so they live in flash and cost no RAM.
 */

#include "comm_names.h"

static const char * const g_link_names[COMM_STATE_COUNT] =
{
    "INITIALISING", "CONNECTING_WIFI", "CONNECTING_MQTT", "CONNECTED",
    "RECONNECTING", "OFFLINE", "DISABLED"
};

static const char * const g_nav_names[NAV_COUNT] =
{
    "NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"
};

static const char * const g_avoid_names[AVOID_COUNT] =
{
    "NONE", "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE"
};

static const char * const g_remote_names[RCMD_COUNT] =
{
    "NONE", "START", "STOP", "ESTOP", "PAUSE", "RESUME",
    "MODE", "SPEED", "RESET_PEAK"
};

static const char * name_of (const char * const * pp_table, uint32_t index,
                             uint32_t count);

/** @brief Name of a link state, e.g. "CONNECTED". */
const char *
comm_names_link (comm_link_state_t state)
{
    return (name_of(g_link_names, (uint32_t)state, COMM_STATE_COUNT));
}

/** @brief Name of a barcode navigation command, e.g. "LEFT". */
const char *
comm_names_nav (comm_nav_cmd_t nav)
{
    return (name_of(g_nav_names, (uint32_t)nav, NAV_COUNT));
}

/** @brief Name of an obstacle avoidance action, e.g. "STOP". */
const char *
comm_names_avoid (comm_avoid_action_t action)
{
    return (name_of(g_avoid_names, (uint32_t)action, AVOID_COUNT));
}

/** @brief Name of a remote command, e.g. "ESTOP". */
const char *
comm_names_remote (comm_remote_cmd_type_t type)
{
    return (name_of(g_remote_names, (uint32_t)type, RCMD_COUNT));
}

/** @brief Table entry @p index, or "?" when it is out of range.
 *
 * The callers convert the enum to uint32_t first, so a negative value
 * (possible for an enum) becomes a large number and is rejected here.
 */
static const char *
name_of (const char * const * pp_table, uint32_t index, uint32_t count)
{
    const char * p_name = "?";

    if (index < count)
    {
        p_name = pp_table[index];
    }

    return (p_name);
}

/*** end of file ***/

