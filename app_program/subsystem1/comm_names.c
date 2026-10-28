/*
 *	comm_names.c
 *	Subsystem 1 -- printable names for the enums in subsystem1.h.
 *	Console output only: the MQTT frames carry numbers, and the laptop
 *	decoder (tools/s1_wire.py) holds the names.  The tables are const, so
 *	they live in flash and cost no RAM.  Every function checks its range
 *	and returns "?" instead of reading past the table.
 */

#include "s1_private.h"

/* One unsigned compare also rejects negative values */
#define NAME_OF(table, idx, count) \
	(((UINT)(idx) < (UINT)(count)) ? (table)[(idx)] : "?")

LOCAL const char *const comm_state_names[COMM_STATE_COUNT] = {
	"INITIALISING", "CONNECTING_WIFI", "CONNECTING_MQTT", "CONNECTED",
	"RECONNECTING", "OFFLINE", "DISABLED"
};

LOCAL const char *const nav_names[NAV_COUNT] = {
	"NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"
};

LOCAL const char *const avoid_names[AVOID_COUNT] = {
	"NONE", "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE"
};

LOCAL const char *const rcmd_names[RCMD_COUNT] = {
	"NONE", "START", "STOP", "ESTOP", "PAUSE", "RESUME",
	"MODE", "SPEED", "RESET_PEAK"
};

EXPORT const char *comm_state_name(COMM_STATE s)
{
	return NAME_OF(comm_state_names, s, COMM_STATE_COUNT);
}

EXPORT const char *nav_cmd_name(NAV_CMD n)
{
	return NAME_OF(nav_names, n, NAV_COUNT);
}

EXPORT const char *avoid_action_name(AVOID_ACTION a)
{
	return NAME_OF(avoid_names, a, AVOID_COUNT);
}

EXPORT const char *remote_cmd_name(REMOTE_CMD_TYPE c)
{
	return NAME_OF(rcmd_names, c, RCMD_COUNT);
}
