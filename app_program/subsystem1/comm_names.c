/*
 *	comm_names.c
 *	Subsystem 1 -- printable names for the enums in subsystem1.h.
 *	Used in the JSON messages and on the console.  Every function checks
 *	its range and returns "?" instead of reading past the table.
 */

#include "s1_private.h"

/* One unsigned compare also rejects negative values */
#define NAME_OF(table, idx, count) \
	(((UINT)(idx) < (UINT)(count)) ? (table)[(idx)] : "?")

LOCAL const char *const comm_state_names[COMM_STATE_COUNT] = {
	"INITIALISING", "CONNECTING_WIFI", "CONNECTING_MQTT", "CONNECTED",
	"RECONNECTING", "OFFLINE", "DISABLED"
};

LOCAL const char *const robot_state_names[ROBOT_STATE_COUNT] = {
	"INIT", "IDLE", "LINE_FOLLOW", "NAV_COMMAND", "HUMP",
	"OBSTACLE_SCAN", "OBSTACLE_AVOID", "LINE_RECOVERY", "SAFE_STOP"
};

LOCAL const char *const nav_names[NAV_COUNT] = {
	"NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"
};

LOCAL const char *const imu_names[IMU_EV_COUNT] = {
	"STATIONARY", "MOVING", "ACCELERATING", "TURNING",
	"CLIMBING", "DESCENDING", "IMPACT"
};

LOCAL const char *const avoid_names[AVOID_COUNT] = {
	"NONE", "STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE"
};

LOCAL const char *const rcmd_names[RCMD_COUNT] = {
	"NONE", "START", "STOP", "ESTOP", "PAUSE", "RESUME",
	"MODE", "SPEED", "RESET_PEAK"
};

LOCAL const char *const mode_names[MODE_COUNT] = {
	"AUTO", "LINE", "TEST"
};

EXPORT const char *comm_state_name(COMM_STATE s)
{
	return NAME_OF(comm_state_names, s, COMM_STATE_COUNT);
}

EXPORT const char *robot_state_name(ROBOT_STATE s)
{
	return NAME_OF(robot_state_names, s, ROBOT_STATE_COUNT);
}

EXPORT const char *nav_cmd_name(NAV_CMD n)
{
	return NAME_OF(nav_names, n, NAV_COUNT);
}

EXPORT const char *imu_event_name(IMU_EVENT e)
{
	return NAME_OF(imu_names, e, IMU_EV_COUNT);
}

EXPORT const char *avoid_action_name(AVOID_ACTION a)
{
	return NAME_OF(avoid_names, a, AVOID_COUNT);
}

EXPORT const char *remote_cmd_name(REMOTE_CMD_TYPE c)
{
	return NAME_OF(rcmd_names, c, RCMD_COUNT);
}

EXPORT const char *robot_mode_name(ROBOT_MODE m)
{
	return NAME_OF(mode_names, m, MODE_COUNT);
}
