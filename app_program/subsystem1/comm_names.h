/** @file comm_names.h
 * @brief Printable names for Subsystem 1's enums, for the console only.
 *
 * The MQTT frames carry numbers; the laptop decoder (tools/s1_wire.py)
 * and dashboard hold their own copies of these names.  Every function
 * checks its range and returns "?" instead of reading past a table, so
 * the result is never NULL.
 */

#ifndef COMM_NAMES_H
#define COMM_NAMES_H

#include "comm.h"

const char * comm_names_link (comm_link_state_t state);
const char * comm_names_nav (comm_nav_cmd_t nav);
const char * comm_names_avoid (comm_avoid_action_t action);
const char * comm_names_remote (comm_remote_cmd_type_t type);

#endif /* COMM_NAMES_H */

/*** end of file ***/

