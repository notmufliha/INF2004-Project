#ifndef S5_BRIDGE_H
#define S5_BRIDGE_H
#include <stdbool.h>
#include "s5_logic.h"

/* Contract to Buddy 2 (motors) and Buddy 3 (line tracking).
 * Replace weak safe defaults in s5_bridge.c with real team functions.
 * NEVER acknowledge a movement until the movement has physically finished.
 * Bridge requests must be non-blocking. Ownership of motors is exclusive
 * during stop/avoid/reverse/recovery. */
bool s5_bridge_ready(void);
bool s5_bridge_request(S5Command command, const S5Profile *profile);
bool s5_bridge_stopped(void);
bool s5_bridge_motion_complete(void);
bool s5_bridge_line_found(void);
void s5_bridge_emergency_stop(void);
#endif
