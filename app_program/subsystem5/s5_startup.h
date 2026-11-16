#ifndef S5_STARTUP_H
#define S5_STARTUP_H
#include <stdbool.h>
#include "../robot_config.h"
/* Startup echo failures are recoverable while motors remain stopped. */
static inline bool s5_startup_ready(unsigned *hits, bool valid)
{
    if (*hits >= ROBOT_CLEAR_CONFIRM) return true;
    if (!valid) *hits = 0u;
    else ++*hits;
    return *hits >= ROBOT_CLEAR_CONFIRM;
}
#endif
