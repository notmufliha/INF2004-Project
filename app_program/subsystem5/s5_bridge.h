#ifndef S5_BRIDGE_H
#define S5_BRIDGE_H

#include <stdbool.h>
#include "s5_logic.h"

/*
 * Buddy 5 -> mission controller / Buddy 2 / Buddy 3 integration contract.
 *
 * The mission controller must install callbacks during initialization,
 * BEFORE starting the B5 task. B5 does not own the motor command queue.
 * The callbacks must be non-blocking and must never report completion
 * until the corresponding encoder-verified motion has finished.
 *
 * Without registration the bridge is NOT READY. subsystem5_init() installs
 * the B5 navigation adapter only in the autonomous robot profile. The
 * guarded forward/STOP profile leaves it unregistered.
 */
typedef struct {
    /* True only when B2, B3, and exclusive VC motor ownership are ready. */
    bool (*ready)(void);
    /* VC must arbitrate STOP, REVERSE, GO_LEFT/RIGHT, FIND_LINE, RESUME.
     * Reject uncalibrated or unsafe manoeuvres, including reverse without
     * independently verified rear clearance. */
    bool (*request)(S5Command command, const S5Profile *profile);
    /* True only after the current B2 command's ID is COMPLETED, not merely
     * when its queue accepts the command or the wheels happen to stop. */
    bool (*motion_complete)(void);
    /* True only for a fresh, debounced B3 line detection after recovery. */
    bool (*line_found)(void);
    /* Must cancel VC's queued/active movement and leave B2 stopped. */
    void (*emergency_stop)(void);
} S5BridgeDriver;

/* Install exactly once during single-threaded startup, before B5 starts.
 * Returns false for NULL/incomplete drivers or repeated registration. */
bool s5_bridge_install(const S5BridgeDriver *driver);

bool s5_bridge_ready(void);
bool s5_bridge_request(S5Command command, const S5Profile *profile);
bool s5_bridge_stopped(void);
bool s5_bridge_motion_complete(void);
bool s5_bridge_line_found(void);
void s5_bridge_emergency_stop(void);

#endif /* S5_BRIDGE_H */
