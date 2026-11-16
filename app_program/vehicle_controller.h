#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "subsystem1/subsystem1.h"
#include "subsystem2/motion_command.h"
#include <stdbool.h>

/* Temporary steering diagnosis: set to 0 and rebuild to remove logging. */
#ifndef VC_STEERING_DIAGNOSTICS
#define VC_STEERING_DIAGNOSTICS 1
#endif
#define VC_STEERING_LOG_MS 250u

#define VC_PERIOD_MS 20u
#define VC_LINE_MAX_AGE_MS 100u
#define VC_IMU_MAX_AGE_MS 1500u /* B4 publishes a quiet snapshot every second. */
#define VC_MOTION_MAX_AGE_MS 100u
#define VC_RECOVERY_MS 3000u
#define VC_LINE_SEARCH_SWITCH_MS 600u /* first sweep; each later sweep is 600 ms longer */
#define VC_LINE_SEARCH_MS 8000u /* then hold, still armed, until the line returns */
#define VC_LINE_SEARCH_STEER 250
/* Line just lost: first turn hard toward the side it was last seen (what
 * Pololu's saturated position + PD does) for this long, then sweep. */
#define VC_LINE_REACQUIRE_MS 1500u
#define VC_JUNCTION_CONFIRM_SAMPLES 3u
/* Curve speed: at full steer the base speed drops to VC_CURVE_MIN_PERMILLE.
 * The slowdown follows the hardest recent steering and relaxes by
 * VC_CURVE_DECAY_PER_TICK each 20 ms tick, so the car stays slow through
 * the whole curve instead of speeding up between corrections. */
#define VC_CURVE_MIN_PERMILLE 600
#define VC_CURVE_DECAY_PER_TICK 40
/* Smoothness: steering may change by at most this much per 20 ms tick
 * (0 to full 800 in about 0.1 s), and speed may rise by at most
 * VC_SPEED_RISE_PER_TICK counts/s per tick.  Slowing down is immediate. */
#define VC_STEER_SLEW_PER_TICK 150
/* Easing off a correction: full 800 to straight in 3 ticks (60 ms). */
#define VC_STEER_RELEASE_PER_TICK 300
/* Following never pivots: on a hard turn the inner wheel stays at
 * VC_MIN_WHEEL_CPS and the outer runs at least this many times faster. */
#define VC_FOLLOW_MIN_OUTER_RATIO 3
#define VC_SPEED_RISE_PER_TICK 100
/* CALIBRATE: slowest wheel speed (counts/s) the loaded motors hold without
 * stalling; about 25 mm/s.  A curve's inner wheel is never asked to run
 * between 0 and this: a little below it is raised to it, far below it is
 * set to 0 so the car pivots around that wheel (a wheel commanded to 0 is
 * never reported as stalled). */
#define VC_MIN_WHEEL_CPS 300
#define VC_JUNCTION_RELEASE_SAMPLES 3u
/* Clear-path mode, line lost on a curve: keep the last following steer
 * (if at least MIN_STEER) for this long before driving straight. */
#define VC_CURVE_REACQUIRE_MS 500u
#define VC_CURVE_REACQUIRE_MIN_STEER 200
/* Junction = a crossing tape (18 mm wide, perpendicular to the line).
 * Both line sensors sit on the 18 mm line whenever the car is centred, so
 * B3's JUNCTION flag alone is set all the time and a barcode's turn ran at
 * once.  The barcode sensor (GP28) beside the line reads floor except over
 * barcode bars (<= ~9 mm) and a crossing tape (18 mm): a junction is GP28
 * black for at least VC_JUNCTION_MIN_UM of travel with both line sensors
 * black and the car not in a hard curve. */
#define VC_JUNCTION_MIN_UM 13000u
/* GP28 level held for this much travel is the floor (learned at run time:
 * the sensor's polarity is not fixed by the code). */
#define VC_FLOOR_LEARN_UM 80000u
/* Wheel travel per encoder count: pi * 65 mm / ~2420 counts. */
#define VC_UM_PER_COUNT 84u
#define VC_JUNCTION_MAX_CURVE_STEER 300
/* Motor stall: up to this many automatic retries (stop, wait
 * VC_STALL_RETRY_MS, drive again) before the car stays stopped for START.
 * The count resets after VC_STALL_RETRY_RESET_MS without a stall. */
#define VC_STALL_AUTO_RETRIES 2u
#define VC_STALL_RETRY_MS 1000u
#define VC_STALL_RETRY_RESET_MS 10000u
/* After START or a stall retry, B5 needs a few of its own ticks to re-arm
 * from the acknowledged fault; its mission FAULT is tolerated (motors
 * stopped) for this long instead of safe-stopping again. */
#define VC_FAULT_ACK_GRACE_MS 600u

typedef struct {
    uint32_t now_ms, safety_generation;
    bool clearance, line_valid, imu_valid, hump, hazard, motion_valid;
    bool barcode_active; /* B3 barcode sensor is crossing bars */
    bool obstacle_active, obstacle_fault;
    bool approach_slow;
    UH left, centre;
    UB right; /* barcode sensor GP28 level (0/1), beside the line */
    UB line_flags, faults;
    motion_status_snapshot_t motion;
} VC_INPUT;

/* Owned exclusively by the controller task. Exposed for deterministic tests. */
typedef struct {
    ROBOT_STATE state;
    RUN_MODE mode;
    UINT speed_percent;
    bool running, paused, estop, stopped, drive_sent;
    bool obstacle_handoff; /* B5 moved the robot since our last line command. */
    bool line_recovery_active; /* Survives temporary clearance stops. */
    uint32_t last_drive_ms, recovery_ms, navigation_id, navigation_ms;
    INT last_line_search_direction; /* -1 left / +1 right: last line side */
    INT line_pos;   /* last on-line position 0..1000 (Pololu _lastPosition) */
    INT last_error; /* previous PD error, for the derivative term */
    INT filtered_error; /* low-passed sensor difference */
    bool error_filter_valid; /* filtered_error holds a following sample */
    INT curve_steer; /* hardest recent steering, decaying; sets curve speed */
    INT last_steer, last_speed; /* last DRIVE sent, for smoothing */
    INT follow_steer; /* last steering sent while following the line */
    uint32_t line_seen_ms; /* last tick the line was detected */
#if VC_STEERING_DIAGNOSTICS
    uint32_t last_steering_log_ms;
#endif
    UINT junction_enter_samples, junction_clear_samples;
    bool junction_latched;
    /* Crossing detection from the barcode sensor (see VC_JUNCTION_MIN_UM). */
    UB gp28_level, floor_level;
    bool floor_known, travel_time_valid;
    uint32_t gp28_run_um, travel_last_ms;
    /* Fault acknowledgement and stall retries. */
    bool ack_grace_pending, hold_active, tolerate_active;
    uint32_t hold_until_ms, tolerate_until_ms, last_stall_ms;
    UINT stall_retries;
    UB navigation, pending_navigation;
} VEHICLE_CONTROLLER;

void vehicle_controller_init(VEHICLE_CONTROLLER *controller);
void vehicle_controller_command(VEHICLE_CONTROLLER *controller,
                                const REMOTE_CMD *command);
void vehicle_controller_step(VEHICLE_CONTROLLER *controller,
                             const VC_INPUT *input, UB navigation);
ER vehicle_controller_start(void);

#endif
