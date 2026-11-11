#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "subsystem1/subsystem1.h"
#include "subsystem2/motion_command.h"
#include <stdbool.h>

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
#define VC_LINE_REACQUIRE_MS 1000u
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
/* First-order filtering of ADC difference; 2 = responsive, 3 = smoother. */
#define VC_LINE_ERROR_FILTER_DIV 2
#define VC_SPEED_RISE_PER_TICK 100
/* CALIBRATE: slowest wheel speed (counts/s) the loaded motors hold without
 * stalling; about 25 mm/s.  A curve's inner wheel is never asked to run
 * between 0 and this: a little below it is raised to it, far below it is
 * set to 0 so the car pivots around that wheel (a wheel commanded to 0 is
 * never reported as stalled). */
#define VC_MIN_WHEEL_CPS 300
/* While a barcode is under the sensor, hold this steady speed (~41 mm/s) so
 * bar widths are timed cleanly: proven teams read barcodes slowly. */
#define VC_BARCODE_CPS 500
#define VC_JUNCTION_RELEASE_SAMPLES 3u

typedef struct {
    uint32_t now_ms, safety_generation;
    bool clearance, line_valid, imu_valid, hump, hazard, motion_valid;
    bool barcode_active; /* B3 barcode sensor is crossing bars */
    bool obstacle_active, obstacle_fault;
    UH left, centre;
    UB line_flags, faults;
    motion_status_snapshot_t motion;
} VC_INPUT;

/* Owned exclusively by the controller task. Exposed for deterministic tests. */
typedef struct {
    ROBOT_STATE state;
    RUN_MODE mode;
    UINT speed_percent;
    bool running, paused, estop, stopped, drive_sent;
    uint32_t last_drive_ms, recovery_ms, navigation_id, navigation_ms;
    INT last_line_search_direction; /* -1 left / +1 right: last line side */
    INT line_pos;   /* last on-line position 0..1000 (Pololu _lastPosition) */
    INT last_error; /* last raw line difference, for diagnostics */
    INT filtered_line_error;
    bool filtered_line_valid;
    INT curve_steer; /* hardest recent steering, decaying; sets curve speed */
    INT last_steer, last_speed; /* last DRIVE sent, for smoothing */
    UINT junction_enter_samples, junction_clear_samples;
    bool junction_latched;
    UB navigation, pending_navigation;
} VEHICLE_CONTROLLER;

void vehicle_controller_init(VEHICLE_CONTROLLER *controller);
void vehicle_controller_command(VEHICLE_CONTROLLER *controller,
                                const REMOTE_CMD *command);
void vehicle_controller_step(VEHICLE_CONTROLLER *controller,
                             const VC_INPUT *input, UB navigation);
ER vehicle_controller_start(void);

#endif
