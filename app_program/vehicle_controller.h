#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "subsystem1/subsystem1.h"
#include "subsystem2/motion_command.h"
#include <stdbool.h>

#define VC_PERIOD_MS 20u
/* Speed at power-up, in percent (B2: 50 counts/s per percent).  Both line
 * sensors sit on the line when centred, so the car only learns it is off
 * centre when a sensor reaches the line edge and must correct within the
 * sensor gap: slower is more accurate.  12% = 600 counts/s (was 20%).
 * The dashboard SPEED command still changes it at run time. */
#define VC_DEFAULT_SPEED_PERCENT 12u
#define VC_LINE_MAX_AGE_MS 100u
#define VC_IMU_MAX_AGE_MS 1500u /* B4 publishes a quiet snapshot every second. */
#define VC_MOTION_MAX_AGE_MS 100u
#define VC_RECOVERY_MS 3000u
/* Line lost while following: pivot toward the side it was last clearly
 * on for at most this long (about 60 degrees at the default speed), then
 * stop and wait, still armed.  No wide sweeps: they drove the car across
 * the course ("moving randomly"). */
#define VC_LINE_REACQUIRE_MS 3000u
/* A LOST lasting no longer than this while following is treated as a
 * sensor dropout (noise, a scuff, a tape joint): keep the last steering
 * and speed instead of starting recovery on a single bad sample. */
#define VC_LINE_DROPOUT_MS 60u
/* Recovery is a tightening arc toward the remembered side: it starts at
 * this steering and tightens by VC_RECOVER_RAMP_PER_TICK each 20 ms up to
 * B3_LINE_STEER_MAX, instead of an immediate pivot (jerky, and the pivot
 * load is what tends to trip B2's stall check). */
#define VC_RECOVER_START_STEER 400
#define VC_RECOVER_RAMP_PER_TICK 40
#define VC_JUNCTION_CONFIRM_SAMPLES 3u
/* Curve speed: at full steer the base speed drops to VC_CURVE_MIN_PERMILLE.
 * The slowdown follows the hardest recent steering and relaxes by
 * VC_CURVE_DECAY_PER_TICK each 20 ms tick, so the car stays slow through
 * the whole curve instead of speeding up between corrections. */
#define VC_CURVE_MIN_PERMILLE 500 /* half speed at full steer */
#define VC_CURVE_DECAY_PER_TICK 40
/* Smoothness: steering may change by at most this much per 20 ms tick
 * (0 to full 800 in about 0.1 s), and speed may rise by at most
 * VC_SPEED_RISE_PER_TICK counts/s per tick.  Slowing down is immediate. */
#define VC_STEER_SLEW_PER_TICK 150
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
    INT last_line_search_direction;
    bool line_seen_since_start; /* never search for a line not yet seen */
    uint32_t lost_since_ms;     /* start of the current LOST, 0 = on line */ /* -1 left / +1 right: last line side */
    INT line_pos;   /* last on-line position 0..1000 (Pololu _lastPosition) */
    INT last_error; /* previous PD error, for the derivative term */
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
