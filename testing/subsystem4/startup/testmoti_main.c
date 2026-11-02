/** @file testmoti_main.c
 * @brief Verify fresh movement, stale/faulted feedback and B4 rest detection.
 */
#include "testmoti_main.h"
#include "imu_motion.h"
#include "imu_analysis.h"
#include "imu_processing.h"
#include "../../../app_program/subsystem2/motion_command.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static motion_status_snapshot_t g_snapshot;
static uint32_t                 g_now_ms;
static ER                       g_snapshot_error;
static ER                       g_clock_error;

static void fixture (void);
static void test_movement (void);
static void test_invalid (void);
static void test_stationary (void);
static void test_sensor_independence (void);

/** @brief Run the production reader against only B2's existing read API. */
int
main (void)
{
    test_movement();
    test_invalid();
    test_stationary();
    test_sensor_independence();
    puts("PASS: B4 reads B2 snapshots; no B2 write/command API is linked");
    return 0;
}

/** @brief Supply a coherent fresh forward command with measured feedback. */
static void
fixture (void)
{
    g_snapshot                     = (motion_status_snapshot_t){0};
    g_snapshot.command_type        = MOTION_CMD_MOVE;
    g_snapshot.status              = MOTION_STATUS_RUNNING;
    g_snapshot.direction           = MOTION_DIR_FORWARD;
    g_snapshot.command_id          = 7U;
    g_snapshot.left_speed_cps      = 100;
    g_snapshot.right_speed_cps     = 120;
    g_snapshot.left_distance_mm    = 100.0f;
    g_snapshot.right_distance_mm   = 120.0f;
    g_snapshot.total_distance_mm   = 220.0f;
    g_snapshot.sample_time_ms      = 990U;
    g_snapshot.control_interval_ms = 10U;
    g_now_ms                       = 1000U;
    g_snapshot_error               = E_OK;
    g_clock_error                  = E_OK;
}

/** @brief Preserve measured speed/travel and distinguish reverse/turn/fault. */
static void
test_movement (void)
{
    imu_motion_t motion;

    fixture();
    assert(imu_motion_capture(&motion));
    assert((motion.b_valid) && (motion.b_forward) && (!motion.b_stopped));
    assert((100 == motion.left_speed_cps) && (120 == motion.right_speed_cps));
    assert((7U == motion.command_id) && (990U == motion.timestamp_ms));
    assert(fabsf(motion.left_travel_cm - 10.0f) < 0.001f);
    assert(fabsf(motion.right_travel_cm - 12.0f) < 0.001f);
    assert(fabsf(motion.total_travel_cm - 22.0f) < 0.001f);
    g_snapshot.direction       = MOTION_DIR_REVERSE;
    g_snapshot.left_speed_cps  = -100;
    g_snapshot.right_speed_cps = -120;
    assert(imu_motion_capture(&motion));
    assert((motion.b_reverse) && (!motion.b_forward));
    g_snapshot.command_type = MOTION_CMD_TURN;
    assert(imu_motion_capture(&motion));
    assert((motion.b_turning) && (!motion.b_reverse) && (!motion.b_forward));
    g_snapshot.status = MOTION_STATUS_FAULT;
    g_snapshot.fault  = MOTION_FAULT_ENCODER;
    assert(imu_motion_capture(&motion));
    assert((motion.b_valid) && (motion.b_fault) && (!motion.b_forward));
    puts("PASS: command identity, speeds, signed travel and movement context");
}

/** @brief Reject stale/future/uninitialized/corrupt snapshots and clear output.
 */
static void
test_invalid (void)
{
    imu_motion_t motion;

    fixture();
    assert(imu_motion_capture(&motion));
    g_snapshot_error = E_OBJ;
    assert(!imu_motion_capture(&motion));
    assert((!motion.b_valid) && (!motion.b_forward) &&
           (0 == motion.left_speed_cps));
    fixture();
    g_clock_error = E_OBJ;
    assert(!imu_motion_capture(&motion));
    fixture();
    g_snapshot.sample_time_ms = 900U;
    assert(imu_motion_capture(&motion));
    g_snapshot.sample_time_ms = 899U;
    assert(!imu_motion_capture(&motion));
    g_snapshot.sample_time_ms = 1001U;
    assert(!imu_motion_capture(&motion));
    fixture();
    g_snapshot.control_interval_ms = 0U;
    assert(!imu_motion_capture(&motion));
    fixture();
    g_snapshot.left_distance_mm = NAN;
    assert(!imu_motion_capture(&motion));
    fixture();
    /* Inject an invalid enum deliberately; production must reject it. */
    g_snapshot.command_type = (motion_cmd_type_t)99;
    assert(!imu_motion_capture(&motion));
    fixture();
    g_snapshot.right_speed_cps = INT32_MIN;
    assert(!imu_motion_capture(&motion));
    fixture();
    g_now_ms                  = 5U;
    g_snapshot.sample_time_ms = UINT32_MAX - 4U;
    assert(imu_motion_capture(&motion));
    assert(!imu_motion_capture(NULL));
    puts("PASS: freshness boundary, clock wrap, bad data and cleared failures");
}

/** @brief Confirm rest only when paired feedback says physically stopped. */
static void
test_stationary (void)
{
    imu_motion_t          motion;
    imu_analysis_t        analysis;
    imu_analysis_config_t config;
    imu_data_t            data = {0};
    imu_event_t           event;

    fixture();
    g_snapshot.command_type       = MOTION_CMD_STOP;
    g_snapshot.status             = MOTION_STATUS_IDLE;
    g_snapshot.physically_stopped = true;
    g_snapshot.left_speed_cps     = 0;
    g_snapshot.right_speed_cps    = 0;
    assert(imu_motion_capture(&motion));
    assert((motion.b_stopped) && (!motion.b_running));
    imu_analysis_default_config(&config);
    assert(imu_analysis_init(&analysis, &config));
    data.b_valid      = true;
    data.accel_z      = 9.81f;
    data.timestamp_ms = g_now_ms;
    data.motion       = motion;
    assert(imu_analysis_process(&analysis, &data, &event));
    assert(IMU_MOTION_STATIONARY == event.motion_state);
    data.timestamp_ms = 1100U;
    assert(imu_analysis_process(&analysis, &data, &event));
    assert(IMU_MOTION_UNKNOWN == event.motion_state);
    data.timestamp_ms += 10U;
    data.motion.b_valid = false;
    assert(imu_analysis_process(&analysis, &data, &event));
    assert(IMU_MOTION_UNKNOWN == event.motion_state);
    puts("PASS: stale or unavailable feedback never confirms stationary");
}

/** @brief Unavailable/faulted B2 feedback must not create an IMU fault. */
static void
test_sensor_independence (void)
{
    imu_processing_t        processing;
    imu_processing_config_t config;
    imu_processing_health_t health;
    imu_data_t              input = {0};
    imu_data_t              output;

    imu_processing_default_config(&config);
    assert(imu_processing_init(&processing, &config));
    input.timestamp_ms = 1000U;
    input.b_valid      = true;
    input.accel_z      = 9.81f;
    assert(
        imu_processing_process(&processing, &input, 1000U, &output, &health));
    assert((output.b_valid) && (!output.motion.b_valid) &&
           (!health.b_fault_active));
    input.timestamp_ms += 10U;
    input.motion.b_valid = true;
    input.motion.b_fault = true;
    assert(
        imu_processing_process(&processing, &input, 1010U, &output, &health));
    assert((output.b_valid) && (!health.b_fault_started));
    puts("PASS: missing/faulted B2 context does not become an IMU fault");
}

/* Stub only the existing B2 read API and kernel clock, not command/fault APIs.
 */

/** @brief Return one coherent snapshot or simulate an unavailable B2 API. */
ER
motion_get_status (motion_status_snapshot_t * p_status)
{
    assert(NULL != p_status);
    *p_status = g_snapshot;
    return g_snapshot_error;
}

/** @brief Return a controlled snapshot-validation clock. */
ER
tk_get_tim (SYSTIM * p_time)
{
    p_time->hi = 0U;
    p_time->lo = g_now_ms;
    return g_clock_error;
}

/*** end of file ***/

