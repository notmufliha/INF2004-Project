/** @file testodom_main.c
 * @brief Test capture timing, error races and shared B2 wheel calibration.
 */
#include "testodom_main.h"
#include "imu_odom.h"
#include "gy511.h"
#include "../../../app_program/subsystem2/encoder_control.h"
#include "../../../app_program/subsystem2/motion_command.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t g_now_ms = 105U;
static uint32_t g_error_calls;
static uint32_t g_count_calls;
static bool     gb_error_race;
static ER       g_clock_error;
static bool     gb_context_ready;
static bool     gb_right_error_race;
static uint32_t g_encoder_delay_ms;
static uint32_t g_context_delay_ms;
static uint32_t g_clock_calls;
static uint32_t g_clock_fail_at;
static uint8_t  g_acc_registers[256];
static uint8_t  g_mag_registers[256];
static uint32_t g_pair_calls;
static uint32_t g_mag_calls;
static uint32_t g_burst_ms = 4U;
static bool     gb_track_reads;
static bool     gb_fail_mag;
static bool     gb_driver_running;
static bool     gb_driver_fault;

static void test_diagnostics (void);
static void test_driver_pairing (void);
static void test_magnetic_policy (gy511_t * p_sensor);
static bool read_sensor (void * p_context, uint8_t address, uint8_t reg,
                         uint8_t * p_bytes, uint32_t count);
static bool write_sensor (void * p_context, uint8_t address, uint8_t reg,
                          uint8_t value);
static bool sensor_clock (void * p_context, uint32_t * p_time_ms);
static void pair_sensor (void * p_context, imu_data_t * p_data,
                         uint32_t begin_ms, uint32_t end_ms, bool b_time_valid);

/** @brief Execute the actual acquisition adapter without a board. */
int
main (void)
{
    imu_data_t   data = {0};

    data.b_valid      = true;
    data.timestamp_ms = 100U;
    imu_odom_capture(&data);
    assert(data.b_odometry_valid);
    assert(0U == data.odometry_reasons);
    assert(5U == data.odometry_age_ms);
    assert(!data.motion.b_valid);
    gb_context_ready = true;
    imu_odom_capture(&data);
    assert(data.motion.b_valid);
    assert(data.motion.b_stopped);
    assert((1000 == data.encoder_left) && (2000 == data.encoder_right));
    assert((1000 == data.encoder_left) && (2000 == data.encoder_right));
    assert((2U == data.encoder_left_errors) &&
           (3U == data.encoder_right_errors));
    assert(105U == data.timestamp_ms);
    gb_error_race = true;
    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    assert(IMU_ODOM_REASON_LEFT == data.odometry_reasons);
    gb_error_race = false;
    g_now_ms += 20U;
    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    assert(IMU_ODOM_REASON_AGE == data.odometry_reasons);
    g_clock_error = E_OBJ;
    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    assert(0U != (data.odometry_reasons & IMU_ODOM_REASON_CLOCK));
    data.b_valid = false;

    const uint32_t prior_calls = g_count_calls;

    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    assert(IMU_ODOM_REASON_INPUT == data.odometry_reasons);
    assert(0U == data.odometry_time_valid);
    assert(prior_calls == g_count_calls);
    imu_odom_capture(NULL);
    test_diagnostics();
    test_driver_pairing();
    puts("PASS: real odometry adapter, paired counts, error races and stale "
         "capture");
    return 0;
}

/** @brief Isolate acquisition age, encoder races, phase costs and clock errors.
 */
static void
test_diagnostics (void)
{
    imu_data_t data = {0};

    data.b_valid       = true;
    g_clock_error      = E_OK;
    g_now_ms           = 1005U;
    data.timestamp_ms  = 1000U;
    g_encoder_delay_ms = 2U;
    g_context_delay_ms = 3U;
    imu_odom_capture(&data);
    assert(data.b_odometry_valid);
    assert(0U == data.odometry_reasons);
    assert(10U == data.odometry_age_ms);
    assert(2U == data.odometry_encoder_ms);
    assert(3U == data.odometry_context_ms);
    assert(IMU_ODOM_TIME_ALL == data.odometry_time_valid);
    g_encoder_delay_ms = 3U;
    g_now_ms           = 2005U;
    data.timestamp_ms  = 2000U;
    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    assert(IMU_ODOM_REASON_AGE == data.odometry_reasons);
    assert(11U == data.odometry_age_ms);
    gb_error_race       = true;
    gb_right_error_race = true;
    g_now_ms            = 3005U;
    data.timestamp_ms   = 3000U;
    imu_odom_capture(&data);
    assert((IMU_ODOM_REASON_AGE | IMU_ODOM_REASON_LEFT |
            IMU_ODOM_REASON_RIGHT) == data.odometry_reasons);
    gb_error_race       = false;
    gb_right_error_race = false;
    g_encoder_delay_ms  = 2U;
    g_now_ms            = UINT32_MAX - 1U;
    data.timestamp_ms   = g_now_ms - 5U;
    imu_odom_capture(&data);
    assert(data.b_odometry_valid);
    assert(10U == data.odometry_age_ms);
    g_clock_calls     = 0U;
    g_clock_fail_at   = 1U;
    data.timestamp_ms = g_now_ms;
    imu_odom_capture(&data);
    assert(
        data.b_odometry_valid); /* Diagnostic probe failure alone is benign. */
    assert(IMU_ODOM_REASON_PROBE == data.odometry_reasons);
    assert(0U == (data.odometry_time_valid & IMU_ODOM_TIME_ENCODER));
    g_clock_calls = 0U;
    g_clock_fail_at =
        4U; /* Context is ready, so it makes the third clock call. */
    data.timestamp_ms = g_now_ms;
    imu_odom_capture(&data);
    assert(!data.b_odometry_valid);
    assert(IMU_ODOM_REASON_CLOCK == data.odometry_reasons);
    assert(IMU_ODOM_TIME_ENCODER == data.odometry_time_valid);
    g_clock_fail_at   = 0U;
    g_now_ms          = 100000U;
    data.timestamp_ms = 1U;
    imu_odom_capture(&data);
    assert(UINT16_MAX == data.odometry_age_ms);
    assert(IMU_ODOM_REASON_AGE == data.odometry_reasons);
    puts("PASS: odometry failure split, 10/11ms boundary, phase timing, "
         "wraparound, probe failure and saturation");
}

/** @brief Pair the actual sensor decoder and encoder adapter with delayed I2C.
 */
static void
test_driver_pairing (void)
{
    gy511_t               sensor;
    gy511_config_t        config;
    imu_data_t            data;
    const gy511_bus_t     bus     = {NULL, read_sensor, write_sensor};
    const gy511_capture_t capture = {NULL, sensor_clock, pair_sensor};

    g_clock_error         = E_OK;
    g_clock_fail_at       = 0U;
    g_encoder_delay_ms    = 2U;
    g_context_delay_ms    = 0U;
    g_acc_registers[0x27] = 8U;
    g_acc_registers[0x2C] = 0x40U;
    g_acc_registers[0x2D] = 0x1FU;
    g_mag_registers[0x0A] = 'H';
    g_mag_registers[0x0B] = '4';
    g_mag_registers[0x0C] = '3';
    g_mag_registers[0x09] = 1U;
    g_mag_registers[0x04] = 110U;
    gy511_default_config(&config);
    assert(gy511_init(&sensor, &bus, &config));
    assert(gy511_set_capture(&sensor, &capture));
    assert(!gy511_set_capture(NULL, &capture));
    assert(!gy511_set_capture(&sensor, NULL));
    gb_track_reads = true;
    g_now_ms       = 1000U;
    assert(GY511_OK == gy511_read(&sensor, 1000U, &data));
    assert(1U == g_pair_calls);
    assert(data.b_valid && data.b_odometry_valid);
    assert(1004U == data.timestamp_ms); /* Combined status/XYZ burst=4ms. */
    assert(1030U == g_now_ms); /* Encoders=2ms, later magnetic reads=24ms. */
    assert(6U ==
           data.odometry_age_ms); /* Burst+pair, not the whole read path. */
    assert(4U == data.acceleration_read_ms);
    assert(24U == data.magnetic_read_ms);
    assert(0U == data.odometry_reasons);
    g_mag_calls = 0U;
    g_now_ms    = UINT32_MAX - 3U;
    assert(GY511_OK == gy511_read(&sensor, g_now_ms, &data));
    assert(data.b_odometry_valid);
    assert(0U == data.timestamp_ms); /* Burst completes across clock wrap. */
    assert(6U == data.odometry_age_ms);
    g_mag_calls = 0U;
    g_burst_ms  = 11U;
    assert(GY511_OK == gy511_read(&sensor, g_now_ms, &data));
    assert(!data.b_odometry_valid); /* Slow burst itself is still rejected. */
    assert(IMU_ODOM_REASON_AGE == data.odometry_reasons);
    g_mag_calls = 0U;
    g_burst_ms  = 4U;
    gb_fail_mag = true;
    assert(GY511_OK == gy511_read(&sensor, g_now_ms, &data));
    assert(data.b_odometry_valid && !data.b_magnetic_valid);
    gb_fail_mag     = false;
    g_mag_calls     = 0U;
    g_clock_calls   = 0U;
    g_clock_fail_at = 2U; /* The acceleration burst-end probe fails. */
    const uint32_t counts_before = g_count_calls;

    assert(GY511_OK == gy511_read(&sensor, g_now_ms, &data));
    assert(!data.b_odometry_valid);
    assert(0U != (data.odometry_reasons & IMU_ODOM_REASON_CLOCK));
    assert(counts_before == g_count_calls);
    g_clock_fail_at       = 0U;
    g_mag_calls           = 0U;
    g_acc_registers[0x2C] = 0U;
    g_acc_registers[0x2D] = 0x80U;
    assert(GY511_ERROR == gy511_read(&sensor, g_now_ms, &data));
    assert(!data.b_valid && !data.b_odometry_valid);
    assert(0U != (data.odometry_reasons & IMU_ODOM_REASON_INPUT));
    g_acc_registers[0x27]      = 0U;
    const uint32_t prior_pairs = g_pair_calls;

    assert(GY511_NO_DATA == gy511_read(&sensor, g_now_ms, &data));
    assert(prior_pairs == g_pair_calls);
    test_magnetic_policy(&sensor);
    puts("PASS: acceleration pairing precedes math/magnetometer; slow bursts, "
         "bad clocks, missing data and clipping remain rejected");
}

/** @brief Defer magnetics for motion/fault/unknown context, resume only at
 * idle. */
static void
test_magnetic_policy (gy511_t * p_sensor)
{
    imu_data_t data;

    g_acc_registers[0x27]             = 8U;
    g_acc_registers[0x2C]             = 0x40U;
    g_acc_registers[0x2D]             = 0x1FU;
    g_burst_ms                        = 4U;
    g_mag_calls                       = 0U;
    p_sensor->config.b_mag_calibrated = true;
    assert(gy511_set_magnetic_idle_only(p_sensor, true));
    assert(!gy511_set_magnetic_idle_only(NULL, true));
    gb_driver_running = true;
    g_now_ms          = 10000U;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    assert(0U == g_mag_calls);
    assert(!data.b_heading_valid && !data.b_turn_rate_valid);
    assert(0U != (data.odometry_time_valid & IMU_READ_MAG_DEFERRED));
    gb_driver_running = false;
    g_now_ms          = 20000U;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    g_now_ms = 20995U;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    assert(0U == g_mag_calls);
    g_now_ms = 21000U;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    assert(2U == g_mag_calls);
    assert(data.b_heading_valid && !data.b_turn_rate_valid);
    g_mag_calls     = 0U;
    gb_driver_fault = true;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    assert(0U == g_mag_calls);
    assert(!data.b_heading_valid && !p_sensor->b_have_heading);
    gb_driver_fault  = false;
    gb_context_ready = false;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    assert(0U == g_mag_calls);
    assert(!data.b_magnetic_valid);
    gb_context_ready = true;
    assert(gy511_set_magnetic_idle_only(p_sensor, true));
    g_now_ms = UINT32_MAX - 500U;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    g_now_ms = 500U;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    assert(2U == g_mag_calls);
    g_mag_calls = 0U;
    assert(gy511_set_magnetic_idle_only(p_sensor, false));
    gb_context_ready = false;
    assert(GY511_OK == gy511_read(p_sensor, g_now_ms, &data));
    assert(2U == g_mag_calls); /* Full mode remains available explicitly. */
    puts("PASS: idle-only magnetics, one-second settle, clock wrap, "
         "fault/unknown suppression and full-mode opt-out");
}

/** @brief Fake register reads include intentionally slow magnetic work.
 */
static bool
read_sensor (void * p_context, uint8_t address, uint8_t reg, uint8_t * p_bytes,
             uint32_t count)
{
    (void)p_context;
    const bool     b_accel = (0x19U == address);
    uint8_t *      p_bank  = b_accel ? g_acc_registers : g_mag_registers;
    const uint32_t offset  = b_accel ? ((uint32_t)reg & 0x7FU) : reg;

    if (gb_track_reads)
    {
        if (!b_accel)
        {
            ++g_mag_calls;
            assert(g_pair_calls > 0U);
        }
        if (b_accel)
        {
            assert((0xA7U == reg) && (7U == count));
        }
        g_now_ms += b_accel ? g_burst_ms : 12U;
    }
    if (gb_fail_mag && !b_accel)
    {
        return false;
    }
    for (uint32_t index = 0U; index < count; ++index)
    {
        p_bytes[index] = p_bank[offset + index];
    }
    return true;
}

/** @brief Store configuration bytes for the driver's readback checks. */
static bool
write_sensor (void * p_context, uint8_t address, uint8_t reg, uint8_t value)
{
    (void)p_context;
    uint8_t * p_bank = (0x19U == address) ? g_acc_registers : g_mag_registers;

    p_bank[reg] = value;
    return true;
}

/** @brief Use the native kernel clock stub, including failure injection. */
static bool
sensor_clock (void * p_context, uint32_t * p_time_ms)
{
    SYSTIM time;

    (void)p_context;
    ER error = tk_get_tim(&time);

    *p_time_ms = time.lo;
    return E_OK == error;
}

/** @brief Match the real adapter's callback and verify ordering before math. */
static void
pair_sensor (void * p_context, imu_data_t * p_data, uint32_t begin_ms,
             uint32_t end_ms, bool b_time_valid)
{
    (void)p_context;
    assert(0U == g_mag_calls);
    assert(!p_data->b_valid);
    assert(0.0f == p_data->accel_z);
    ++g_pair_calls;
    imu_odom_pair(p_data, begin_ms, end_ms, b_time_valid);
}

/* These stubs retain the external kernel/B2 ABI names and exact-width types. */

/** @brief Provide a deterministic acquisition clock or kernel error. */
ER
tk_get_tim (SYSTIM * p_time)
{
    ++g_clock_calls;
    p_time->hi = 0U;
    p_time->lo = g_now_ms;
    return ((0U != g_clock_fail_at) && (g_clock_calls == g_clock_fail_at))
               ? E_OBJ
               : g_clock_error;
}

/** @brief Return one paired real-interface count snapshot. */
void
encoder_get_counts (INT * p_left, INT * p_right)
{
    *p_left  = 1000;
    *p_right = 2000;
    g_count_calls++;
    g_now_ms += g_encoder_delay_ms;
}

/** @brief Inject a transition error between the before/after error snapshots.
 */
void
encoder_get_invalid_counts (UW * p_left, UW * p_right)
{
    g_error_calls++;
    *p_left = ((gb_error_race) && (0U == (g_error_calls % 2U))) ? 3U : 2U;
    *p_right =
        ((gb_right_error_race) && (0U == (g_error_calls % 2U))) ? 4U : 3U;
}

/** @brief An unavailable B2 controller must not suppress raw encoder capture.
 */
ER
motion_get_status (motion_status_snapshot_t * p_status)
{
    assert(NULL != p_status);
    g_now_ms += g_context_delay_ms;

    if (gb_context_ready)
    {
        *p_status                    = (motion_status_snapshot_t){0};
        p_status->physically_stopped = !gb_driver_running;
        p_status->status =
            gb_driver_running ? MOTION_STATUS_RUNNING : MOTION_STATUS_IDLE;
        if (gb_driver_running)
        {
            p_status->command_type    = MOTION_CMD_MOVE;
            p_status->direction       = MOTION_DIR_FORWARD;
            p_status->left_speed_cps  = 500;
            p_status->right_speed_cps = 500;
        }
        if (gb_driver_fault)
        {
            p_status->fault  = MOTION_FAULT_ENCODER;
            p_status->status = MOTION_STATUS_FAULT;
        }
        p_status->sample_time_ms      = g_now_ms;
        p_status->control_interval_ms = 10U;
        if (gb_track_reads)
        {
            /* B2's last control tick precedes the acceleration source time. */
            p_status->sample_time_ms -= g_encoder_delay_ms;
        }
        p_status->left_encoder_counts  = -123;
        p_status->right_encoder_counts = -456;
        return E_OK;
    }

    return E_OBJ;
}

/*** end of file ***/
