/** @file gy511.c
 * @brief LSM303DLHC sensor acquisition and calibration.
 */
/* LSM303DLHC DS7202 Rev 2:
 * st.com/resource/en/datasheet/lsm303dlhc.pdf. */
#include "gy511.h"
#include <math.h>
#include <stddef.h>

#define GY511_AXIS_COUNT 3
#define GY511_ACCEL_BYTES 6U
#define GY511_IDLE_SETTLE_MS 1000U

#define ACC_ADDR 0x19U
#define MAG_ADDR 0x1EU
#define GRAVITY 9.81f
#define RAD_TO_DEG 57.295779513f

static int32_t axis_index (int32_t axis);
static bool    magnetic_allowed (gy511_t * p_sensor, const imu_data_t * p_data);

static int32_t signed_word (uint8_t high, uint8_t low);

static float32_t mapped (const float32_t p_values[3], int32_t axis);

static gy511_result_t read_acceleration (gy511_t *    p_sensor,
                                         uint32_t     timestamp_ms,
                                         imu_data_t * p_data,
                                         float32_t *  p_norm);

static void calculate_heading (gy511_t * p_sensor, imu_data_t * p_data,
                               float32_t norm, const float32_t p_field[3],
                               uint32_t timestamp_ms);

static void read_magnetic_field (gy511_t * p_sensor, imu_data_t * p_data,
                                 float32_t norm, uint32_t timestamp_ms);
static gy511_result_t acceleration_bytes (gy511_t *    p_sensor,
                                          imu_data_t * p_data,
                                          uint8_t      p_bytes[6]);
static bool           magnetic_bytes (gy511_t * p_sensor, imu_data_t * p_data,
                                      uint8_t p_bytes[6], uint32_t * p_time_ms);
static bool           read_clock (gy511_t * p_sensor, uint32_t * p_time_ms);
static void           read_duration (gy511_t * p_sensor, imu_data_t * p_data,
                                     uint32_t begin_ms, uint32_t end_ms,
                                     bool b_time_valid, bool b_acceleration);

/** @brief Supply the default robot-axis map and calibration values. */
void
gy511_default_config (gy511_config_t * p_config)
{
    if (p_config != NULL)
    {
        *p_config =
            (gy511_config_t){{1, 2, 3}, {0, 0, 0}, {0, 0, 0}, {1, 1, 1}, false};
    }
}

/** @brief Validate the configuration and verify the sensor registers. */
bool
gy511_init (gy511_t * p_sensor, const gy511_bus_t * p_bus,
            const gy511_config_t * p_config)
{
    uint8_t sensor_identity[3];
    uint8_t value;

    if (NULL == p_sensor)
    {
        return false;
    }

    *p_sensor = (gy511_t){0};
    if ((NULL == p_bus) || (NULL == p_config) || (NULL == p_bus->p_read) ||
        (NULL == p_bus->p_write))
    {
        return false;
    }

    for (int32_t index = 0; index < GY511_AXIS_COUNT; ++index)
    {
        if ((0 == p_config->axis[index]) || (p_config->axis[index] < -3) ||
            (p_config->axis[index] > 3) ||
            (!isfinite(p_config->accel_offset_ms2[index])) ||
            (!isfinite(p_config->mag_offset_ut[index])) ||
            (!isfinite(p_config->mag_scale[index])) ||
            (p_config->mag_scale[index] <= 0.0f))
        {
            return false;
        }

        for (int32_t other_index = 0; other_index < index; ++other_index)
        {
            if (axis_index(p_config->axis[index]) ==
                axis_index(p_config->axis[other_index]))
            {
                return false;
            }
        }
    }

    p_sensor->bus    = *p_bus;
    p_sensor->config = *p_config;

    if ((!p_bus->p_read(p_bus->p_context, MAG_ADDR, 0x0AU, sensor_identity,
                        3U)) ||
        (sensor_identity[0] != 0x48U) || (sensor_identity[1] != 0x34U) ||
        (sensor_identity[2] != 0x33U))
    {
        return false;
    }

    /* 100 Hz XYZ enabled, normal mode; BDU + high-resolution +/-4g.
     * 12-bit left-justified data, sensitivity 2 mg/LSB. */
    const uint8_t address[] = {ACC_ADDR, ACC_ADDR, MAG_ADDR, MAG_ADDR,
                               MAG_ADDR};
    const uint8_t reg[]     = {0x20U, 0x23U, 0x00U, 0x01U, 0x02U};
    const uint8_t setting[] = {0x57U, 0x98U, 0x18U, 0x20U, 0x00U};

    for (size_t index = 0U; index < sizeof(reg); ++index)
    {
        if ((!p_bus->p_write(p_bus->p_context, address[index], reg[index],
                             setting[index])) ||
            (!p_bus->p_read(p_bus->p_context, address[index], reg[index],
                            &value, 1U)) ||
            (value != setting[index]))
        {
            return false;
        }
    }

    p_sensor->b_ready = true;
    return true;
}

/** @brief Remove level offsets while preserving gravity on robot Z. */
bool
gy511_set_level_calibration (gy511_t * p_sensor, const float32_t p_mean[3])
{
    if ((NULL == p_sensor) || (NULL == p_mean) || (!p_sensor->b_ready) ||
        (!isfinite(p_mean[0])) || (!isfinite(p_mean[1])) ||
        (!isfinite(p_mean[2])) || (fabsf(p_mean[0]) > 1.0f) ||
        (fabsf(p_mean[1]) > 1.0f) || (fabsf(p_mean[2] - GRAVITY) > 1.0f))
    {
        return false;
    }

    p_sensor->config.accel_offset_ms2[0] = p_mean[0];
    p_sensor->config.accel_offset_ms2[1] = p_mean[1];
    p_sensor->config.accel_offset_ms2[2] = p_mean[2] - GRAVITY;
    return true;
}

/** @brief Read fresh acceleration and optional magnetic measurements. */
gy511_result_t
gy511_read (gy511_t * p_sensor, uint32_t timestamp_ms, imu_data_t * p_data)
{
    float32_t      norm = 0.0f;
    gy511_result_t result =
        read_acceleration(p_sensor, timestamp_ms, p_data, &norm);

    if (GY511_OK == result)
    {
        if (magnetic_allowed(p_sensor, p_data))
        {
            read_magnetic_field(p_sensor, p_data, norm, p_data->timestamp_ms);
        }
        else
        {
            p_data->odometry_time_valid |= IMU_READ_MAG_DEFERRED;
        }
    }

    if ((GY511_OK != result) && (NULL != p_data))
    {
        p_data->b_odometry_valid = false;
        p_data->odometry_reasons |= IMU_ODOM_REASON_INPUT;
    }

    return result;
}

/** @brief Install sampling-task hooks after sensor initialization. */
bool
gy511_set_capture (gy511_t * p_sensor, const gy511_capture_t * p_capture)
{
    if ((NULL == p_sensor) || (!p_sensor->b_ready) || (NULL == p_capture) ||
        (NULL == p_capture->p_clock) || (NULL == p_capture->p_pair))
    {
        return false;
    }

    p_sensor->capture = *p_capture;
    return true;
}

/** @brief Configure the sampling-task policy without retaining old headings. */
bool
gy511_set_magnetic_idle_only (gy511_t * p_sensor, bool b_enabled)
{
    if ((NULL == p_sensor) || (!p_sensor->b_ready))
    {
        return false;
    }

    p_sensor->b_magnetic_idle_only = b_enabled;
    p_sensor->b_idle_seen          = false;
    p_sensor->b_have_heading       = false;
    return true;
}

/** @brief Convert a validated signed axis number to a zero-based index. */
static int32_t
axis_index (int32_t axis)
{
    return (axis < 0 ? -axis : axis) - 1;
}

/** @brief Decode two bytes as a signed 16-bit two-complement value. */
static int32_t
signed_word (uint8_t high, uint8_t low)
{
    /* Widening uint8_t to uint32_t is exact before the unsigned shift. */
    uint32_t word = ((uint32_t)high << 8U) | (uint32_t)low;

    /* The assembled word is at most 65535 and fits int32_t exactly. */
    return word >= 32768U ? (int32_t)word - 65536 : (int32_t)word;
}

/** @brief Select and orient one axis using the validated mounting map. */
static float32_t
mapped (const float32_t p_values[3], int32_t axis)
{
    return p_values[axis_index(axis)] * (axis < 0 ? -1.0f : 1.0f);
}

/** @brief Decode unclipped acceleration and estimate finite tilt. */
static gy511_result_t
read_acceleration (gy511_t * p_sensor, uint32_t timestamp_ms,
                   imu_data_t * p_data, float32_t * p_norm)
{
    uint8_t   bytes[6];
    float32_t accel[3];
    float32_t robot[3];

    if (NULL == p_data)
    {
        return GY511_ERROR;
    }

    *p_data              = (imu_data_t){0};
    p_data->timestamp_ms = timestamp_ms;

    if ((NULL == p_sensor) || (!p_sensor->b_ready))
    {
        return GY511_ERROR;
    }

    gy511_result_t result = acceleration_bytes(p_sensor, p_data, bytes);

    if (GY511_OK != result)
    {
        return result;
    }

    for (int32_t index = 0; index < GY511_AXIS_COUNT; ++index)
    {
        int32_t raw = signed_word(bytes[(2 * index) + 1], bytes[2 * index]);

        /* Reject clipping; divide left-aligned 12-bit signed readings by 16. */
        if ((raw >= 32752) || (raw <= -32768))
        {
            return GY511_ERROR;
        }

        /* The signed 12-bit count fits binary32 without rounding. */
        accel[index] = (float32_t)(raw / 16) * 0.002f * GRAVITY;
    }

    for (int32_t index = 0; index < GY511_AXIS_COUNT; ++index)
    {
        robot[index] = mapped(accel, p_sensor->config.axis[index]) -
                       p_sensor->config.accel_offset_ms2[index];
    }

    p_data->accel_x = robot[0];
    p_data->accel_y = robot[1];
    p_data->accel_z = robot[2];

    float32_t norm = hypotf(hypotf(robot[0], robot[1]), robot[2]);

    if ((!isfinite(norm)) || (norm < 0.1f))
    {
        return GY511_ERROR;
    }

    p_data->pitch_deg =
        atan2f(-robot[0], hypotf(robot[1], robot[2])) * RAD_TO_DEG;
    p_data->roll_deg = atan2f(robot[1], robot[2]) * RAD_TO_DEG;
    p_data->b_valid =
        (isfinite(p_data->pitch_deg)) && (isfinite(p_data->roll_deg));
    *p_norm = norm;
    return p_data->b_valid ? GY511_OK : GY511_ERROR;
}

/** @brief Estimate tilt-compensated heading and optional turn rate. */
static void
calculate_heading (gy511_t * p_sensor, imu_data_t * p_data, float32_t norm,
                   const float32_t p_field[3], uint32_t timestamp_ms)
{
    /* Tilt compensation only near 1g; acceleration makes the tilt estimate
     * unreliable. */
    if (fabsf(norm - GRAVITY) > 1.5f)
    {
        p_sensor->b_have_heading = false;
        return;
    }

    float32_t pitch        = p_data->pitch_deg / RAD_TO_DEG;
    float32_t roll         = p_data->roll_deg / RAD_TO_DEG;
    float32_t horizontal_x = (p_field[0] * cosf(pitch)) +
                             (p_field[1] * sinf(roll) * sinf(pitch)) +
                             (p_field[2] * cosf(roll) * sinf(pitch));
    float32_t horizontal_y =
        (p_field[1] * cosf(roll)) - (p_field[2] * sinf(roll));
    const float32_t horizontal_norm = hypotf(horizontal_x, horizontal_y);

    if ((!isfinite(horizontal_x)) || (!isfinite(horizontal_y)) ||
        (!isfinite(horizontal_norm)) || (horizontal_norm < 1.0f))
    {
        p_sensor->b_have_heading = false;
        return;
    }

    p_data->heading_deg = atan2f(-horizontal_y, horizontal_x) * RAD_TO_DEG;

    if (p_data->heading_deg < 0.0f)
    {
        p_data->heading_deg += 360.0f;
    }

    if (!isfinite(p_data->heading_deg))
    {
        p_sensor->b_have_heading = false;
        return;
    }

    p_data->b_heading_valid      = true;
    p_data->b_heading_calibrated = p_sensor->config.b_mag_calibrated;

    uint32_t interval_ms = timestamp_ms - p_sensor->previous_heading_ms;

    if ((p_sensor->config.b_mag_calibrated) && (p_sensor->b_have_heading) &&
        (interval_ms > 0U) && (interval_ms <= 100U))
    {
        float32_t delta = p_data->heading_deg - p_sensor->previous_heading;

        if (delta > 180.0f)
        {
            delta -= 360.0f;
        }

        if (delta < -180.0f)
        {
            delta += 360.0f;
        }

        /* The validated interval is 1..100 ms, exactly representable. */
        p_data->turn_rate_dps     = delta * 1000.0f / (float32_t)interval_ms;
        p_data->b_turn_rate_valid = isfinite(p_data->turn_rate_dps);
    }

    p_sensor->previous_heading    = p_data->heading_deg;
    p_sensor->previous_heading_ms = timestamp_ms;
    p_sensor->b_have_heading      = true;
    return;
}

/** @brief Decode fresh magnetic values and apply calibration. */
static void
read_magnetic_field (gy511_t * p_sensor, imu_data_t * p_data, float32_t norm,
                     uint32_t timestamp_ms)
{
    uint8_t   bytes[6];
    float32_t mag[3];

    /* Magnetometer runs at 75 Hz. No fresh magnetic sample is not an IMU fault.
     */
    /* Widening the received status byte to uint32_t is exact. */
    if (!magnetic_bytes(p_sensor, p_data, bytes, &timestamp_ms))
    {
        return;
    }

    int32_t raw_mag_x = signed_word(bytes[0], bytes[1]);
    int32_t raw_mag_z = signed_word(bytes[2], bytes[3]);
    int32_t raw_mag_y = signed_word(bytes[4], bytes[5]);

    if ((-4096 == raw_mag_x) || (-4096 == raw_mag_y) || (-4096 == raw_mag_z))
    {
        return;
    }

    /* Signed 16-bit decoded counts fit binary32 exactly before scaling. */
    mag[0] = (float32_t)raw_mag_x * (100.0f / 1100.0f);
    mag[1] = (float32_t)raw_mag_y * (100.0f / 1100.0f);
    mag[2] = (float32_t)raw_mag_z * (100.0f / 980.0f);

    float32_t field[3];

    for (int32_t index = 0; index < GY511_AXIS_COUNT; ++index)
    {
        field[index] = (mapped(mag, p_sensor->config.axis[index]) -
                        p_sensor->config.mag_offset_ut[index]) *
                       p_sensor->config.mag_scale[index];

        if (!isfinite(field[index]))
        {
            p_sensor->b_have_heading = false;
            return;
        }
    }

    p_data->magnetic_x_ut    = field[0];
    p_data->magnetic_y_ut    = field[1];
    p_data->magnetic_z_ut    = field[2];
    p_data->b_magnetic_valid = true;

    calculate_heading(p_sensor, p_data, norm, field, timestamp_ms);
}

/** @brief Read contiguous status/XYZ and pair before float calculations. */
static gy511_result_t
acceleration_bytes (gy511_t * p_sensor, imu_data_t * p_data, uint8_t p_bytes[6])
{
    uint8_t       packet[GY511_ACCEL_BYTES + 1U];
    uint32_t      before   = 0U;
    uint32_t      after    = 0U;
    gy511_bus_t * p_bus    = &p_sensor->bus;
    bool          b_before = read_clock(p_sensor, &before);

    /* DS7202: SUB(7) auto-increments; 0x27 status precedes XYZ 0x28..0x2D. */
    bool b_read  = p_bus->p_read(p_bus->p_context, ACC_ADDR, 0xA7U, packet,
                                 GY511_ACCEL_BYTES + 1U);
    bool b_after = read_clock(p_sensor, &after);

    read_duration(p_sensor, p_data, before, after, b_before && b_after, true);

    if (!b_read)
    {
        return GY511_ERROR;
    }

    if (0U == ((uint32_t)packet[0] & 0x08U))
    {
        return GY511_NO_DATA;
    }

    for (uint32_t index = 0U; index < GY511_ACCEL_BYTES; ++index)
    {
        p_bytes[index] = packet[index + 1U];
    }

    const bool b_time_valid =
        b_before && b_after && ((after - before) <= UINT32_MAX / 2U);

    if (b_time_valid)
    {
        /* Software read-completion time; hardware conversion time is unknown.
         */
        p_data->timestamp_ms = after;
    }

    if (NULL != p_sensor->capture.p_pair)
    {
        p_sensor->capture.p_pair(p_sensor->capture.p_context, p_data, before,
                                 after, b_time_valid);
    }

    /* A failed read-duration probe must remain visible after pairing reset. */
    if ((NULL != p_sensor->capture.p_clock) &&
        ((!b_before) || (!b_after) || ((after - before) > UINT32_MAX / 2U)))
    {
        p_data->odometry_reasons |= IMU_ODOM_REASON_PROBE;
    }

    return GY511_OK;
}

/** @brief Only fresh idle feedback authorizes blocking magnetic reads. */
static bool
magnetic_allowed (gy511_t * p_sensor, const imu_data_t * p_data)
{
    if (!p_sensor->b_magnetic_idle_only)
    {
        return true;
    }

    const imu_motion_t * p_motion = &p_data->motion;
    const bool b_idle = (p_motion->b_valid) && (p_motion->b_stopped) &&
                        (!p_motion->b_running) && (!p_motion->b_fault) &&
                        (0 == p_motion->left_speed_cps) &&
                        (0 == p_motion->right_speed_cps) &&
                        ((p_data->timestamp_ms - p_motion->timestamp_ms) <=
                         IMU_MOTION_MAX_AGE_MS);

    if (!b_idle)
    {
        p_sensor->b_idle_seen = false;
    }
    else if (!p_sensor->b_idle_seen)
    {
        p_sensor->b_idle_seen   = true;
        p_sensor->idle_since_ms = p_data->timestamp_ms;
    }
    else if ((p_data->timestamp_ms - p_sensor->idle_since_ms) >=
             GY511_IDLE_SETTLE_MS)
    {
        return true;
    }
    else
    {
        /* Hold off during initial standstill to protect final hump samples. */
    }

    p_sensor->b_have_heading = false;
    return false;
}

/** @brief Measure optional magnetic I2C work separately from acceleration. */
static bool
magnetic_bytes (gy511_t * p_sensor, imu_data_t * p_data, uint8_t p_bytes[6],
                uint32_t * p_time_ms)
{
    uint8_t       status   = 0U;
    uint32_t      before   = 0U;
    uint32_t      after    = 0U;
    gy511_bus_t * p_bus    = &p_sensor->bus;
    bool          b_before = read_clock(p_sensor, &before);
    bool          b_read =
        (p_bus->p_read(p_bus->p_context, MAG_ADDR, 0x09U, &status, 1U)) &&
        (0U != ((uint32_t)status & 1U)) &&
        (p_bus->p_read(p_bus->p_context, MAG_ADDR, 0x03U, p_bytes, 6U));
    bool b_after = read_clock(p_sensor, &after);

    read_duration(p_sensor, p_data, before, after, b_before && b_after, false);

    if (b_before && b_after && ((after - before) <= UINT32_MAX / 2U))
    {
        *p_time_ms = after;
    }

    return b_read;
}

/** @brief Keep the portable sensor decoder independent of the RTOS clock. */
static bool
read_clock (gy511_t * p_sensor, uint32_t * p_time_ms)
{
    return (NULL != p_sensor->capture.p_clock) &&
           (p_sensor->capture.p_clock(p_sensor->capture.p_context, p_time_ms));
}

/** @brief Store bounded read timings without manufacturing an unknown duration.
 */
static void
read_duration (gy511_t * p_sensor, imu_data_t * p_data, uint32_t begin_ms,
               uint32_t end_ms, bool b_time_valid, bool b_acceleration)
{
    if (NULL == p_sensor->capture.p_clock)
    {
        return;
    }

    const uint32_t elapsed = end_ms - begin_ms;

    if ((!b_time_valid) || (elapsed > UINT32_MAX / 2U))
    {
        p_data->odometry_reasons |= IMU_ODOM_REASON_PROBE;
        return;
    }

    /* Values above uint16_t range retain a lower bound, not a wrapped value. */
    const uint16_t duration =
        (elapsed > UINT16_MAX) ? UINT16_MAX : (uint16_t)elapsed;

    if (b_acceleration)
    {
        p_data->acceleration_read_ms = duration;
        p_data->odometry_time_valid |= IMU_READ_TIME_ACCEL;
    }
    else
    {
        p_data->magnetic_read_ms = duration;
        p_data->odometry_time_valid |= IMU_READ_TIME_MAG;
    }
}

/*** end of file ***/

