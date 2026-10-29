/* LSM303DLHC DS7202 Rev 2: https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf */
#include "drv_gy511.h"
#include <math.h>
#include <stddef.h>

#define ACC_ADDR 0x19U
#define MAG_ADDR 0x1EU
#define GRAVITY 9.81f
#define RAD_TO_DEG 57.295779513f

void gy511_default_config(gy511_config_t *config)
{
    if (config != NULL) {
        *config = (gy511_config_t){{1,2,3}, {0,0,0}, {0,0,0}, {1,1,1}, false};
    }
}

static int axis_index(int axis) { return (axis < 0 ? -axis : axis) - 1; }

bool gy511_init(gy511_t *sensor, const gy511_bus_t *bus,
                 const gy511_config_t *config)
{
    uint8_t id[3], value;
    if (sensor == NULL) { return false; }
    *sensor = (gy511_t){0};
    if (bus == NULL || config == NULL || bus->read == NULL || bus->write == NULL) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (config->axis[i] == 0 || config->axis[i] < -3 || config->axis[i] > 3 ||
            !isfinite(config->accel_offset_ms2[i]) ||
            !isfinite(config->mag_offset_ut[i]) || !isfinite(config->mag_scale[i]) ||
            config->mag_scale[i] <= 0.0f) { return false; }
        for (int j = 0; j < i; ++j) {
            if (axis_index(config->axis[i]) == axis_index(config->axis[j])) { return false; }
        }
    }
    sensor->bus = *bus;
    sensor->config = *config;
    if (!bus->read(bus->context, MAG_ADDR, 0x0AU, id, 3U) ||
        id[0] != 0x48U || id[1] != 0x34U || id[2] != 0x33U) { return false; }
    /* 100 Hz XYZ enabled, normal mode; BDU + high-resolution +/-4g.
     * 12-bit left-justified data, sensitivity 2 mg/LSB. */
    const uint8_t address[] = {ACC_ADDR, ACC_ADDR, MAG_ADDR, MAG_ADDR, MAG_ADDR};
    const uint8_t reg[] = {0x20U, 0x23U, 0x00U, 0x01U, 0x02U};
    const uint8_t setting[] = {0x57U, 0x98U, 0x18U, 0x20U, 0x00U};
    for (size_t i = 0; i < sizeof(reg); ++i) {
        if (!bus->write(bus->context, address[i], reg[i], setting[i]) ||
            !bus->read(bus->context, address[i], reg[i], &value, 1U) ||
            value != setting[i]) { return false; }
    }
    sensor->ready = true;
    return true;
}

static int32_t signed_word(uint8_t high, uint8_t low)
{
    uint32_t word = ((uint32_t)high << 8) | low;
    return word >= 32768U ? (int32_t)word - 65536 : (int32_t)word;
}

static float mapped(const float values[3], int axis)
{
    return values[axis_index(axis)] * (axis < 0 ? -1.0f : 1.0f);
}

bool gy511_set_level_calibration(gy511_t *sensor, const float mean[3])
{
    if (sensor == NULL || mean == NULL || !sensor->ready ||
        !isfinite(mean[0]) || !isfinite(mean[1]) || !isfinite(mean[2]) ||
        fabsf(mean[0]) > 1.0f || fabsf(mean[1]) > 1.0f ||
        fabsf(mean[2] - GRAVITY) > 1.0f) { return false; }
    sensor->config.accel_offset_ms2[0] = mean[0];
    sensor->config.accel_offset_ms2[1] = mean[1];
    sensor->config.accel_offset_ms2[2] = mean[2] - GRAVITY;
    return true;
}

gy511_result_t gy511_read(gy511_t *sensor, uint32_t timestamp_ms, imu_data_t *data)
{
    uint8_t status, bytes[6];
    float accel[3], robot[3], mag[3];
    if (data == NULL) { return GY511_ERROR; }
    *data = (imu_data_t){0};
    data->timestamp_ms = timestamp_ms;
    if (sensor == NULL || !sensor->ready) { return GY511_ERROR; }
    gy511_bus_t *bus = &sensor->bus;
    if (!bus->read(bus->context, ACC_ADDR, 0x27U, &status, 1U)) { return GY511_ERROR; }
    if (!(status & 0x08U)) { return GY511_NO_DATA; }
    if (!bus->read(bus->context, ACC_ADDR, 0xA8U, bytes, 6U)) { return GY511_ERROR; }
    for (int i = 0; i < 3; ++i) {
        int32_t raw = signed_word(bytes[2*i+1], bytes[2*i]);
        /* Reject clipping; divide left-aligned 12-bit signed readings by 16. */
        if (raw >= 32752 || raw <= -32768) { return GY511_ERROR; }
        accel[i] = (float)(raw / 16) * 0.002f * GRAVITY;
    }
    for (int i = 0; i < 3; ++i) {
        robot[i] = mapped(accel, sensor->config.axis[i]) - sensor->config.accel_offset_ms2[i];
    }
    data->accel_x = robot[0]; data->accel_y = robot[1]; data->accel_z = robot[2];
    float norm = hypotf(hypotf(robot[0], robot[1]), robot[2]);
    if (!isfinite(norm) || norm < 0.1f) { return GY511_ERROR; }
    data->pitch_deg = atan2f(-robot[0], hypotf(robot[1], robot[2])) * RAD_TO_DEG;
    data->roll_deg = atan2f(robot[1], robot[2]) * RAD_TO_DEG;
    data->valid = true;
    /* Magnetometer runs at 75 Hz. No fresh magnetic sample is not an IMU fault. */
    if (!bus->read(bus->context, MAG_ADDR, 0x09U, &status, 1U) || !(status & 1U) ||
        !bus->read(bus->context, MAG_ADDR, 0x03U, bytes, 6U)) { return GY511_OK; }
    int32_t mx = signed_word(bytes[0], bytes[1]);
    int32_t mz = signed_word(bytes[2], bytes[3]);
    int32_t my = signed_word(bytes[4], bytes[5]);
    if (mx == -4096 || my == -4096 || mz == -4096) { return GY511_OK; }
    mag[0] = (float)mx * (100.0f / 1100.0f);
    mag[1] = (float)my * (100.0f / 1100.0f);
    mag[2] = (float)mz * (100.0f / 980.0f);
    float field[3];
    for (int i = 0; i < 3; ++i) {
        field[i] = (mapped(mag, sensor->config.axis[i]) - sensor->config.mag_offset_ut[i]) *
                   sensor->config.mag_scale[i];
    }
    data->magnetic_x_ut = field[0]; data->magnetic_y_ut = field[1]; data->magnetic_z_ut = field[2];
    data->magnetic_valid = true;
    /* Tilt compensation only near 1g; acceleration makes the tilt estimate unreliable. */
    if (fabsf(norm - GRAVITY) > 1.5f) { sensor->have_heading = false; return GY511_OK; }
    float pitch = data->pitch_deg / RAD_TO_DEG, roll = data->roll_deg / RAD_TO_DEG;
    float horizontal_x = field[0]*cosf(pitch) + field[1]*sinf(roll)*sinf(pitch) +
                         field[2]*cosf(roll)*sinf(pitch);
    float horizontal_y = field[1]*cosf(roll) - field[2]*sinf(roll);
    if (!isfinite(horizontal_x) || !isfinite(horizontal_y) ||
        hypotf(horizontal_x, horizontal_y) < 1.0f) { sensor->have_heading = false; return GY511_OK; }
    data->heading_deg = atan2f(-horizontal_y, horizontal_x) * RAD_TO_DEG;
    if (data->heading_deg < 0.0f) { data->heading_deg += 360.0f; }
    data->heading_valid = true;
    data->heading_calibrated = sensor->config.mag_calibrated;
    uint32_t dt = timestamp_ms - sensor->previous_heading_ms;
    if (sensor->config.mag_calibrated && sensor->have_heading && dt > 0U && dt <= 100U) {
        float delta = data->heading_deg - sensor->previous_heading;
        if (delta > 180.0f) { delta -= 360.0f; }
        if (delta < -180.0f) { delta += 360.0f; }
        data->turn_rate_dps = delta * 1000.0f / (float)dt;
        data->turn_rate_valid = true;
    }
    sensor->previous_heading = data->heading_deg;
    sensor->previous_heading_ms = timestamp_ms;
    sensor->have_heading = true;
    return GY511_OK;
}
