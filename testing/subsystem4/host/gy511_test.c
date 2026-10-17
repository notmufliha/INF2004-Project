#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "drv_gy511.h"
#include "imu_analysis.h"

static uint8_t acc[256], mag[256];
static bool fail;
static bool read_fake(void *ctx, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t n)
{
    (void)ctx;
    if (fail) { return false; }
    assert(addr == 0x19U || addr == 0x1EU);
    uint8_t *bank = addr == 0x19U ? acc : mag;
    if (addr == 0x19U && n > 1U) { assert(reg & 0x80U); }
    memcpy(data, bank + (reg & 0x7FU), n);
    return true;
}
static bool write_fake(void *ctx, uint8_t addr, uint8_t reg, uint8_t value)
{
    (void)ctx;
    if (fail) { return false; }
    (addr == 0x19U ? acc : mag)[reg] = value;
    return true;
}
static void word(uint8_t *bank, int index, int value, bool little)
{
    uint16_t u = (uint16_t)value;
    bank[index + (little ? 0 : 1)] = (uint8_t)u;
    bank[index + (little ? 1 : 0)] = (uint8_t)(u >> 8);
}
static void fixture(void)
{
    memset(acc, 0, sizeof(acc)); memset(mag, 0, sizeof(mag)); fail = false;
    mag[10] = 0x48; mag[11] = 0x34; mag[12] = 0x33;
    acc[0x27] = 8; mag[9] = 1;
    word(acc, 0x2C, 500*16, true); /* +1g at +/-4g, 2mg/LSB. */
    word(mag, 3, 1100, false); /* X=100uT; order is X,Z,Y. */
    word(mag, 5, 490, false); /* Z=50uT. */
    word(mag, 7, 0, false);
}
int main(void)
{
    gy511_t sensor;
    gy511_config_t config;
    gy511_bus_t bus = {NULL, read_fake, write_fake};
    imu_data_t data;
    fixture(); gy511_default_config(&config);
    assert(gy511_init(&sensor, &bus, &config));
    assert(acc[0x20] == 0x57 && acc[0x23] == 0x98 && mag[0] == 0x18);
    assert(gy511_read(&sensor, 10U, &data) == GY511_OK);
    assert(fabsf(data.accel_z - 9.81f) < 0.001f);
    assert(fabsf(data.pitch_deg) < 0.01f && fabsf(data.roll_deg) < 0.01f);
    assert(fabsf(data.magnetic_x_ut - 100.0f) < 0.01f);
    assert(fabsf(data.magnetic_z_ut - 50.0f) < 0.01f);
    assert(data.heading_valid && !data.heading_calibrated && !data.turn_rate_valid);
    assert(gy511_read(&sensor, 20U, &data) == GY511_OK && !data.turn_rate_valid);
    puts("PASS: configuration, signed conversion, X/Z/Y ordering and honest availability");
    word(acc, 0x28, -250*16, true); word(acc, 0x2C, 433*16, true);
    assert(gy511_read(&sensor, 30U, &data) == GY511_OK);
    assert(fabsf(data.pitch_deg - 30.0f) < 0.1f);
    acc[0x27] = 0;
    assert(gy511_read(&sensor, 40U, &data) == GY511_NO_DATA && !data.valid);
    acc[0x27] = 8;
    word(acc, 0x28, -32768, true);
    assert(gy511_read(&sensor, 50U, &data) == GY511_ERROR && !data.valid);
    fail = true;
    assert(gy511_read(&sensor, 60U, &data) == GY511_ERROR && !data.valid);
    fixture(); mag[10] = 0;
    assert(!gy511_init(&sensor, &bus, &config));
    assert(gy511_read(&sensor, 70U, &data) == GY511_ERROR);
    puts("PASS: tilt, freshness, clipping, bus failures and identity mismatch");
    fixture(); config.axis[0] = -2; config.axis[1] = 1;
    assert(gy511_init(&sensor, &bus, &config));
    word(acc, 0x2A, 50*16, true);
    assert(gy511_read(&sensor, 80U, &data) == GY511_OK && data.accel_x < -0.9f);
    float level[3] = {0.1f,-0.2f,9.91f};
    assert(gy511_set_level_calibration(&sensor, level));
    level[2] = -9.81f;
    assert(!gy511_set_level_calibration(&sensor, level));
    config.axis[1] = 2;
    assert(!gy511_init(&sensor, &bus, &config));
    puts("PASS: mounting map validation and gravity-preserving level calibration");
    fixture(); gy511_default_config(&config); config.mag_calibrated = true;
    assert(gy511_init(&sensor, &bus, &config));
    assert(gy511_read(&sensor, 100U, &data) == GY511_OK && !data.turn_rate_valid);
    word(mag, 7, -110, false);
    assert(gy511_read(&sensor, 120U, &data) == GY511_OK && data.turn_rate_valid);
    assert(data.turn_rate_dps > 280.0f && data.turn_rate_dps < 290.0f);
    assert(gy511_read(&sensor, 300U, &data) == GY511_OK && !data.turn_rate_valid);
    imu_analysis_t analysis; imu_analysis_config_t ac; imu_event_t event;
    imu_analysis_default_config(&ac); assert(imu_analysis_init(&analysis, &ac));
    data.turn_rate_dps = 100.0f; data.turn_rate_valid = false;
    assert(imu_analysis_process(&analysis, &data, &event));
    assert(event.motion_state != IMU_MOTION_TURNING && !event.turn_rate_valid);
    puts("PASS: calibrated heading derivative and unavailable-turn classification");
    return 0;
}
