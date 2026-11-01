/** @file testgyro_main.c
 * @brief Buddy 4 testgyro main module.
 */
#include "testgyro_main.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "gy511.h"
#include "imu_analysis.h"

static uint8_t g_acc[256];
static uint8_t g_mag[256];
static bool    gb_fail;

static bool read_fake (void * p_ctx, uint8_t addr, uint8_t reg,
                       uint8_t * p_data, uint32_t count);

static bool write_fake (void * p_ctx, uint8_t addr, uint8_t reg, uint8_t value);

static void word (uint8_t * p_bank, int32_t index, int32_t value,
                  bool b_little);

static void fixture (void);

static void test_magnetic_overflow (void);

/** @brief Main. */
int
main (void)
{
    test_magnetic_overflow();

    gy511_t        sensor;
    gy511_config_t config;
    gy511_bus_t    bus = {NULL, read_fake, write_fake};
    imu_data_t     data;

    fixture();
    gy511_default_config(&config);
    assert(gy511_init(&sensor, &bus, &config));
    assert((0x57 == g_acc[0x20]) && (0x98 == g_acc[0x23]) &&
           (0x18 == g_mag[0]));
    assert(GY511_OK == gy511_read(&sensor, 10U, &data));
    assert(fabsf(data.accel_z - 9.81f) < 0.001f);
    assert((fabsf(data.pitch_deg) < 0.01f) && (fabsf(data.roll_deg) < 0.01f));
    assert(fabsf(data.magnetic_x_ut - 100.0f) < 0.01f);
    assert(fabsf(data.magnetic_z_ut - 50.0f) < 0.01f);
    assert((data.b_heading_valid) && (!data.b_heading_calibrated) &&
           (!data.b_turn_rate_valid));
    assert((GY511_OK == gy511_read(&sensor, 20U, &data)) &&
           (!data.b_turn_rate_valid));
    puts("PASS: configuration, signed conversion, X/Z/Y ordering and honest "
         "availability");
    word(g_acc, 0x28, -250 * 16, true);
    word(g_acc, 0x2C, 433 * 16, true);
    assert(GY511_OK == gy511_read(&sensor, 30U, &data));
    assert(fabsf(data.pitch_deg - 30.0f) < 0.1f);
    g_acc[0x27] = 0;
    assert((GY511_NO_DATA == gy511_read(&sensor, 40U, &data)) &&
           (!data.b_valid));
    g_acc[0x27] = 8;
    word(g_acc, 0x28, -32768, true);
    assert((GY511_ERROR == gy511_read(&sensor, 50U, &data)) && (!data.b_valid));
    gb_fail = true;
    assert((GY511_ERROR == gy511_read(&sensor, 60U, &data)) && (!data.b_valid));
    fixture();
    g_mag[10] = 0;
    assert(!gy511_init(&sensor, &bus, &config));
    assert(GY511_ERROR == gy511_read(&sensor, 70U, &data));
    puts("PASS: tilt, freshness, clipping, bus failures and identity mismatch");
    fixture();
    config.axis[0] = -2;
    config.axis[1] = 1;
    assert(gy511_init(&sensor, &bus, &config));
    word(g_acc, 0x2A, 50 * 16, true);
    assert((GY511_OK == gy511_read(&sensor, 80U, &data)) &&
           (data.accel_x < -0.9f));

    float32_t level_mean[3] = {0.1f, -0.2f, 9.91f};

    assert(gy511_set_level_calibration(&sensor, level_mean));
    level_mean[2] = -9.81f;
    assert(!gy511_set_level_calibration(&sensor, level_mean));
    config.axis[1] = 2;
    assert(!gy511_init(&sensor, &bus, &config));
    puts("PASS: mounting map validation and gravity-preserving level_mean "
         "calibration");
    fixture();
    gy511_default_config(&config);
    config.b_mag_calibrated = true;
    assert(gy511_init(&sensor, &bus, &config));
    assert((GY511_OK == gy511_read(&sensor, 100U, &data)) &&
           (!data.b_turn_rate_valid));
    word(g_mag, 7, -110, false);
    assert((GY511_OK == gy511_read(&sensor, 120U, &data)) &&
           (data.b_turn_rate_valid));
    assert((data.turn_rate_dps > 280.0f) && (data.turn_rate_dps < 290.0f));
    assert((GY511_OK == gy511_read(&sensor, 300U, &data)) &&
           (!data.b_turn_rate_valid));

    imu_analysis_t        analysis;
    imu_analysis_config_t analysis_config;
    imu_event_t           event;

    imu_analysis_default_config(&analysis_config);
    assert(imu_analysis_init(&analysis, &analysis_config));
    data.turn_rate_dps     = 100.0f;
    data.b_turn_rate_valid = false;
    assert(imu_analysis_process(&analysis, &data, &event));
    assert((event.motion_state != IMU_MOTION_TURNING) &&
           (!event.b_turn_rate_valid));
    puts("PASS: calibrated heading derivative and unavailable-turn "
         "classification");
    return 0;
}

/** @brief Reject overflowed magnetic calibration without losing acceleration.
 */
static void
test_magnetic_overflow (void)
{
    gy511_t           sensor;
    gy511_config_t    config;
    const gy511_bus_t bus = {NULL, read_fake, write_fake};
    imu_data_t        data;

    fixture();
    gy511_default_config(&config);
    config.b_mag_calibrated = true;
    assert(gy511_init(&sensor, &bus, &config));
    assert(GY511_OK == gy511_read(&sensor, 10U, &data));
    sensor.config.mag_scale[0] = FLT_MAX;
    assert(GY511_OK == gy511_read(&sensor, 20U, &data));
    assert((data.b_valid) && (!data.b_magnetic_valid) &&
           (!data.b_heading_valid) && (!data.b_turn_rate_valid));
    sensor.config.mag_scale[0] = 1.0f;
    assert(GY511_OK == gy511_read(&sensor, 30U, &data));
    assert((data.b_heading_valid) && (!data.b_turn_rate_valid));
    puts("PASS: magnetic overflow is unavailable and resets heading history");
}

/** @brief Read fake. */
static bool
read_fake (void * p_ctx, uint8_t addr, uint8_t reg, uint8_t * p_data,
           uint32_t count)
{
    (void)p_ctx;

    if (gb_fail)
    {
        return false;
    }

    assert((0x19U == addr) || (0x1EU == addr));

    uint8_t * p_bank = 0x19U == addr ? g_acc : g_mag;

    if ((0x19U == addr) && (count > 1U))
    {
        /* Exact byte-to-word widening avoids signed integer promotion. */
        assert(0U != ((uint32_t)reg & 0x80U));
    }

    /* A byte widens exactly; masking gives a valid register index. */
    memcpy(p_data, p_bank + ((uint32_t)reg & 0x7FU), count);
    return true;
}

/** @brief Write fake. */
static bool
write_fake (void * p_ctx, uint8_t addr, uint8_t reg, uint8_t value)
{
    (void)p_ctx;

    if (gb_fail)
    {
        return false;
    }

    (0x19U == addr ? g_acc : g_mag)[reg] = value;
    return true;
}

/** @brief Word. */
static void
word (uint8_t * p_bank, int32_t index, int32_t value, bool b_little)
{
    /* Conversion to unsigned wraps modulo 2^16, preserving the test word.
     * Widen to uint32_t so shifts never operate on promoted signed data. */
    uint32_t packed_word = (uint16_t)value;

    /* Masking each byte bounds both narrowing casts to 0..255. */
    p_bank[index + (b_little ? 0 : 1)] = (uint8_t)(packed_word & 0xFFU);
    p_bank[index + (b_little ? 1 : 0)] = (uint8_t)(packed_word >> 8);
}

/** @brief Fixture. */
static void
fixture (void)
{
    memset(g_acc, 0, sizeof(g_acc));
    memset(g_mag, 0, sizeof(g_mag));
    gb_fail     = false;
    g_mag[10]   = 0x48;
    g_mag[11]   = 0x34;
    g_mag[12]   = 0x33;
    g_acc[0x27] = 8;
    g_mag[9]    = 1;
    word(g_acc, 0x2C, 500 * 16, true); /* +1g at +/-4g, 2mg/LSB. */
    word(g_mag, 3, 1100, false);       /* X=100uT; order is X,Z,Y. */
    word(g_mag, 5, 490, false);        /* Z=50uT. */
    word(g_mag, 7, 0, false);
}

/*** end of file ***/

