/** @file testlift_main.c
 * @brief Verify B4 lift detection: floor motion ignored, pick-ups caught.
 */
#include "testlift_main.h"
#include "imu_lift.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_G (9.81f)
#define TEST_PERIOD_MS 10U
#define TEST_SEEDS 5U          /* vibration runs with different noise    */
#define TEST_JOLTS 30U         /* tape/hump-edge jolts in one run        */
#define TEST_LIFT_SAMPLES 40U  /* one pick-up: up then slowing, 10 ms    */
#define TEST_LIFT_HALF 20U     /* ...the accelerating half              */
#define TEST_HOLD_SAMPLES 200U /* still in the hand after the pick-up    */

static imu_lift_t g_lift;
static imu_data_t g_data;
static uint32_t   g_noise_state = 12345U; /* xorshift32 state */

static void reset (void);
static float32_t noise (float32_t amplitude);
static bool feed (float32_t up_ms2,
    float32_t               side_ms2,
    uint32_t                count,
    float32_t               noise_ms2,
    float32_t               scale);
static void test_floor (void);
static void test_lifts (void);
static void test_hold_and_guards (void);

/** @brief Run floor rejection, lift detection, hold and input guards. */
int
main (void)
{
    test_floor();
    test_lifts();
    test_hold_and_guards();
    puts("All lift detection tests passed.");
    return 0;
}

/** @brief Fresh detector and a level, still car. */
static void
reset (void)
{
    imu_lift_config_t config;

    imu_lift_default_config(&config);
    assert(imu_lift_init(&g_lift, &config));
    g_data              = (imu_data_t){0};
    g_data.b_valid      = true;
    g_data.timestamp_ms = 1000U;
}

/** @brief Deterministic uniform noise in [-amplitude, +amplitude]. */
static float32_t
noise (float32_t amplitude)
{
    g_noise_state ^= g_noise_state << 13;
    g_noise_state ^= g_noise_state >> 17;
    g_noise_state ^= g_noise_state << 5;
    return amplitude * (((float32_t)(g_noise_state >> 8) / 8388607.5f) - 1.0f);
}

/** @brief Feed samples: upward and sideways acceleration on top of 1 g
 *         (times scale: accelerometer gain error), plus vibration noise.
 *         Returns true if any sample reported lifted. */
static bool
feed (float32_t up_ms2,
    float32_t   side_ms2,
    uint32_t    count,
    float32_t   noise_ms2,
    float32_t   scale)
{
    bool b_any = false;

    for (uint32_t index = 0U; index < count; ++index)
    {
        g_data.timestamp_ms += TEST_PERIOD_MS;
        g_data.accel_x = (side_ms2 + noise(noise_ms2)) * scale;
        g_data.accel_y = noise(noise_ms2) * scale;
        g_data.accel_z = (TEST_G + up_ms2 + noise(noise_ms2)) * scale;
        b_any          = (imu_lift_step(&g_lift, &g_data)) || (b_any);
    }
    return b_any;
}

/** @brief Nothing on the floor counts as a lift. */
static void
test_floor (void)
{
    reset();
    /* Standing still with a 3 % accelerometer gain error: the learned
     * baseline absorbs it (a fixed 1 g would integrate 0.3 m/s^2). */
    assert(!feed(0.0f, 0.0f, 1000U, 0.0f, 1.03f));

    /* Driving: heavy broadband vibration, uniform +-2 m/s^2 on every axis
     * (0.12 g rms; real wheel/motor vibration is mostly above 10 Hz and
     * far less of it reaches the integrators), 60 s, several seeds. */
    for (uint32_t seed = 1U; seed <= TEST_SEEDS; ++seed)
    {
        g_noise_state = seed * 7919U;
        reset();
        assert(!feed(0.0f, 0.0f, 6000U, 2.0f, 1.0f));
    }

    /* Speeding up and braking at 1.5 m/s^2 (sideways in robot axes). */
    reset();
    feed(0.0f, 0.0f, 100U, 0.5f, 1.0f);
    assert(!feed(0.0f, 1.5f, 80U, 0.5f, 1.0f));
    assert(!feed(0.0f, -1.5f, 80U, 0.5f, 1.0f));

    /* A wheel climbing a 10 mm hump over 0.3 s and coming down. */
    reset();
    feed(0.0f, 0.0f, 100U, 0.5f, 1.0f);
    assert(!feed(0.45f, 0.0f, 15U, 0.5f, 1.0f));
    assert(!feed(-0.45f, 0.0f, 15U, 0.5f, 1.0f));
    assert(!feed(-0.45f, 0.0f, 15U, 0.5f, 1.0f));
    assert(!feed(0.45f, 0.0f, 15U, 0.5f, 1.0f));

    /* Junction tape and hump edges every 0.3 s: a 20 ms jolt of 8 m/s^2
     * up, then down again (the car ends where it was). */
    reset();
    feed(0.0f, 0.0f, 100U, 0.5f, 1.0f);
    for (uint32_t index = 0U; index < TEST_JOLTS; ++index)
    {
        assert(!feed(8.0f, 0.0f, 2U, 0.5f, 1.0f));
        assert(!feed(-8.0f, 0.0f, 2U, 0.5f, 1.0f));
        assert(!feed(0.0f, 0.0f, 26U, 0.5f, 1.0f));
    }
    puts("PASS: still with gain error, heavy vibration, speed changes, hump "
         "and tape jolts are not lifts");
}

/** @brief Hand pick-ups are caught quickly, gently or tilted. */
static void
test_lifts (void)
{
    bool     b_lifted;
    uint32_t start_ms;
    uint32_t taken_ms = 0U;

    /* Typical pick-up: 3 m/s^2 up for 0.2 s, then slowing (~12 cm). */
    reset();
    feed(0.0f, 0.0f, 100U, 0.5f, 1.0f);
    start_ms = g_data.timestamp_ms;
    for (uint32_t index = 0U; (index < TEST_LIFT_SAMPLES) && (0U == taken_ms);
        ++index)
    {
        if (feed(index < TEST_LIFT_HALF ? 3.0f : -3.0f, 0.0f, 1U, 0.5f, 1.0f))
        {
            taken_ms = g_data.timestamp_ms - start_ms;
        }
    }
    printf("   typical pick-up detected after %u ms\n", (unsigned)taken_ms);
    assert((taken_ms > 0U) && (taken_ms <= 250U));

    /* Gentle lift: 1.5 m/s^2 for 0.3 s then slowing (~13 cm). */
    reset();
    feed(0.0f, 0.0f, 100U, 0.5f, 1.0f);
    b_lifted = feed(1.5f, 0.0f, 30U, 0.5f, 1.0f);
    b_lifted = (feed(-1.5f, 0.0f, 30U, 0.5f, 1.0f)) || (b_lifted);
    assert(b_lifted);

    /* Lifted while driving (vibration still on), car tilted 30 degrees:
     * the size of the acceleration does not depend on orientation. */
    reset();
    feed(0.0f, 0.0f, 200U, 1.0f, 1.0f);
    b_lifted = false;
    for (uint32_t index = 0U; index < TEST_LIFT_SAMPLES; ++index)
    {
        float32_t up_ms2 = index < TEST_LIFT_HALF ? 3.0f : -3.0f;
        g_data.timestamp_ms += TEST_PERIOD_MS;
        g_data.accel_x = (TEST_G + up_ms2) * 0.5f; /* sin 30 */
        g_data.accel_y = noise(1.0f);
        g_data.accel_z = (TEST_G + up_ms2) * 0.8660254f; /* cos 30 */
        b_lifted       = (imu_lift_step(&g_lift, &g_data)) || (b_lifted);
    }
    assert(b_lifted);
    puts("PASS: typical, gentle and tilted pick-ups (also while driving)");
}

/** @brief Hold time, single start pulse, re-arm and invalid input. */
static void
test_hold_and_guards (void)
{
    imu_lift_config_t config;
    uint32_t          starts         = 0U;
    uint32_t          lifted_samples = 0U;

    reset();
    feed(0.0f, 0.0f, 100U, 0.0f, 1.0f);
    for (uint32_t index = 0U; index < TEST_LIFT_SAMPLES; ++index)
    {
        feed(index < TEST_LIFT_HALF ? 3.0f : -3.0f, 0.0f, 1U, 0.0f, 1.0f);
        starts += imu_lift_started(&g_lift, &g_data) ? 1U : 0U;
    }
    for (uint32_t index = 0U; index < TEST_HOLD_SAMPLES; ++index)
    {
        lifted_samples += feed(0.0f, 0.0f, 1U, 0.0f, 1.0f) ? 1U : 0U;
        starts += imu_lift_started(&g_lift, &g_data) ? 1U : 0U;
    }
    assert(1U == starts);                      /* one detection per lift      */
    assert(lifted_samples > 50U);              /* held after the motion stops */
    assert(!feed(0.0f, 0.0f, 1U, 0.0f, 1.0f)); /* then released */

    /* Re-armed: a second pick-up is caught again. */
    for (uint32_t index = 0U; index < TEST_LIFT_SAMPLES; ++index)
    {
        feed(index < TEST_LIFT_HALF ? 3.0f : -3.0f, 0.0f, 1U, 0.0f, 1.0f);
        starts += imu_lift_started(&g_lift, &g_data) ? 1U : 0U;
    }
    assert(2U == starts);

    /* Invalid and late samples restart the motion; never lift by themselves. */
    reset();
    feed(0.0f, 0.0f, 100U, 0.0f, 1.0f);
    for (uint32_t index = 0U; index < TEST_LIFT_SAMPLES; ++index)
    {
        feed(index < TEST_LIFT_HALF ? 3.0f : -3.0f, 0.0f, 1U, 0.0f, 1.0f);
        if (0U == (index % 4U))
        {
            g_data.timestamp_ms += 500U; /* sample gap */
        }
    }
    assert(!g_lift.b_lifted);
    g_data.b_valid = false;
    assert(!imu_lift_step(&g_lift, &g_data));
    g_data.b_valid = true;
    g_data.accel_z = NAN;
    assert(!imu_lift_step(&g_lift, &g_data));
    assert(!imu_lift_step(NULL, &g_data));
    assert(!imu_lift_step(&g_lift, NULL));

    imu_lift_default_config(&config);
    config.rise_m = 0.0f;
    assert(!imu_lift_init(&g_lift, &config));
    imu_lift_default_config(&config);
    config.hold_ms = 0U;
    assert(!imu_lift_init(&g_lift, &config));
    assert(!imu_lift_init(NULL, &config));
    puts("PASS: one detection per lift, hold then release, re-arm, gaps, "
         "invalid data and configuration");
}

/*** end of file ***/

