#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "imu_processing.h"
#include "imu_analysis.h"

static imu_processing_t state;
static imu_processing_config_t config;
static imu_data_t raw, out;
static imu_health_t health;

static void reset(void)
{
    imu_processing_default_config(&config);
    assert(imu_processing_init(&state, &config));
    raw = (imu_data_t){0};
    raw.valid = true;
    raw.accel_z = 9.81f;
}

static bool step(void)
{
    raw.timestamp_ms += 10U;
    return imu_processing_process(&state, &raw, raw.timestamp_ms, &out, &health);
}

int main(void)
{
    reset();
    assert(step());
    raw.pitch_deg = 12.0f;
    raw.roll_deg = -12.0f;
    raw.turn_rate_dps = 40.0f;
    raw.accel_x = 15.0f;
    assert(step());
    assert(out.pitch_deg == 3.0f && out.roll_deg == -3.0f);
    assert(out.turn_rate_dps == 10.0f && out.accel_x == 15.0f);
    imu_analysis_t analysis;
    imu_analysis_config_t ac;
    imu_event_t event;
    imu_analysis_default_config(&ac);
    assert(imu_analysis_init(&analysis, &ac));
    assert(imu_analysis_process(&analysis, &out, &event) && event.sudden_impact);
    puts("PASS: smoothing preserves an immediate impact spike");

    reset();
    raw.pitch_deg = 179.0f;
    assert(step());
    raw.pitch_deg = -179.0f;
    assert(step());
    assert(out.pitch_deg > 179.0f);
    puts("PASS: angles wrap along the shortest path");

    reset();
    assert(step());
    raw.valid = false;
    assert(!step() && !health.fault_active && !out.valid);
    assert(!step() && !health.fault_active);
    assert(!step() && health.fault_started && health.fault_active);
    assert(!step() && !health.fault_started && health.fault_active);
    raw.valid = true;
    for (unsigned int i = 0; i < 4U; ++i) {
        assert(!step() && health.fault_active);
    }
    raw.pitch_deg = 20.0f;
    assert(step() && health.recovered && !health.fault_active);
    assert(out.pitch_deg == 20.0f); /* Reseeded, no stale filter history. */
    assert(step() && !health.recovered);
    puts("PASS: fault latching, recovery suppression and filter reseeding");

    reset();
    assert(step());
    raw.timestamp_ms += 101U;
    assert(!step() && health.fault_started && health.fault_active);
    assert(!step());
    raw.valid = false;
    assert(!step()); /* Interrupt recovery. */
    raw.valid = true;
    for (unsigned int i = 0; i < 4U; ++i) { assert(!step()); }
    assert(step() && health.recovered);
    assert(!imu_processing_process(&state, NULL, raw.timestamp_ms + 10U, &out, &health));
    assert(!out.valid);
    assert(!imu_processing_process(&state, &raw, raw.timestamp_ms, &out, &health));
    assert(health.fault_started); /* Duplicate timestamp. */
    puts("PASS: missing data, gaps, duplicate timestamps and interrupted recovery");

    reset();
    raw.pitch_deg = NAN;
    assert(!step());
    raw.pitch_deg = 181.0f;
    assert(!step());
    raw.pitch_deg = 0.0f;
    raw.accel_x = INFINITY;
    assert(!step() && health.fault_started);
    config.smoothing_alpha = 0.0f;
    assert(!imu_processing_init(&state, &config));
    assert(!imu_processing_process(NULL, &raw, 0U, &out, &health));
    puts("PASS: nonfinite readings, angle bounds and invalid configuration");

    reset();
    raw.timestamp_ms = UINT32_MAX - 20U;
    assert(step());
    assert(step());
    assert(step()); /* Clock wraps normally. */
    raw.timestamp_ms -= 20U;
    assert(!step() && health.fault_started);
    puts("PASS: timestamp wrap and backward-clock fault");
    puts("All IMU processing tests passed.");
    return 0;
}
