/* b4_timing.c - B4 per-sample pipeline (filter + terrain/motion analysis),
 * compiled against the tree before (34f8528) and after (d4c6499) the
 * minimum-bit change.  The change only narrows header types, so the same
 * harness builds against both.  b4_20_samples runs 20 samples at 10 ms
 * over a small hump (pitch 0 -> 6 -> 0 deg) while driving forward. */
#include "subsystem4/imu_processing.h"
#include "subsystem4/imu_analysis.h"

static imu_processing_t proc;
static imu_analysis_t ana;
volatile int result;

void b4_init(void)
{
	imu_processing_config_t pc;
	imu_analysis_config_t ac;
	imu_processing_default_config(&pc);
	imu_analysis_default_config(&ac);
	result = (int)imu_processing_init(&proc, &pc) +
		 (int)imu_analysis_init(&ana, &ac);
}

void b4_20_samples(void)
{
	static const float pitch[20] = { 0, 0, 1, 2, 4, 6, 6, 5, 3, 1,
					 0, -1, -2, -1, 0, 0, 0, 0, 0, 0 };
	imu_data_t in, out;
	imu_processing_health_t health;
	imu_event_t ev;
	int events = 0;

	for (int i = 0; i < 20; i++) {
		in = (imu_data_t){0};
		in.timestamp_ms = 1000U + 10U * (uint32_t)i;
		in.accel_x = 0.05f; in.accel_z = 1.0f;
		in.pitch_deg = pitch[i]; in.roll_deg = 0.2f;
		in.sample_sequence = (uint32_t)i;
		in.encoder_left = 100 * i; in.encoder_right = 100 * i;
		in.b_valid = true; in.b_odometry_valid = true;
		if (imu_processing_process(&proc, &in, in.timestamp_ms, &out,
					   &health) &&
		    imu_analysis_process(&ana, &out, &ev)) {
			events++;
		}
	}
	result = events;
}
