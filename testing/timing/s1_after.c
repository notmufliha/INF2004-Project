/* s1_after.c - B1 telemetry build AFTER Change Requirement 1 (30-byte
 * binary frame, comm_pack_telemetry).  Same typical values as s1_before.c. */
#include "subsystem1/comm_pack.h"

static uint8_t frame[WIRE_TELEM_LEN];
static comm_telemetry_t t;
volatile int32_t result;

void s1_after_init(void)
{
	t.robot_state = 2U; t.line_flags = 1U; t.ir_right = 1U;
	t.ir_left = 812U; t.ir_centre = 3010U;
	t.speed_left = 300U; t.speed_right = 296U; t.motion = 0U;
	t.enc_left = 41230U; t.enc_right = 40987U; t.barcode = 1U; t.nav = 3U;
	t.pitch_ddeg = (uint32_t)(-12) & 0x3FFU; t.hump_valid = 1U;
	t.hump_last_mm = 23U; t.hump_peak_mm = 31U;
	t.obs_dist_cm = 25U; t.obs_width_cm = 12U;
	t.obs_left_cm = 30U; t.obs_right_cm = 9U; t.roll_ddeg = 5U;
}

void s1_after_telemetry(void)
{
	result = comm_pack_telemetry(frame, sizeof(frame), 42U, 2100U, 0U, &t);
}
