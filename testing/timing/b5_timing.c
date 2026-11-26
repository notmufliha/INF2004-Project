/* b5_timing.c - B5 scan result handed on once per obstacle scan.
 * b5_before : copy of the native S5Profile (56 B), the old hand-over.
 * b5_after  : s5_pack_profile into the 6-byte S5PackedProfile and
 *             s5_unpack_profile back, the new hand-over. */
#include "subsystem5/s5_packed.h"

static S5Profile profile, received;
static S5PackedProfile packed;
static S5DecodedProfile decoded;
volatile int result;

void b5_init(void)
{
	profile.valid = true;
	profile.size = S5_SIZE_SMALL;
	profile.nearest_cm = 24.6f;
	profile.nearest_forward_cm = 24.0f;
	profile.nearest_angle_deg = 90U;
	profile.left_boundary_deg = 80U;
	profile.right_boundary_deg = 100U;
	profile.estimated_width_cm = 12.3f;
	profile.frontal_width_cm = 12.0f;
	profile.left_clearance_cm = 30.2f;
	profile.right_clearance_cm = 9.4f;
}

void b5_before(void)
{
	received = profile;
	result = (int)received.nearest_cm;
}

void b5_after(void)
{
	result = (int)s5_pack_profile(&profile, &packed) +
		 (int)s5_unpack_profile(&packed, &decoded);
}
