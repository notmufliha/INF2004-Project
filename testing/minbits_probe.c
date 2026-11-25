/* minbits_probe.c - Cortex-M0+ sizes of the B3 and B5 minimum-bit objects.
 *
 * Measurement object only (never linked into the firmware).  Compile it with
 * the firmware's own flags and read the symbol sizes:
 *
 *   sh testing/measure_minbits.sh
 *
 * Each measured_* array is exactly sizeof() of the object it names.
 */
#include <stdbool.h>
#include <stdint.h>
#include "subsystem3/b3_line_barcode.h"
#include "subsystem5/s5_packed.h"

/* B3_STATE's six fields as ordinary 32-bit members (the unpacked design). */
typedef struct {
	int ir_left;
	int ir_centre;
	int ir_right;
	int line_flags;
	int last_barcode;
	int last_nav_command;
} b3_wide_state_t;

#define MEASURE(name, type) unsigned char measured_##name[sizeof(type)]

MEASURE(b3_state, B3_STATE);
MEASURE(b3_wide_state, b3_wide_state_t);
MEASURE(s5_packed_profile, S5PackedProfile);
MEASURE(s5_decoded_profile, S5DecodedProfile);
MEASURE(s5_native_profile, S5Profile);
