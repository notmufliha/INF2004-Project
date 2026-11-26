/* b3_timing.c - B3 shared snapshot: one line-sensor update (writer, every
 * sample) and one full read by the controller.  "After" is the real packed
 * B3_STATE; "before" (-DB3_BEFORE) is the same six fields as 32-bit ints,
 * the plain design it replaces.  The mutex calls are the same in both and
 * are left out. */
#include "subsystem3/b3_line_barcode.h"

#ifdef B3_BEFORE
typedef struct {
	int ir_left, ir_centre, ir_right, line_flags, last_barcode,
	    last_nav_command;
} STATE;
#else
typedef B3_STATE STATE;
#endif

static STATE state;
volatile int result;
static volatile unsigned adc_l = 812U, adc_c = 3010U, flags = 1U;

void b3_write_line(void)
{
	state.ir_left = adc_l;
	state.ir_centre = adc_c;
	state.ir_right = 1U;
	state.line_flags = flags;
}

void b3_read_all(void)
{
	STATE copy = state;
	result = (int)copy.ir_left + (int)copy.ir_centre + (int)copy.ir_right +
		 (int)copy.line_flags + (int)copy.last_barcode +
		 (int)copy.last_nav_command;
}
