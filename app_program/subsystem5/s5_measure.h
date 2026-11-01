#ifndef S5_MEASURE_H
#define S5_MEASURE_H
#include <stdbool.h>
/* True for echoes that should not be used to authorize driving or scanning.
 * This does not convert an ignored echo into a safe distance. */
bool s5_echo_suppressed_cm(float cm);
/* Returns true only if 3 of 5 non-near-field echoes agree (with one bounded retry). A false return
 * is UNKNOWN and must never be used as permission to move forward. */
bool s5_measure_stable_cm(float *out_cm);
#endif
