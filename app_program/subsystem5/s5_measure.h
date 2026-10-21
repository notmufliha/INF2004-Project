#ifndef S5_MEASURE_H
#define S5_MEASURE_H
#include <stdbool.h>
/* Three independent echoes, reject out-of-range values. At least 2 valid
 * echoes required. Returns median of 3 or average of 2. Caller owns sensor. */
bool s5_measure_stable_cm(float *out_cm);
#endif
