/** @file testnum_main.c
 * @brief Verify finite and bounded serial-output conversion.
 */
#include "testnum_main.h"
#include "imu_numeric.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief Verify truncation, saturation and invalid-input handling. */
int
main (void)
{
    assert(imu_numeric_is_finite(FLT_MAX));
    assert(!imu_numeric_is_finite(NAN));
    assert(!imu_numeric_is_finite(INFINITY));
    assert(12 == imu_numeric_to_int(12.75f));
    assert(-12 == imu_numeric_to_int(-12.75f));
    assert(0 == imu_numeric_to_int(NAN));
    assert(0 == imu_numeric_to_int(INFINITY));
    assert(INT32_MAX == imu_numeric_to_int(2147483648.0f));
    assert(INT32_MIN == imu_numeric_to_int(-2147483648.0f));
    assert(INT32_MIN == imu_numeric_to_int(-1.0e30f));
    puts(
        "PASS: numeric console conversion handles bounds and nonfinite values");
    return 0;
}

/*** end of file ***/

