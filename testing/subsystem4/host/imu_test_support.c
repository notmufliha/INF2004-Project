/** @file imu_test_support.c
 * @brief Compare finite test values within the test tolerance.
 */
#include "imu_test_support.h"
#include <math.h>

/** @brief Test whether two finite values differ by at most 0.0001. */
bool
imu_test_support_near (float32_t actual, float32_t expected)
{
    return ((isfinite(actual)) && (isfinite(expected)) &&
            ((fabsf(actual - expected) <= 0.0001f)));
}

/*** end of file ***/

