/** @file imu_numeric.c
 * @brief Buddy 4 imu numeric module.
 */
#include "imu_numeric.h"
#include <math.h>

/** @brief Check finiteness without exposing libc headers to kernel modules. */
bool
imu_numeric_is_finite (float32_t value)
{
    return 0 != isfinite(value);
}

/** @brief Convert a finite display value to int32_t, clamping its range. */
int32_t
imu_numeric_to_int (float32_t value)
{
    if (!isfinite(value))
    {
        return 0;
    }

    if (value > 2147483520.0f)
    {
        return INT32_MAX;
    }

    if (value < -2147483648.0f)
    {
        return INT32_MIN;
    }

    /* Bounds above are binary32 values inside the int32_t range. */
    return (int32_t)value;
}

/*** end of file ***/

