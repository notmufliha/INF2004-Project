/** @file imu_numeric.h
 * @brief Buddy 4 imu numeric interface.
 */
#ifndef IMU_NUMERIC_H
#define IMU_NUMERIC_H

#include <float.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>

/* C99 has no float32_t typedef. Name and verify the target representation. */
typedef float float32_t;

/* A negative array bound rejects an incompatible representation in C99. */
typedef uint8_t
    imu_numeric_size_check_t[(32U == (sizeof(float32_t) * CHAR_BIT)) ? 1 : -1];
#if (FLT_RADIX != 2) || (FLT_MANT_DIG != 24) || (FLT_MAX_EXP != 128)
#error B4 requires binary32 floating-point precision
#endif

int32_t imu_numeric_to_int (float32_t value);
bool    imu_numeric_is_finite (float32_t value);

#endif

/*** end of file ***/

