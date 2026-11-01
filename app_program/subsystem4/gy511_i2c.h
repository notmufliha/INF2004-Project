/** @file gy511_i2c.h
 * @brief Buddy 4 gy511 i2c interface.
 */
#ifndef GY511_I2C_H
#define GY511_I2C_H
#include "gy511.h"
/* Called only from the sampling task, exclusive owner of I2C0. */
bool gy511_i2c_open (gy511_bus_t * p_bus);
#endif

/*** end of file ***/

