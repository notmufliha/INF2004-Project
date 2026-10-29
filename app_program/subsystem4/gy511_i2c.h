#ifndef GY511_I2C_H
#define GY511_I2C_H
#include "drv_gy511.h"
/* Called only from the sampling task, exclusive owner of I2C0. */
bool gy511_i2c_open(gy511_bus_t *bus);
#endif
