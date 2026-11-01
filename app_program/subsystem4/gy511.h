/** @file gy511.h
 * @brief Buddy 4 drv gy511 interface.
 */
#ifndef GY511_H
#define GY511_H

#include "imu.h"

/* A successful read callback fills all count bytes before returning true.
 * Callbacks execute synchronously in the single sampling-task owner.
 */
typedef bool (*gy511_read_fn_t)(void * p_context, uint8_t address, uint8_t reg,
                                uint8_t * p_data, uint32_t count);
typedef bool (*gy511_write_fn_t)(void * p_context, uint8_t address, uint8_t reg,
                                 uint8_t value);

typedef struct
{
    void * p_context;

    gy511_read_fn_t  p_read;
    gy511_write_fn_t p_write;
} gy511_bus_t;

typedef struct
{
    /* Signed sensor axes for robot X/Y/Z: +1=X, -2=-Y, etc. */
    int32_t   axis[3];
    float32_t accel_offset_ms2[3];
    float32_t mag_offset_ut[3]; /* In robot axes, after axis mapping. */
    float32_t mag_scale[3];
    bool b_mag_calibrated; /* Enable only after magnetic calibration/trials. */
} gy511_config_t;

typedef struct
{
    gy511_bus_t    bus;
    gy511_config_t config;
    float32_t      previous_heading;
    uint32_t       previous_heading_ms;
    bool           b_have_heading;
    bool           b_ready;
} gy511_t;

typedef enum
{
    GY511_ERROR   = -1,
    GY511_NO_DATA = 0,
    GY511_OK      = 1
} gy511_result_t;

void gy511_default_config (gy511_config_t * p_config);
/* Checks magnetic H43 ID, configures and verifies both register banks.
 * The DLHC accelerometer has no documented WHO_AM_I register. */
bool           gy511_init (gy511_t * p_sensor, const gy511_bus_t * p_bus,
                           const gy511_config_t * p_config);
gy511_result_t gy511_read (gy511_t * p_sensor, uint32_t timestamp_ms,
                           imu_data_t * p_data);
/* Level, still startup average in robot axes; preserves 1g on Z. */
bool gy511_set_level_calibration (gy511_t *       p_sensor,
                                  const float32_t p_mean[3]);

#endif

/*** end of file ***/

