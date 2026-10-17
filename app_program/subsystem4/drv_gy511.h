#ifndef DRV_GY511_H
#define DRV_GY511_H

#include "imu_types.h"

typedef bool (*gy511_read_fn)(void *, uint8_t, uint8_t, uint8_t *, uint32_t);
typedef bool (*gy511_write_fn)(void *, uint8_t, uint8_t, uint8_t);
typedef struct
{
    void *context;
    gy511_read_fn read;
    gy511_write_fn write;
} gy511_bus_t;

typedef struct
{
    /* Signed sensor axes for robot X/Y/Z: +1=X, -2=-Y, etc. */
    int axis[3];
    float accel_offset_ms2[3];
    float mag_offset_ut[3]; /* In robot axes, after axis mapping. */
    float mag_scale[3];
    bool mag_calibrated; /* Enable only after magnetic calibration/trials. */
} gy511_config_t;

typedef struct
{
    gy511_bus_t bus;
    gy511_config_t config;
    float previous_heading;
    uint32_t previous_heading_ms;
    bool have_heading;
    bool ready;
} gy511_t;

typedef enum { GY511_ERROR = -1, GY511_NO_DATA = 0, GY511_OK = 1 } gy511_result_t;

void gy511_default_config(gy511_config_t *config);
/* Checks magnetic H43 ID, configures and verifies both register banks.
 * The DLHC accelerometer has no documented WHO_AM_I register. */
bool gy511_init(gy511_t *sensor, const gy511_bus_t *bus,
                 const gy511_config_t *config);
gy511_result_t gy511_read(gy511_t *sensor, uint32_t timestamp_ms, imu_data_t *data);
/* Level, still startup average in robot axes; preserves 1g on Z. */
bool gy511_set_level_calibration(gy511_t *sensor, const float mean[3]);

#endif
