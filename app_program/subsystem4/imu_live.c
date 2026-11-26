/** @file imu_live.c
 * @brief Latest raw IMU sample, shared between the IMU sampling task and
 *        the Vehicle Controller (lift-stop).  Copies are made with
 *        interrupts disabled: the record is a few words, single core.
 */
#include "../app.h"
#include "imu_live.h"

static imu_live_t g_live;

/** @brief Save a valid raw sample atomically for the lift-stop consumer. */
void
imu_live_store (const imu_data_t * p_data)
{
    UINT irq;

    if ((NULL == p_data) || (!p_data->b_valid))
    {
        return;
    }
    DI(irq);
    g_live.accel_x      = p_data->accel_x;
    g_live.accel_y      = p_data->accel_y;
    g_live.accel_z      = p_data->accel_z;
    g_live.pitch_deg    = p_data->pitch_deg;
    g_live.timestamp_ms = p_data->timestamp_ms;
    g_live.b_valid      = true;
    EI(irq);
}

/** @brief Copy the latest sample while excluding its acquisition writer. */
void
imu_live_get (imu_live_t * p_out)
{
    UINT irq;

    if (NULL == p_out)
    {
        return;
    }
    DI(irq);
    *p_out = g_live;
    EI(irq);
}

/*** end of file ***/

