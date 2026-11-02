#include "subsystem5.h"
#include "../subsystem2/b2_motor.h"

EXPORT ER subsystem5_init(void)
{
    /* B2 is initialized before B5 in robot mode. In Wednesday's
     * sensor-only demonstration, keep the motor safety latch closed
     * from initialization, not just after the B5 task starts. */
#if S5_DEMO_SENSOR_ONLY
    b2_request_stop();
#endif
    return E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID tid = app_start_task((FP)s5_task, S5_PRI_MAIN, 0, "s5_task");
    return (tid < E_OK) ? tid : E_OK;
}
