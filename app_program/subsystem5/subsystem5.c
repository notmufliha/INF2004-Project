#include "subsystem5.h"

EXPORT ER subsystem5_init(void)
{
    /* No kernel objects required for the isolated first milestone. */
    return E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID tid = app_start_task((FP)s5_task, S5_PRI_MAIN, 0, "s5_task");
    return (tid < E_OK) ? tid : E_OK;
}
