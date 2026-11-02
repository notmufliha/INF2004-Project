/** @file queue_stub.c
 * @brief Unused queue operations when testing startup without running tasks.
 */
#include "boot_stub.h"

/** @brief Startup does not execute the publisher. */
ER
tk_snd_mbf (ID h_buffer, const void * p_data, INT length, TMO timeout)
{
    (void)h_buffer;
    (void)p_data;
    (void)length;
    (void)timeout;
    return E_TMOUT;
}

/** @brief Startup does not execute the consumer. */
INT
tk_rcv_mbf (ID h_buffer, void * p_data, TMO timeout)
{
    (void)h_buffer;
    (void)p_data;
    (void)timeout;
    return E_TMOUT;
}

/*** end of file ***/
