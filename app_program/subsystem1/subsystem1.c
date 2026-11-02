/** @file subsystem1.c
 * @brief Subsystem 1, WiFi Communication, Command & Telemetry: creates
 * the kernel objects (init) and starts the tasks (start).  The objects
 * and their users are listed in s1_private.h.
 *
 * APIs shown: tk_cre_mbf, tk_cre_mtx (TA_INHERIT), tk_cre_flg,
 *             tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem1.h"
#include "s1_private.h"
#include "s1_comm.h"
#include "s1_led.h"
#include "s1_demo.h"
#include "s1_test.h"

ID tid_led        = 0; /* IF-05 (template example)   */
ID mbf_remote_cmd = 0; /* I1.3 IF-S1-CMD             */

ID gh_s1_comm_task = 0;
ID gh_s1_led_cyc   = 0;
ID gh_s1_event_mbf = 0;
ID gh_s1_tx_mbf    = 0;
ID gh_s1_rx_mbf    = 0;
ID gh_s1_telem_mtx = 0;
ID gh_s1_comm_flg  = 0;

static ID make_mbf (uint32_t buf_bytes, uint32_t max_msg_bytes);
static ER make_sync_objects (void);

/** @brief Create every subsystem 1 kernel object (runs before any task).
 *
 * @return E_OK, or the first kernel error.
 */
ER
subsystem1_init (void)
{
    ER result = E_OK;

    gh_s1_event_mbf = make_mbf(EVENT_QUEUE_LEN *
                               ((uint32_t)sizeof(comm_pack_event_t) +
                                MBF_MSG_OVERHEAD),
                               (uint32_t)sizeof(comm_pack_event_t));
    gh_s1_tx_mbf    = make_mbf(TX_BUF_SIZE, TX_RECORD_MAX);
    gh_s1_rx_mbf    = make_mbf(RX_BUF_SIZE, RX_RECORD_MAX);
    mbf_remote_cmd  = make_mbf(REMOTE_CMD_QUEUE_LEN *
                               ((uint32_t)sizeof(comm_remote_cmd_t) +
                                MBF_MSG_OVERHEAD),
                               (uint32_t)sizeof(comm_remote_cmd_t));

    if (gh_s1_event_mbf < E_OK)
    {
        result = gh_s1_event_mbf;
    }
    else if (gh_s1_tx_mbf < E_OK)
    {
        result = gh_s1_tx_mbf;
    }
    else if (gh_s1_rx_mbf < E_OK)
    {
        result = gh_s1_rx_mbf;
    }
    else if (mbf_remote_cmd < E_OK)
    {
        result = mbf_remote_cmd;
    }
    else
    {
        result = make_sync_objects();
    }

    return (result);
}

/** @brief Start the comm and LED tasks (and the demo, if built in). */
ER
subsystem1_start (void)
{
    ER result = E_OK;

    /* Entry points are cast to the kernel's generic FP type; the kernel
     * calls them with (INT stacd, void *exinf), matching their type. */
    gh_s1_comm_task = app_start_task((FP)s1_comm_task, S1_PRI_COMM, 0U,
                                     "comm_task");
    tid_led         = app_start_task((FP)s1_led_task, S1_PRI_LED, 0U,
                                     "led_task");

    if (gh_s1_comm_task < E_OK)
    {
        result = gh_s1_comm_task;
    }
    else if (tid_led < E_OK)
    {
        result = tid_led;
    }
    else
    {
        result = s1_demo_start();
    }

    if (E_OK == result)
    {
        result = tk_sta_cyc(gh_s1_led_cyc);
    }

    return (result);
}

#if APP_TEST
/** @brief On-target tests (make TEST=1); see s1_test.c. */
void
subsystem1_test (void)
{
    s1_test_run();
}
#endif

/** @brief Create a FIFO message buffer. */
static ID
make_mbf (uint32_t buf_bytes, uint32_t max_msg_bytes)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    /* Sizes are a few hundred bytes at most: they fit SZ and INT. */
    cmbf.bufsz  = (SZ)buf_bytes;
    cmbf.maxmsz = (INT)max_msg_bytes;

    return (tk_cre_mbf(&cmbf));
}

/** @brief Create the mutex, the two event flags and the LED cyclic. */
static ER
make_sync_objects (void)
{
    T_CMTX cmtx   = {0};
    T_CFLG cflg   = {0};
    T_CFLG cfault = {0};
    T_CCYC ccyc   = {0};
    ER     result = E_OK;

    /* TA_INHERIT: if a high-priority sensor task waits for the snapshot
     * while the low-priority comm task holds it, the comm task is
     * raised, so the wait is as short as possible (no inversion). */
    cmtx.mtxatr     = TA_INHERIT;
    gh_s1_telem_mtx = tk_cre_mtx(&cmtx);

    cflg.flgatr    = TA_TFIFO | TA_WSGL; /* only the comm task waits */
    cflg.iflgptn   = 0U;
    gh_s1_comm_flg = tk_cre_flg(&cflg);

    /* Fault bitmask: any task may wait on it (TA_WMUL), all clear. */
    cfault.flgatr  = TA_TFIFO | TA_WMUL;
    cfault.iflgptn = 0U;
    flg_faults     = tk_cre_flg(&cfault);

    ccyc.cycatr   = TA_HLNG;
    /* The kernel calls the handler as void (*)(void *exinf). */
    ccyc.cychdr   = (FP)s1_led_cyclic_isr;
    ccyc.cyctim   = S1_TICK_MS;
    ccyc.cycphs   = S1_TICK_MS;
    gh_s1_led_cyc = tk_cre_cyc(&ccyc);

    if (gh_s1_telem_mtx < E_OK)
    {
        result = gh_s1_telem_mtx;
    }
    else if (gh_s1_comm_flg < E_OK)
    {
        result = gh_s1_comm_flg;
    }
    else if (flg_faults < E_OK)
    {
        result = flg_faults;
    }
    else if (gh_s1_led_cyc < E_OK)
    {
        result = gh_s1_led_cyc;
    }
    else
    {
        result = E_OK;
    }

    return (result);
}

/*** end of file ***/

